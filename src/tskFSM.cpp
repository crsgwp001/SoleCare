#include "tskFSM.h"
#include <Arduino.h>
#include <StateMachine.h>
#include <Events.h>
#include "global.h"
#include "fsm_debug.h"
#include "tskUV.h"
#include "tskMotor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Button definitions (configurable in include/config.h)
static constexpr int START_PIN = HW_START_PIN;
static constexpr int RESET_PIN = HW_RESET_PIN;

// Timing/config
static constexpr uint32_t DEBOUNCE_MS = 300u;
static constexpr uint32_t FSM_LOOP_DELAY_MS = 50u;
static constexpr size_t   FSM_QUEUE_LEN = 8;

// Instantiate Global + two Sub-FSMs
static StateMachine<GlobalState, Event> fsmGlobal(GlobalState::Idle);
static StateMachine<SubState,   Event> fsmSub1  (SubState::S_IDLE);
static StateMachine<SubState,   Event> fsmSub2  (SubState::S_IDLE);

// Sensor equalize timeout (milliseconds)
static constexpr uint32_t SENSOR_EQUALIZE_MS = 6u * 1000u; // 6s
// timestamp when we entered Detecting (0 = not active)
static uint32_t detectingStartMs = 0;
// Event queue for serializing FSM events
struct EventMsg { Event ev; bool broadcastAll; };
static QueueHandle_t g_fsmEventQ = nullptr;

// forward declaration
static bool fsmPostEvent(Event ev, bool broadcastAll);

// Track which subs have reached S_DONE. Bit0 = sub1, Bit1 = sub2.
static uint8_t g_subDoneMask = 0;
// timestamp when we entered Done (0 = not active). After 60s -> auto-reset
static uint32_t g_doneStartMs = 0;
// per-sub timestamps for managing time-based transitions
static uint32_t g_subWetStartMs[2] = {0,0};
// g_subDryerStartMs removed; use g_subWetStartMs for wet runtime and g_subCoolingStartMs for cooling
static uint32_t g_subCoolingStartMs[2] = {0,0};
// Track whether the UV timer expired while the sub was in the COOLING phase.
static bool g_uvExpiredDuringCooling[2] = { false, false };
// UV complete flag: set when a UV timer has finished for a sub. Prevents
// restarting the UV timer on subsequent loops between cooling->wet.
static bool g_uvComplete[2] = { false, false };
// Protect g_uvComplete access since it's read from UI (core 0) and written by FSM (core 1)
// (g_uvComplete is internal to FSM)

static void markSubDone(int idx) {
  if (idx == 0) g_subDoneMask |= 1u;
  else if (idx == 1) g_subDoneMask |= 2u;
  if (g_subDoneMask == 0x03) fsmPostEvent(Event::SubFSMDone, false);
}

// Helper to post an event to the FSM queue (safe from task context)
static bool fsmPostEvent(Event ev, bool broadcastAll = false) {
  if (!g_fsmEventQ) return false;
  // duplicate-guard for Start/SubStart
  static Event lastEv = Event::None;
  static uint32_t lastEvMs = 0;
  if (ev == Event::StartPressed || ev == Event::SubStart) {
    uint32_t now = millis();
    if (lastEv == ev && (uint32_t)(now - lastEvMs) < DEBOUNCE_MS) return false;
    lastEv = ev; lastEvMs = now;
  }
  EventMsg m{ev, broadcastAll};
  return xQueueSend(g_fsmEventQ, &m, 0) == pdTRUE;
}

// Allow external tasks to post events to the FSM queue
bool fsmExternalPost(Event ev) { return fsmPostEvent(ev, false); }

static bool readStart() {
  static bool last = HIGH; static uint32_t t0 = 0;
  bool raw = digitalRead(START_PIN);
  if (raw != last && (millis() - t0) > DEBOUNCE_MS) { t0 = millis(); last = raw; return raw == LOW; }
  return false;
}

static bool readReset() {
  static bool last = HIGH; static uint32_t t0 = 0;
  bool raw = digitalRead(RESET_PIN);
  if (raw != last && (millis() - t0) > DEBOUNCE_MS) { t0 = millis(); last = raw; return raw == LOW; }
  return false;
}

// Variadic broadcast helper
template<typename StateT, typename EventT>
static void broadcast(EventT ev, StateMachine<StateT, EventT>& first) { first.handleEvent(ev); }
template<typename StateT, typename EventT, typename... Fs>
static void broadcast(EventT ev, StateMachine<StateT, EventT>& first, Fs&... rest) { first.handleEvent(ev); broadcast<StateT, EventT>(ev, rest...); }

//------------------------------------------------------------------------------
// Configure all transitions and run-callbacks
//------------------------------------------------------------------------------
static void setupStateMachines() {
  // Transition tables
  Transition<GlobalState, Event> global_trans[] = {
    { GlobalState::Idle,      Event::StartPressed, GlobalState::Detecting, [](){ FSM_DBG_PRINTLN("GLOBAL: Detecting start"); } },
    { GlobalState::Detecting, Event::SensorTimeout, GlobalState::Checking,  [](){ FSM_DBG_PRINTLN("GLOBAL: Sensor timeout -> Checking"); } },
    { GlobalState::Checking,  Event::StartPressed, GlobalState::Running,   [](){ FSM_DBG_PRINTLN("GLOBAL: Checking -> Running"); } },
    { GlobalState::Running,   Event::SubFSMDone,    GlobalState::Done,      [](){ FSM_DBG_PRINTLN("GLOBAL: All subs done -> Done"); } },
    { GlobalState::Idle,      Event::Error,        GlobalState::Error,     [](){ FSM_DBG_PRINTLN("GLOBAL: Error"); } },
  };

  Transition<SubState, Event> sub1_trans[] = {
    { SubState::S_IDLE,     Event::Shoe0InitWet,   SubState::S_WET,      [](){ FSM_DBG_PRINTLN("SUB1: entered S_WET"); } },
    { SubState::S_IDLE,     Event::Shoe0InitDry,   SubState::S_DRY,      [](){ FSM_DBG_PRINTLN("SUB1: entered S_DRY"); } },
    // New flow: S_WET -> S_COOLING (heater off, motor continues) -> S_DRY
    { SubState::S_WET,      Event::SubStart,       SubState::S_COOLING,  [](){ FSM_DBG_PRINTLN("SUB1: Wet→Cooling (SubStart)"); } },
    { SubState::S_COOLING,  Event::SubStart,       SubState::S_DRY,      [](){ FSM_DBG_PRINTLN("SUB1: Cooling→Dry (SubStart)"); } },
    { SubState::S_DRY,      Event::SubStart,       SubState::S_DONE,     [](){ FSM_DBG_PRINTLN("SUB1: Dry→Done (SubStart)"); markSubDone(0); } },
    // If dry-check fails, return to S_WET to run dryer again
    { SubState::S_DRY,      Event::DryCheckFailed, SubState::S_WET,      [](){ FSM_DBG_PRINTLN("SUB1: Dry->Wet (DryCheckFailed)"); } },
  };

  Transition<SubState, Event> sub2_trans[] = {
    { SubState::S_IDLE,     Event::Shoe1InitWet,   SubState::S_WET,      [](){ FSM_DBG_PRINTLN("SUB2: entered S_WET"); } },
    { SubState::S_IDLE,     Event::Shoe1InitDry,   SubState::S_DRY,      [](){ FSM_DBG_PRINTLN("SUB2: entered S_DRY"); } },
    { SubState::S_WET,      Event::SubStart,       SubState::S_COOLING,  [](){ FSM_DBG_PRINTLN("SUB2: Wet→Cooling (SubStart)"); } },
    { SubState::S_COOLING,  Event::SubStart,       SubState::S_DRY,      [](){ FSM_DBG_PRINTLN("SUB2: Cooling→Dry (SubStart)"); } },
    { SubState::S_DRY,      Event::SubStart,       SubState::S_DONE,     [](){ FSM_DBG_PRINTLN("SUB2: Dry→Done (SubStart)"); markSubDone(1); } },
    { SubState::S_DRY,      Event::DryCheckFailed, SubState::S_WET,      [](){ FSM_DBG_PRINTLN("SUB2: Dry->Wet (DryCheckFailed)"); } },
    { SubState::S_DONE,     Event::SubStart,       SubState::S_DONE,     [](){ /* end */ } }
  };

  // Register transitions
  for (auto &t : global_trans) fsmGlobal.addTransition(t);
  for (auto &t : sub1_trans)  fsmSub1.addTransition(t);
  for (auto &t : sub2_trans)  fsmSub2.addTransition(t);

  // Make ResetPressed a catch-all: Global -> Idle
  for (uint8_t s = 0; s < static_cast<uint8_t>(GlobalState::Count); ++s) {
    fsmGlobal.addTransition({ static_cast<GlobalState>(s), Event::ResetPressed, GlobalState::Idle, [](){ FSM_DBG_PRINTLN("GLOBAL: Reset (Idle)"); } });
  }
  // Reset for subs -> S_IDLE
  for (int s = 0; s <= static_cast<int>(SubState::S_DONE); ++s) {
    fsmSub1.addTransition({ static_cast<SubState>(s), Event::ResetPressed, SubState::S_IDLE, [](){ FSM_DBG_PRINTLN("SUB1: Reset (S_Idle)"); g_subDoneMask &= ~1u; } });
    fsmSub2.addTransition({ static_cast<SubState>(s), Event::ResetPressed, SubState::S_IDLE, [](){ FSM_DBG_PRINTLN("SUB2: Reset (S_Idle)"); g_subDoneMask &= ~2u; } });
  }

  // Run callback for Running: poll subs
  fsmGlobal.setRun(GlobalState::Running, [](){ fsmSub1.run(); fsmSub2.run(); });

  // Running entry: initialize subs (post init events; do not auto-kick SubStart)
  fsmGlobal.setEntry(GlobalState::Running, [](){
  // use boolean wet flags populated by tskDHT
  bool s1Wet = g_dhtIsWet[0];
  bool s2Wet = g_dhtIsWet[1];
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Running - initializing subs");
    fsmPostEvent(s1Wet ? Event::Shoe0InitWet : Event::Shoe0InitDry, false);
    fsmPostEvent(s2Wet ? Event::Shoe1InitWet : Event::Shoe1InitDry, false);
  });

  // Register entry/exit callbacks
  struct StateCBG { GlobalState s; void(*entry)(); void(*exit)(); };
  struct StateCBS { SubState s; void(*entry)(); void(*exit)(); };

  static StateCBG global_cbs[] = {
  { GlobalState::Running, [](){ bool s1Wet = g_dhtIsWet[0]; bool s2Wet = g_dhtIsWet[1]; FSM_DBG_PRINTLN("GLOBAL ENTRY: Running - initializing subs"); g_uvComplete[0] = g_uvComplete[1] = false; fsmPostEvent(s1Wet ? Event::Shoe0InitWet : Event::Shoe0InitDry, false); fsmPostEvent(s2Wet ? Event::Shoe1InitWet : Event::Shoe1InitDry, false); }, nullptr },
    { GlobalState::Detecting, [](){ detectingStartMs = millis(); FSM_DBG_PRINTLN("GLOBAL ENTRY: Detecting - equalize timer started"); }, [](){ detectingStartMs = 0; FSM_DBG_PRINTLN("GLOBAL EXIT: Detecting - equalize timer cleared"); } },
  { GlobalState::Done, [](){ FSM_DBG_PRINTLN("GLOBAL ENTRY: Done - resetting subs (local)"); fsmSub1.handleEvent(Event::ResetPressed); fsmSub2.handleEvent(Event::ResetPressed); uvStop(0); uvStop(1); g_subDoneMask = 0; g_uvComplete[0] = g_uvComplete[1] = false; g_doneStartMs = millis(); }, [](){ g_doneStartMs = 0; } }
  };

  static StateCBS sub1_cbs[] = {
    // S_WET: start motor + heater and run for WET_TIMEOUT_MS then transition to COOLING
    { SubState::S_WET,
      [](){ FSM_DBG_PRINTLN("SUB1 ENTRY: WET"); motorStart(0); /* heater on during wet */ heaterRun(0, true); g_subWetStartMs[0] = millis(); },
      [](){ FSM_DBG_PRINTLN("SUB1 EXIT: leaving WET"); /* leaving wet */ } },
    // S_COOLING: heater off, motor continues; start cooling timer
    { SubState::S_COOLING,
      [](){ FSM_DBG_PRINTLN("SUB1 ENTRY: COOLING"); /* turn heater off, motor kept running */ heaterRun(0, false);
        // On cooling entry ensure UV is running for the cooling period: if it
        // was paused resume it; if it wasn't started, start it. Clear any
        // previous 'expired during cooling' marker.
        g_uvExpiredDuringCooling[0] = false;
        if (!g_uvComplete[0]) {
          if (uvIsPaused(0)) { FSM_DBG_PRINTLN("SUB1 ENTRY: COOLING -> resuming UV"); uvResume(0); }
          else if (!uvIsStarted(0)) { FSM_DBG_PRINTLN("SUB1 ENTRY: COOLING -> starting UV"); uvStart(0, 0); }
        } else {
          FSM_DBG_PRINTLN("SUB1 ENTRY: COOLING -> UV already complete, not starting");
        }
        g_subCoolingStartMs[0] = millis(); },
      [](){ FSM_DBG_PRINTLN("SUB1 EXIT: leaving COOLING"); /* leaving cooling */ } },
    // S_DRY: on entry stop motor; if UV already expired (timer fired during cooling)
    // then the sub will be advanced by the UVTimer event. If starting from DRY, start
    // UV here only after an immediate dry-check (to avoid skipping the UV duration).
    { SubState::S_DRY,
      [](){ FSM_DBG_PRINTLN("SUB1 ENTRY: DRY"); motorStop(0);
        // If UV expired while in COOLING, we should advance immediately.
        if (g_uvExpiredDuringCooling[0]) {
          FSM_DBG_PRINTLN("SUB1 ENTRY: DRY -> UV already expired during COOLING, advancing");
          g_uvExpiredDuringCooling[0] = false;
          fsmSub1.handleEvent(Event::SubStart);
          return;
        }
        // If sensor reads wet now, go back to WET.
        if (g_dhtIsWet[0]) {
          FSM_DBG_PRINTLN("SUB1 ENTRY: DRY -> sensor wet, returning to WET");
          // Ensure UV is not running while we go back to wet
          uvPause(0);
          fsmSub1.handleEvent(Event::DryCheckFailed);
          return;
        }
        // Otherwise ensure UV is running (start or resume) and wait for UVTimer to advance
        if (!g_uvComplete[0]) {
          if (uvIsPaused(0)) { FSM_DBG_PRINTLN("SUB1 ENTRY: DRY -> resuming UV"); uvResume(0); }
          else if (!uvIsStarted(0)) { FSM_DBG_PRINTLN("SUB1 ENTRY: DRY -> starting UV"); uvStart(0, 0); }
        } else {
          FSM_DBG_PRINTLN("SUB1 ENTRY: DRY -> UV already complete, advancing");
          fsmSub1.handleEvent(Event::SubStart);
        }
      },
      nullptr }
  };

  static StateCBS sub2_cbs[] = {
    { SubState::S_WET,
      [](){ FSM_DBG_PRINTLN("SUB2 ENTRY: WET"); motorStart(1); /* heater on during wet */ heaterRun(1, true); g_subWetStartMs[1] = millis(); },
      [](){ FSM_DBG_PRINTLN("SUB2 EXIT: leaving WET"); } },
    { SubState::S_COOLING,
      [](){ FSM_DBG_PRINTLN("SUB2 ENTRY: COOLING"); heaterRun(1, false);
        g_uvExpiredDuringCooling[1] = false;
        if (!g_uvComplete[1]) {
          if (uvIsPaused(1)) { FSM_DBG_PRINTLN("SUB2 ENTRY: COOLING -> resuming UV"); uvResume(1); }
          else if (!uvIsStarted(1)) { FSM_DBG_PRINTLN("SUB2 ENTRY: COOLING -> starting UV"); uvStart(1, 0); }
        } else {
          FSM_DBG_PRINTLN("SUB2 ENTRY: COOLING -> UV already complete, not starting");
        }
        g_subCoolingStartMs[1] = millis(); },
      [](){ FSM_DBG_PRINTLN("SUB2 EXIT: leaving COOLING"); } },
    { SubState::S_DRY,
      [](){ FSM_DBG_PRINTLN("SUB2 ENTRY: DRY"); motorStop(1);
        // If UV expired while in COOLING, advance immediately.
        if (g_uvExpiredDuringCooling[1]) {
          FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> UV already expired during COOLING, advancing");
          g_uvExpiredDuringCooling[1] = false;
          fsmSub2.handleEvent(Event::SubStart);
          return;
        }
        // If sensor reads wet now, go back to WET.
        if (g_dhtIsWet[1]) {
          FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> sensor wet, returning to WET");
          uvPause(1);
          fsmSub2.handleEvent(Event::DryCheckFailed);
          return;
        }
        // Otherwise ensure UV is running (start or resume) and wait for UVTimer.
        if (!g_uvComplete[1]) {
          if (uvIsPaused(1)) { FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> resuming UV"); uvResume(1); }
          else if (!uvIsStarted(1)) { FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> starting UV"); uvStart(1, 0); }
        } else {
          FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> UV already complete, advancing");
          fsmSub2.handleEvent(Event::SubStart);
        }
      },
      nullptr }
  };

  for (auto &c : global_cbs) { if (c.entry) fsmGlobal.setEntry(c.s, c.entry); if (c.exit) fsmGlobal.setExit(c.s, c.exit); }
  for (auto &c : sub1_cbs)   { if (c.entry) fsmSub1.setEntry(c.s, c.entry); if (c.exit)  fsmSub1.setExit(c.s, c.exit); }
  for (auto &c : sub2_cbs)   { if (c.entry) fsmSub2.setEntry(c.s, c.entry); if (c.exit)  fsmSub2.setExit(c.s, c.exit); }

  // Per-state run callbacks to handle time-based progression and checks
  // S_WET run: run wet period (heater+motor) for WET_TIMEOUT_MS -> advance to COOLING
  fsmSub1.setRun(SubState::S_WET, [](){ if (g_subWetStartMs[0] != 0 && (uint32_t)(millis() - g_subWetStartMs[0]) >= WET_TIMEOUT_MS) { FSM_DBG_PRINTLN("SUB1: WET timeout -> advance to COOLING"); g_subWetStartMs[0] = 0; fsmSub1.handleEvent(Event::SubStart); } });
  fsmSub2.setRun(SubState::S_WET, [](){ if (g_subWetStartMs[1] != 0 && (uint32_t)(millis() - g_subWetStartMs[1]) >= WET_TIMEOUT_MS) { FSM_DBG_PRINTLN("SUB2: WET timeout -> advance to COOLING"); g_subWetStartMs[1] = 0; fsmSub2.handleEvent(Event::SubStart); } });

  // S_COOLING run: motor continues, heater is off; after DRY_COOL_MS -> perform dry-check
  // If dry-check passes -> advance to DRY. If it fails -> pause UV and go back to WET.
  fsmSub1.setRun(SubState::S_COOLING, [](){
    if (g_subCoolingStartMs[0] == 0) return;
    if ((uint32_t)(millis() - g_subCoolingStartMs[0]) < DRY_COOL_MS) return;
    FSM_DBG_PRINTLN("SUB1: COOLING timeout -> advance to DRY");
    g_subCoolingStartMs[0] = 0;
    // Always advance to DRY when the cooling timer expires. The DRY
    // entry will perform the sensor dry-check and either return to WET
    // (pausing UV) or allow UV to continue.
    fsmSub1.handleEvent(Event::SubStart);
  });

  fsmSub2.setRun(SubState::S_COOLING, [](){
    if (g_subCoolingStartMs[1] == 0) return;
    if ((uint32_t)(millis() - g_subCoolingStartMs[1]) < DRY_COOL_MS) return;
    FSM_DBG_PRINTLN("SUB2: COOLING timeout -> advance to DRY");
    g_subCoolingStartMs[1] = 0;
    // Always advance to DRY; DRY entry decides whether to return to WET or
    // continue with UV.
    fsmSub2.handleEvent(Event::SubStart);
  });

  // S_DRY: on entry, motor should be turned off. If UV already finished during cooling
  // advance immediately. If UV isn't running (starting-from-DRY scenario), perform a
  // quick dry-check and start UV; otherwise wait for UV timer to complete.
  fsmSub1.setRun(SubState::S_DRY, [](){ /* no-op: advancement driven by UV timer or entry checks */ });

  // Detecting entry/exit
  fsmGlobal.setEntry(GlobalState::Detecting, [](){ detectingStartMs = millis(); FSM_DBG_PRINTLN("GLOBAL ENTRY: Detecting - equalize timer started"); });
  fsmGlobal.setExit(GlobalState::Detecting,  [](){ detectingStartMs = 0; FSM_DBG_PRINTLN("GLOBAL EXIT: Detecting - equalize timer cleared"); });
  // Done entry: reset subs back to idle and stop UVs
  fsmGlobal.setEntry(GlobalState::Done, [](){
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Done - resetting subs (local)");
    fsmSub1.handleEvent(Event::ResetPressed); fsmSub2.handleEvent(Event::ResetPressed);
    uvStop(0); uvStop(1); g_subDoneMask = 0;
    g_doneStartMs = millis();
  });

  // Ensure Done exit clears the done-start timestamp used for auto-reset
  fsmGlobal.setExit(GlobalState::Done, [](){ g_doneStartMs = 0; });
}

// FreeRTOS task that drives all FSMs
static void vStateMachineTask(void* /*pvParameters*/) {
  pinMode(START_PIN, INPUT_PULLUP); pinMode(RESET_PIN, INPUT_PULLUP);
  if (!g_fsmEventQ) g_fsmEventQ = xQueueCreate(FSM_QUEUE_LEN, sizeof(EventMsg));
  uvInit(); setupStateMachines(); FSM_DBG_PRINTLN("FSM Task started");

  auto forwardToSubs = [](Event e){ fsmSub1.handleEvent(e); fsmSub2.handleEvent(e); };

  while (true) {
    if (readStart()) {
      if (fsmGlobal.getState() == GlobalState::Running) fsmPostEvent(Event::SubStart, false);
      else fsmPostEvent(Event::StartPressed, false);
    }
    if (readReset()) fsmPostEvent(Event::ResetPressed, true);

    if (detectingStartMs != 0) {
      uint32_t now = millis();
      if ((uint32_t)(now - detectingStartMs) >= SENSOR_EQUALIZE_MS) {
        FSM_DBG_PRINTLN("GLOBAL: Detecting timeout -> SensorTimeout");
        fsmPostEvent(Event::SensorTimeout, false); detectingStartMs = 0;
      }
    }

    if (g_fsmEventQ) {
      EventMsg m;
      if (xQueueReceive(g_fsmEventQ, &m, 0) == pdTRUE) {
        FSM_DBG_PRINT("FSM: dequeued event -> "); FSM_DBG_PRINT_INT("", (int)m.ev);
        bool globalConsumed = fsmGlobal.handleEvent(m.ev);
        if (m.broadcastAll) forwardToSubs(m.ev);
        else if (!globalConsumed) {
          switch (m.ev) {
            case Event::Shoe0InitWet: case Event::Shoe0InitDry: fsmSub1.handleEvent(m.ev); break;
            case Event::Shoe1InitWet: case Event::Shoe1InitDry: fsmSub2.handleEvent(m.ev); break;
            case Event::SubStart:
              // Allow SubStart to advance subs regardless of UV timer state.
              fsmSub1.handleEvent(m.ev);
              fsmSub2.handleEvent(m.ev);
              break;
            case Event::UVTimer0:
              // UV timer expired: if sub1 is in DRY -> advance to DONE.
              // If sub1 is in COOLING -> mark that UV expired during cooling
              // and handle progression when COOLING finishes/exits.
              if (fsmSub1.getState() == SubState::S_DRY) {
                fsmSub1.handleEvent(Event::SubStart);
              } else if (fsmSub1.getState() == SubState::S_COOLING) {
                FSM_DBG_PRINTLN("SUB1: UV timer expired during COOLING -> mark expired (handle on cooling exit)");
                g_uvExpiredDuringCooling[0] = true;
              }
              // Mark UV complete for sub1 so we don't restart it on repeated loops
              g_uvComplete[0] = true;
              break;
            case Event::UVTimer1:
              if (fsmSub2.getState() == SubState::S_DRY) {
                fsmSub2.handleEvent(Event::SubStart);
              } else if (fsmSub2.getState() == SubState::S_COOLING) {
                FSM_DBG_PRINTLN("SUB2: UV timer expired during COOLING -> mark expired (handle on cooling exit)");
                g_uvExpiredDuringCooling[1] = true;
              }
              // Mark UV complete for sub2 so we don't restart it on repeated loops
              g_uvComplete[1] = true;

              break;
            case Event::SubFSMDone:
              if (fsmSub1.getState() == SubState::S_DONE && fsmSub2.getState() == SubState::S_DONE) fsmGlobal.handleEvent(m.ev);
              break;
            default: break;
          }
        }
      }
    }
    // Auto-reset: if we entered Done, after 60s reset to Idle
    if (g_doneStartMs != 0) {
      uint32_t now = millis();
      if ((uint32_t)(now - g_doneStartMs) >= DONE_TIMEOUT_MS) {
        FSM_DBG_PRINTLN("GLOBAL: Done timeout -> ResetPressed (auto)");
        fsmPostEvent(Event::ResetPressed, /*broadcastAll=*/true);
        g_doneStartMs = 0;
      }
    }

    fsmGlobal.run();
    vTaskDelay(pdMS_TO_TICKS(FSM_LOOP_DELAY_MS));
  }
}

void createStateMachineTask() {
  motorInit();
  xTaskCreatePinnedToCore(vStateMachineTask, "StateMachineTask", 4096, nullptr, 1, nullptr, 1);
}

GlobalState getGlobalState() { return fsmGlobal.getState(); }
SubState getSub1State()    { return fsmSub1.getState(); }
SubState getSub2State()    { return fsmSub2.getState(); }