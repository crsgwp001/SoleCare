#include "tskFSM.h"
#include <Arduino.h>
#include <StateMachine.h>
#include <Events.h>
#include "global.h"
#include "fsm_debug.h"
#include "tskUV.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Your global string status array (2 shoes, up to 4 chars + null)

// Button definitions
static constexpr int START_PIN = 35;
static constexpr int RESET_PIN = 34;

// Tidy: named timing/config constants
static constexpr uint32_t DEBOUNCE_MS = 300u;
static constexpr uint32_t FSM_LOOP_DELAY_MS = 50u;
static constexpr size_t   FSM_QUEUE_LEN = 8;

// Instantiate Global + two Sub-FSMs
static StateMachine<GlobalState, Event> fsmGlobal(GlobalState::Idle);
static StateMachine<SubState,   Event> fsmSub1  (SubState::S_IDLE);
static StateMachine<SubState,   Event> fsmSub2  (SubState::S_IDLE);

// Sensor equalize timeout (milliseconds)
static constexpr uint32_t SENSOR_EQUALIZE_MS = 6u * 1000u; // 6 seconds
// timestamp when we entered Detecting (0 = not active)
static uint32_t detectingStartMs = 0;
// Event queue for serializing FSM events
struct EventMsg { Event ev; bool broadcastAll; };
static QueueHandle_t g_fsmEventQ = nullptr;

// forward declaration
static bool fsmPostEvent(Event ev, bool broadcastAll);

// Track which subs have reached S_DONE. Bit0 = sub1, Bit1 = sub2.
static uint8_t g_subDoneMask = 0;

static void markSubDone(int idx) {
  if (idx == 0) g_subDoneMask |= 1u;
  else if (idx == 1) g_subDoneMask |= 2u;
  // If both bits set, queue a single SubFSMDone for Global
  if (g_subDoneMask == 0x03) {
    fsmPostEvent(Event::SubFSMDone, /*broadcastAll=*/false);
  }
}

// Helper to post an event to the FSM queue (safe from task context)
static bool fsmPostEvent(Event ev, bool broadcastAll = false) {
  if (!g_fsmEventQ) return false;
  // simple duplicate-guard: suppress identical quick repeats for Start/SubStart
  static Event lastEv = Event::None;
  static uint32_t lastEvMs = 0;
  if (ev == Event::StartPressed || ev == Event::SubStart) {
    uint32_t now = millis();
    if (lastEv == ev && (uint32_t)(now - lastEvMs) < DEBOUNCE_MS) {
      // drop duplicate
      return false;
    }
    lastEv = ev;
    lastEvMs = now;
  }
  EventMsg m{ev, broadcastAll};
  return xQueueSend(g_fsmEventQ, &m, 0) == pdTRUE;
}

  // Allow external tasks to post events to the FSM queue
  bool fsmExternalPost(Event ev) {
    return fsmPostEvent(ev, /*broadcastAll=*/false);
  }

//------------------------------------------------------------------------------
// 50 ms software debounce helpers
//------------------------------------------------------------------------------
static bool readStart() {
  static bool    last = HIGH;
  static uint32_t t0  = 0;
  bool raw = digitalRead(START_PIN);
  if (raw != last && (millis() - t0) > DEBOUNCE_MS) {
    t0   = millis();
    last = raw;
    if (raw == LOW) return true;
  }
  return false;
}

static bool readReset() {
  static bool    last = HIGH;
  static uint32_t t0  = 0;
  bool raw = digitalRead(RESET_PIN);
  if (raw != last && (millis() - t0) > DEBOUNCE_MS) {
    t0   = millis();
    last = raw;
    if (raw == LOW) return true;
  }
  return false;
}

//------------------------------------------------------------------------------
// Variadic broadcast: send the same event into N FSMs
//------------------------------------------------------------------------------
// C++11-friendly variadic broadcast (deliver synchronously)
template<typename StateT, typename EventT>
static void broadcast(EventT ev, StateMachine<StateT, EventT>& first) {
  first.handleEvent(ev);
}

template<typename StateT, typename EventT, typename... Fs>
static void broadcast(EventT ev, StateMachine<StateT, EventT>& first, Fs&... rest) {
  first.handleEvent(ev);
  broadcast<StateT, EventT>(ev, rest...);
}

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
    { SubState::S_IDLE,    Event::Shoe0InitWet, SubState::S_WET,     [](){ FSM_DBG_PRINTLN("SUB1: entered S_WET"); } },
    { SubState::S_IDLE,    Event::Shoe0InitDry, SubState::S_DRY,     [](){ FSM_DBG_PRINTLN("SUB1: entered S_DRY"); } },
  { SubState::S_WET,     Event::SubStart,     SubState::S_COOLING, [](){ FSM_DBG_PRINTLN("SUB1: Wet→Cooling (SubStart)"); } },
  { SubState::S_COOLING, Event::SubStart,     SubState::S_DRY,     [](){ FSM_DBG_PRINTLN("SUB1: Cooling→Dry (SubStart)"); } },
  { SubState::S_DRY,     Event::SubStart,     SubState::S_DONE,    [](){ FSM_DBG_PRINTLN("SUB1: Dry→Done (SubStart)"); markSubDone(0); } },
  };

  Transition<SubState, Event> sub2_trans[] = {
    { SubState::S_IDLE,    Event::Shoe1InitWet, SubState::S_WET,     [](){ FSM_DBG_PRINTLN("SUB2: entered S_WET"); } },
    { SubState::S_IDLE,    Event::Shoe1InitDry, SubState::S_DRY,     [](){ FSM_DBG_PRINTLN("SUB2: entered S_DRY"); } },
  { SubState::S_WET,     Event::SubStart,     SubState::S_COOLING, [](){ FSM_DBG_PRINTLN("SUB2: Wet→Cooling (SubStart)"); } },
  { SubState::S_COOLING, Event::SubStart,     SubState::S_DRY,     [](){ FSM_DBG_PRINTLN("SUB2: Cooling→Dry (SubStart)"); } },
  { SubState::S_DRY,     Event::SubStart,     SubState::S_DONE,    [](){ FSM_DBG_PRINTLN("SUB2: Dry→Done (SubStart)"); markSubDone(1); } },
    { SubState::S_DONE,    Event::SubStart,     SubState::S_DONE,    [](){ /* end */ } }
  };

  // Register transitions
  for (auto &t : global_trans) fsmGlobal.addTransition(t);
  for (auto &t : sub1_trans)  fsmSub1.addTransition(t);
  for (auto &t : sub2_trans)  fsmSub2.addTransition(t);

  // Make ResetPressed a catch-all: any Global state -> Idle
  for (uint8_t s = 0; s < static_cast<uint8_t>(GlobalState::Count); ++s) {
    GlobalState st = static_cast<GlobalState>(s);
  fsmGlobal.addTransition({ st, Event::ResetPressed, GlobalState::Idle, [](){ FSM_DBG_PRINTLN("GLOBAL: Reset (Idle)"); } });
  }

  // Make ResetPressed catch-all for subs -> S_IDLE
  for (int s = 0; s <= static_cast<int>(SubState::S_DONE); ++s) {
    SubState st = static_cast<SubState>(s);
  fsmSub1.addTransition({ st, Event::ResetPressed, SubState::S_IDLE, [](){ FSM_DBG_PRINTLN("SUB1: Reset (S_Idle)"); g_subDoneMask &= ~1u; } });
  fsmSub2.addTransition({ st, Event::ResetPressed, SubState::S_IDLE, [](){ FSM_DBG_PRINTLN("SUB2: Reset (S_Idle)"); g_subDoneMask &= ~2u; } });
  }

  // Run callback for Running: poll subs
  fsmGlobal.setRun(GlobalState::Running, [](){ fsmSub1.run(); fsmSub2.run(); });

  // Running entry: initialize subs and kick with internal SubStart
  fsmGlobal.setEntry(GlobalState::Running, [](){
    bool s1Wet = (strcmp(g_dhtStatus[0], "wet") == 0);
    bool s2Wet = (strcmp(g_dhtStatus[1], "wet") == 0);
  FSM_DBG_PRINTLN("GLOBAL ENTRY: Running - initializing subs");
    // Post init events into the FSM queue rather than calling directly so
    // that all transitions are processed by the FSM task and serialized.
    fsmPostEvent(s1Wet ? Event::Shoe0InitWet : Event::Shoe0InitDry, /*broadcast*/false);
    fsmPostEvent(s2Wet ? Event::Shoe1InitWet : Event::Shoe1InitDry, /*broadcast*/false);
    // Do NOT auto-kick subs with SubStart here — that would make a single
    // physical Start press cause both the ShoeInit + SubStart transitions
    // (two transitions) for the same sub. Instead require an explicit
    // subsequent Start press to advance subs via SubStart.
  });

  // Sub entries/exits
  // Register state entry/exit callbacks via compact tables
  struct StateCBG { GlobalState s; void(*entry)(); void(*exit)(); };
  struct StateCBS { SubState s; void(*entry)(); void(*exit)(); };

  static StateCBG global_cbs[] = {
    { GlobalState::Running, [](){ bool s1Wet = (strcmp(g_dhtStatus[0],"wet") == 0); bool s2Wet = (strcmp(g_dhtStatus[1],"wet") == 0); FSM_DBG_PRINTLN("GLOBAL ENTRY: Running - initializing subs"); fsmPostEvent(s1Wet ? Event::Shoe0InitWet : Event::Shoe0InitDry, false); fsmPostEvent(s2Wet ? Event::Shoe1InitWet : Event::Shoe1InitDry, false); }, nullptr },
    { GlobalState::Detecting, [](){ detectingStartMs = millis(); FSM_DBG_PRINTLN("GLOBAL ENTRY: Detecting - equalize timer started"); }, [](){ detectingStartMs = 0; FSM_DBG_PRINTLN("GLOBAL EXIT: Detecting - equalize timer cleared"); } },
    { GlobalState::Done, [](){ FSM_DBG_PRINTLN("GLOBAL ENTRY: Done - resetting subs (local)"); fsmSub1.handleEvent(Event::ResetPressed); fsmSub2.handleEvent(Event::ResetPressed); uvStop(0); uvStop(1); g_subDoneMask = 0; }, nullptr }
  };

  static StateCBS sub1_cbs[] = {
    { SubState::S_WET, [](){ FSM_DBG_PRINTLN("SUB1 ENTRY: WET"); }, [](){ FSM_DBG_PRINTLN("SUB1 EXIT: leaving WET"); } },
    { SubState::S_COOLING, [](){ FSM_DBG_PRINTLN("SUB1 ENTRY: COOLING"); uvStart(0, 0); }, nullptr },
    { SubState::S_DRY, [](){ FSM_DBG_PRINTLN("SUB1 ENTRY: DRY"); if (!uvIsStarted(0)) uvStart(0, 0); }, nullptr }
  };

  static StateCBS sub2_cbs[] = {
    { SubState::S_WET, [](){ FSM_DBG_PRINTLN("SUB2 ENTRY: WET"); }, nullptr },
    { SubState::S_COOLING, [](){ FSM_DBG_PRINTLN("SUB2 ENTRY: COOLING"); uvStart(1, 0); }, nullptr },
    { SubState::S_DRY, [](){ FSM_DBG_PRINTLN("SUB2 ENTRY: DRY"); if (!uvIsStarted(1)) uvStart(1, 0); }, nullptr }
  };

  for (auto &c : global_cbs) {
    if (c.entry) fsmGlobal.setEntry(c.s, c.entry);
    if (c.exit)  fsmGlobal.setExit(c.s, c.exit);
  }
  for (auto &c : sub1_cbs) {
    if (c.entry) fsmSub1.setEntry(c.s, c.entry);
    if (c.exit)  fsmSub1.setExit(c.s, c.exit);
  }
  for (auto &c : sub2_cbs) {
    if (c.entry) fsmSub2.setEntry(c.s, c.entry);
    if (c.exit)  fsmSub2.setExit(c.s, c.exit);
  }

  // Detecting entry/exit
  fsmGlobal.setEntry(GlobalState::Detecting, [](){ detectingStartMs = millis(); FSM_DBG_PRINTLN("GLOBAL ENTRY: Detecting - equalize timer started"); });
  fsmGlobal.setExit(GlobalState::Detecting,  [](){ detectingStartMs = 0; FSM_DBG_PRINTLN("GLOBAL EXIT: Detecting - equalize timer cleared"); });
  // Done entry: reset subs back to idle
  fsmGlobal.setEntry(GlobalState::Done, [](){
  FSM_DBG_PRINTLN("GLOBAL ENTRY: Done - resetting subs (local)");
    // Use local sub resets instead of the global catch-all ResetPressed
    // to avoid affecting Global's state machine.
    fsmSub1.handleEvent(Event::ResetPressed);
    fsmSub2.handleEvent(Event::ResetPressed);
    // clear completion mask
    // ensure UVs are stopped
    uvStop(0);
    uvStop(1);
    g_subDoneMask = 0;
  });
}

//------------------------------------------------------------------------------
// FreeRTOS task that drives all FSMs
//------------------------------------------------------------------------------
static void vStateMachineTask(void* pvParameters) {
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  // Create the FSM event queue
  if (!g_fsmEventQ) {
    g_fsmEventQ = xQueueCreate(FSM_QUEUE_LEN, sizeof(EventMsg));
  }
  // Initialize UV task/queue
  uvInit();
  setupStateMachines();
  FSM_DBG_PRINTLN("FSM Task started");

  // forward an event to both subs
  auto forwardToSubs = [](Event e){ fsmSub1.handleEvent(e); fsmSub2.handleEvent(e); };

  while (true) {
    if (readStart()) {
      // If Global already Running, the Start button should advance subs
      // via the internal SubStart event. Otherwise, post StartPressed to
      // kick Global through Detecting->Checking->Running.
      if (fsmGlobal.getState() == GlobalState::Running) {
        fsmPostEvent(Event::SubStart, /*broadcastAll=*/false);
      } else {
        fsmPostEvent(Event::StartPressed, /*broadcastAll=*/false);
      }
    }

    if (readReset()) {
      // Reset Global + both subs back to Idle — broadcast to all FSMs
      fsmPostEvent(Event::ResetPressed, /*broadcastAll=*/true);
    }

    // If we're in Detecting and the equalize timer expired, enqueue SensorTimeout
    if (detectingStartMs != 0) {
      uint32_t now = millis();
      // handle wrap-around safely
      if ((uint32_t)(now - detectingStartMs) >= SENSOR_EQUALIZE_MS) {
  FSM_DBG_PRINTLN("GLOBAL: Detecting timeout -> SensorTimeout");
        fsmPostEvent(Event::SensorTimeout, /*broadcastAll=*/false);
        // clear to avoid repeats
        detectingStartMs = 0;
      }
    }

    // 1) Drain FSM event queue and route events. Deliver to Global first.
    if (g_fsmEventQ) {
        EventMsg m;
        // don't block; process at most one queued event per loop iteration
        if (xQueueReceive(g_fsmEventQ, &m, 0) == pdTRUE) {
  // Deliver to Global first
  FSM_DBG_PRINT("FSM: dequeued event -> ");
  FSM_DBG_PRINT_INT("", (int)m.ev);
        bool globalConsumed = fsmGlobal.handleEvent(m.ev);

        // Always forward broadcastAll events to subs (Reset should reach subs)
        if (m.broadcastAll) {
          forwardToSubs(m.ev);
        } else {
          // If Global didn't handle the event, route specific events to the
          // appropriate sub-FSM(s).
          if (!globalConsumed) {
            switch (m.ev) {
              case Event::Shoe0InitWet:
              case Event::Shoe0InitDry:
                fsmSub1.handleEvent(m.ev);
                break;
              case Event::Shoe1InitWet:
              case Event::Shoe1InitDry:
                fsmSub2.handleEvent(m.ev);
                break;
              case Event::SubStart:
                // advance both subs, but block advancement from S_DRY if UV timer still running
                if (!(fsmSub1.getState() == SubState::S_DRY && !uvTimerFinished(0))) {
                  fsmSub1.handleEvent(m.ev);
                } else {
                  FSM_DBG_PRINTLN("SUB1: SubStart ignored - UV timer running");
                }
                if (!(fsmSub2.getState() == SubState::S_DRY && !uvTimerFinished(1))) {
                  fsmSub2.handleEvent(m.ev);
                } else {
                  FSM_DBG_PRINTLN("SUB2: SubStart ignored - UV timer running");
                }
                break;
                case Event::UVTimer0:
                  // When UV 0 completes, only advance if sub1 is in DRY (not while in COOLING)
                  if (fsmSub1.getState() == SubState::S_DRY) {
                    fsmSub1.handleEvent(Event::SubStart);
                  }
                  break;
                case Event::UVTimer1:
                  if (fsmSub2.getState() == SubState::S_DRY) {
                    fsmSub2.handleEvent(Event::SubStart);
                  }
                  break;
              case Event::SubFSMDone:
                // Only forward completion to Global when both subs reached S_DONE
                if (fsmSub1.getState() == SubState::S_DONE && fsmSub2.getState() == SubState::S_DONE) {
                  fsmGlobal.handleEvent(m.ev);
                } else {
                  // drop; wait until both subs signal done
                }
                break;
              default:
                break;
            }
          }
        }
      }
    }

    // Continuously service the Global FSM (which polls subs)
    fsmGlobal.run();

  // Yield for the main loop delay
  vTaskDelay(pdMS_TO_TICKS(FSM_LOOP_DELAY_MS));
  }
}

void createStateMachineTask() {
  xTaskCreatePinnedToCore(
    vStateMachineTask,
    "StateMachineTask",
    4096,
    nullptr,
    1,
    nullptr,
    1
  );
}