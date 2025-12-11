#include "tskFSM.h"
#include "global.h"
#include "fsm_debug.h"
#include "tskMotor.h"
#include "tskUV.h"
#include "PIDcontrol.h"
#include <Arduino.h>
#include <events.h>
#include <StateMachine.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// External PID control globals (defined in tskMotor.cpp)
extern PIDcontrol g_motorPID[2];
extern bool g_pidInitialized[2];

// Button definitions (configurable in include/config.h)
static constexpr int START_PIN = HW_START_PIN;
static constexpr int RESET_PIN = HW_RESET_PIN;

// Timing/config
static constexpr uint32_t DEBOUNCE_MS = 300u;
static constexpr uint32_t FSM_LOOP_DELAY_MS = 50u;
static constexpr size_t FSM_QUEUE_LEN = 8;

// Instantiate Global + two Sub-FSMs
static StateMachine<GlobalState, Event> fsmGlobal(GlobalState::Idle);
static StateMachine<SubState, Event> fsmSub1(SubState::S_IDLE);
static StateMachine<SubState, Event> fsmSub2(SubState::S_IDLE);

// Sensor equalize timeout (milliseconds)
static constexpr uint32_t SENSOR_EQUALIZE_MS = 6u * 1000u; // 6s
// timestamp when we entered Detecting (0 = not active)
static uint32_t detectingStartMs = 0;
// Event queue for serializing FSM events
struct EventMsg {
  Event ev;
  bool broadcastAll;
};
static QueueHandle_t g_fsmEventQ = nullptr;

// forward declaration
static bool fsmPostEvent(Event ev, bool broadcastAll);

// Track which subs have reached S_DONE. Bit0 = sub1, Bit1 = sub2.
static uint8_t g_subDoneMask = 0;
// timestamp when we entered Done (0 = not active). After 60s -> auto-reset
static uint32_t g_doneStartMs = 0;
// per-sub timestamps for managing time-based transitions
static uint32_t g_subWetStartMs[2] = {0, 0};
// g_subDryerStartMs removed; use g_subWetStartMs for wet runtime and g_subCoolingStartMs for
// cooling
static uint32_t g_subCoolingStartMs[2] = {0, 0};
static uint32_t g_subCoolingStabilizeStartMs[2] = {0, 0};
static bool g_coolingLocked[2] = {false, false};

// Track AH rate-of-change during WET to detect peak evaporation
// Used to exit WET state when drying rate starts declining
static float g_prevAHRate[2] = {0.0f, 0.0f};  // Previous AH rate-of-change sample
static uint32_t g_lastAHRateSampleMs[2] = {0, 0};  // Timestamp of last sample
static int g_ahRateSampleCount[2] = {0, 0};  // Number of samples collected
constexpr int MIN_AH_RATE_SAMPLES = 3;  // Need at least this many samples before checking decline
constexpr float AH_RATE_DECLINE_THRESHOLD = -0.01f;  // Rate-of-change decline to trigger exit (g/m³/min²)
// Track whether the UV timer expired while the sub was in the COOLING phase.
static bool g_uvExpiredDuringCooling[2] = {false, false};
// UV complete flag: set when a UV timer has finished for a sub. Prevents
// restarting the UV timer on subsequent loops between cooling->wet.
static bool g_uvComplete[2] = {false, false};
// Protect g_uvComplete access since it's read from UI (core 0) and written by FSM (core 1)
// (g_uvComplete is internal to FSM)

// Track whether we have started the motor for a sub during the current wet->..->dry cycle.
static bool g_motorStarted[2] = {false, false};

// Sequential WET lock: only one shoe can be in S_WET at a time
// -1 = free, 0 = shoe 0 owns the lock, 1 = shoe 1 owns the lock
static int g_wetLockOwner = -1;

// LED status tracking
static uint32_t g_ledBlinkMs = 0;  // Timestamp for LED blinking
static bool g_ledBlinkState = false;  // Current blink state

// Battery check timestamp for LowBattery state
static uint32_t g_lastBatteryCheckMs = 0;

static void markSubDone(int idx) {
  if (idx == 0)
    g_subDoneMask |= 1u;
  else if (idx == 1)
    g_subDoneMask |= 2u;
  if (g_subDoneMask == 0x03)
    fsmPostEvent(Event::SubFSMDone, false);
}

// Helper to post an event to the FSM queue (safe from task context)
static bool fsmPostEvent(Event ev, bool broadcastAll = false) {
  if (!g_fsmEventQ)
    return false;
  // duplicate-guard for Start/SubStart
  static Event lastEv = Event::None;
  static uint32_t lastEvMs = 0;
  if (ev == Event::StartPressed || ev == Event::SubStart) {
    uint32_t now = millis();
    if (lastEv == ev && (uint32_t)(now - lastEvMs) < DEBOUNCE_MS)
      return false;
    lastEv = ev;
    lastEvMs = now;
  }
  EventMsg m{ev, broadcastAll};
  return xQueueSend(g_fsmEventQ, &m, 0) == pdTRUE;
}

// Allow external tasks to post events to the FSM queue
bool fsmExternalPost(Event ev) {
  return fsmPostEvent(ev, false);
}

static bool readStart() {
  static bool last = HIGH;
  static uint32_t t0 = 0;
  bool raw = digitalRead(START_PIN);
  if (raw != last && (millis() - t0) > DEBOUNCE_MS) {
    t0 = millis();
    last = raw;
    return raw == LOW;
  }
  return false;
}

static bool readReset() {
  static bool last = HIGH;
  static uint32_t t0 = 0;
  bool raw = digitalRead(RESET_PIN);
  if (raw != last && (millis() - t0) > DEBOUNCE_MS) {
    t0 = millis();
    last = raw;
    return raw == LOW;
  }
  return false;
}

// Variadic broadcast helper
template <typename StateT, typename EventT>
static void broadcast(EventT ev, StateMachine<StateT, EventT> &first) {
  first.handleEvent(ev);
}
template <typename StateT, typename EventT, typename... Fs>
static void broadcast(EventT ev, StateMachine<StateT, EventT> &first, Fs &...rest) {
  first.handleEvent(ev);
  broadcast<StateT, EventT>(ev, rest...);
}

//------------------------------------------------------------------------------
// Configure all transitions and run-callbacks
//------------------------------------------------------------------------------
static void setupStateMachines() {
  // Transition tables
  Transition<GlobalState, Event> global_trans[] = {
      {GlobalState::Idle, Event::StartPressed, GlobalState::Detecting,
       []() { FSM_DBG_PRINTLN("GLOBAL: Detecting start"); }},
      {GlobalState::Detecting, Event::SensorTimeout, GlobalState::Checking,
       []() { FSM_DBG_PRINTLN("GLOBAL: Sensor timeout -> Checking"); }},
      {GlobalState::Checking, Event::StartPressed, GlobalState::Running,
       []() { FSM_DBG_PRINTLN("GLOBAL: Checking -> Running"); }},
      {GlobalState::Checking, Event::BatteryLow, GlobalState::LowBattery,
       []() { FSM_DBG_PRINTLN("GLOBAL: Battery low -> LowBattery"); }},
      {GlobalState::LowBattery, Event::BatteryRecovered, GlobalState::Idle,
       []() { FSM_DBG_PRINTLN("GLOBAL: Battery recovered -> Idle"); }},
      {GlobalState::Running, Event::SubFSMDone, GlobalState::Done,
       []() { FSM_DBG_PRINTLN("GLOBAL: All subs done -> Done"); }},
      {GlobalState::Idle, Event::Error, GlobalState::Error,
       []() { FSM_DBG_PRINTLN("GLOBAL: Error"); }},
  };

  Transition<SubState, Event> sub1_trans[] = {
      {SubState::S_IDLE, Event::Shoe0InitWet, SubState::S_WAITING,
       []() { FSM_DBG_PRINTLN("SUB1: S_IDLE -> S_WAITING (wet shoe)"); }},
      {SubState::S_IDLE, Event::Shoe0InitDry, SubState::S_DRY,
       []() { FSM_DBG_PRINTLN("SUB1: S_IDLE -> S_DRY (dry shoe)"); }},
      // S_WAITING -> S_WET (when lock acquired)
      {SubState::S_WAITING, Event::SubStart, SubState::S_WET,
       []() { FSM_DBG_PRINTLN("SUB1: S_WAITING -> S_WET (lock acquired)"); }},
      // New flow: S_WET -> S_COOLING (heater off, motor continues) -> S_DRY
      {SubState::S_WET, Event::SubStart, SubState::S_COOLING,
       []() { FSM_DBG_PRINTLN("SUB1: Wet→Cooling (SubStart)"); }},
      {SubState::S_COOLING, Event::DryCheckFailed, SubState::S_WAITING,
       []() { FSM_DBG_PRINTLN("SUB1: Cooling->Waiting (DryCheckFailed - return to queue)"); }},
      {SubState::S_COOLING, Event::SubStart, SubState::S_DRY,
       []() { FSM_DBG_PRINTLN("SUB1: Cooling→Dry (SubStart)"); }},
      {SubState::S_DRY, Event::SubStart, SubState::S_DONE,
       []() {
         FSM_DBG_PRINTLN("SUB1: Dry→Done (SubStart)");
         markSubDone(0);
       }},
      // If dry-check fails, return to waiting queue
      {SubState::S_DRY, Event::DryCheckFailed, SubState::S_WAITING,
       []() { FSM_DBG_PRINTLN("SUB1: Dry->Waiting (DryCheckFailed - return to queue)"); }},
  };

  Transition<SubState, Event> sub2_trans[] = {
      {SubState::S_IDLE, Event::Shoe1InitWet, SubState::S_WAITING,
       []() { FSM_DBG_PRINTLN("SUB2: S_IDLE -> S_WAITING (wet shoe)"); }},
      {SubState::S_IDLE, Event::Shoe1InitDry, SubState::S_DRY,
       []() { FSM_DBG_PRINTLN("SUB2: S_IDLE -> S_DRY (dry shoe)"); }},
      // S_WAITING -> S_WET (when lock acquired)
      {SubState::S_WAITING, Event::SubStart, SubState::S_WET,
       []() { FSM_DBG_PRINTLN("SUB2: S_WAITING -> S_WET (lock acquired)"); }},
      {SubState::S_WET, Event::SubStart, SubState::S_COOLING,
       []() { FSM_DBG_PRINTLN("SUB2: Wet→Cooling (SubStart)"); }},
      {SubState::S_COOLING, Event::DryCheckFailed, SubState::S_WAITING,
       []() { FSM_DBG_PRINTLN("SUB2: Cooling->Waiting (DryCheckFailed - return to queue)"); }},
      {SubState::S_COOLING, Event::SubStart, SubState::S_DRY,
       []() { FSM_DBG_PRINTLN("SUB2: Cooling→Dry (SubStart)"); }},
      {SubState::S_DRY, Event::SubStart, SubState::S_DONE,
       []() {
         FSM_DBG_PRINTLN("SUB2: Dry→Done (SubStart)");
         markSubDone(1);
       }},
      {SubState::S_DRY, Event::DryCheckFailed, SubState::S_WAITING,
       []() { FSM_DBG_PRINTLN("SUB2: Dry->Waiting (DryCheckFailed - return to queue)"); }},
      {SubState::S_DONE, Event::SubStart, SubState::S_DONE, []() { /* end */ }}};

  // Register transitions
  for (auto &t : global_trans)
    fsmGlobal.addTransition(t);
  for (auto &t : sub1_trans)
    fsmSub1.addTransition(t);
  for (auto &t : sub2_trans)
    fsmSub2.addTransition(t);

  // Make ResetPressed a catch-all: Global -> Idle
  for (uint8_t s = 0; s < static_cast<uint8_t>(GlobalState::Count); ++s) {
    fsmGlobal.addTransition({static_cast<GlobalState>(s), Event::ResetPressed, GlobalState::Idle,
                             []() { FSM_DBG_PRINTLN("GLOBAL: Reset (Idle)"); }});
  }
  // Reset for subs -> S_IDLE
  for (int s = 0; s <= static_cast<int>(SubState::S_DONE); ++s) {
    fsmSub1.addTransition({static_cast<SubState>(s), Event::ResetPressed, SubState::S_IDLE, []() {
                             FSM_DBG_PRINTLN("SUB1: Reset (S_Idle)");
                             g_subDoneMask &= ~1u;
                           }});
    fsmSub2.addTransition({static_cast<SubState>(s), Event::ResetPressed, SubState::S_IDLE, []() {
                             FSM_DBG_PRINTLN("SUB2: Reset (S_Idle)");
                             g_subDoneMask &= ~2u;
                           }});
  }

  // Run callback for Running: poll subs
  fsmGlobal.setRun(GlobalState::Running, []() {
    fsmSub1.run();
    fsmSub2.run();
  });

  // Register entry/exit callbacks
  struct StateCBG {
    GlobalState s;
    void (*entry)();
    void (*exit)();
  };
  struct StateCBS {
    SubState s;
    void (*entry)();
    void (*exit)();
  };

  static StateCBG global_cbs[] = {
       {GlobalState::Detecting,
       []() {
         detectingStartMs = millis();
         FSM_DBG_PRINTLN("GLOBAL ENTRY: Detecting - equalize timer started");
       },
       []() {
         detectingStartMs = 0;
         FSM_DBG_PRINTLN("GLOBAL EXIT: Detecting - equalize timer cleared");
       }},
      {GlobalState::Done,
       []() {
         FSM_DBG_PRINTLN("GLOBAL ENTRY: Done - resetting subs (local)");
         fsmSub1.handleEvent(Event::ResetPressed);
         fsmSub2.handleEvent(Event::ResetPressed);
         uvStop(0);
         uvStop(1);
         g_subDoneMask = 0;
         g_uvComplete[0] = g_uvComplete[1] = false;
         g_doneStartMs = millis();
         // Blink status LED in Done state, ensure error LED is off
         digitalWrite(HW_ERROR_LED_PIN, LOW);
         digitalWrite(HW_STATUS_LED_PIN, LOW);
         g_ledBlinkMs = millis();
         g_ledBlinkState = false;
       },
       []() { g_doneStartMs = 0; }}};

  static StateCBS sub1_cbs[] = {
      // S_WAITING: waiting for WET lock to become available
      {SubState::S_WAITING,
       []() {
         FSM_DBG_PRINTLN("SUB1 ENTRY: WAITING for WET lock");
       },
       []() {
         FSM_DBG_PRINTLN("SUB1 EXIT: leaving WAITING");
       }},
      // S_WET: start motor + heater and run for WET_TIMEOUT_MS then transition to COOLING
      {SubState::S_WET,
       []() {
         FSM_DBG_PRINTLN("SUB1 ENTRY: WET");
         // Heater on immediately; motor will start after warmup delay.
         motorStop(0);
         g_motorStarted[0] = false;
         motorSetDutyPercent(0, 0);
         heaterRun(0, true);
         g_subWetStartMs[0] = millis();
       },
       []() {
         FSM_DBG_PRINTLN("SUB1 EXIT: WET -> resetting PID and releasing lock");
         g_pidInitialized[0] = false;
         g_motorPID[0].reset();
         // Release WET lock since COOLING doesn't need it
         if (g_wetLockOwner == 0) {
           g_wetLockOwner = -1;
           FSM_DBG_PRINTLN("SUB1: Released WET lock on exit to COOLING");
         }
       }},
      // S_COOLING: heater off, motor continues; start cooling timer
      {SubState::S_COOLING,
       []() {
         FSM_DBG_PRINTLN("SUB1 ENTRY: COOLING"); /* turn heater off, motor kept running */
         heaterRun(0, false);
         motorSetDutyPercent(0, 80); // hold motor at 80% during COOLING
         g_subCoolingStartMs[0] = millis();
         g_coolingLocked[0] = true;
       },
       []() {
         FSM_DBG_PRINTLN("SUB1 EXIT: leaving COOLING"); /* leaving cooling */
         g_coolingLocked[0] = false;
       }},
      // S_DRY: on entry stop motor; start single UV (GPIO14) when BOTH shoes reach DRY
      {SubState::S_DRY,
       []() {
         FSM_DBG_PRINTLN("SUB1 ENTRY: DRY");
         motorStop(0);
         g_motorStarted[0] = false;
         if (fsmSub2.getState() == SubState::S_DRY && !g_uvComplete[0] && !uvIsStarted(0)) {
           FSM_DBG_PRINTLN("SUB1 ENTRY: DRY -> Both shoes dry, starting single UV on GPIO14");
           uvStart(0, 0);
         } else {
           FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> Waiting for other shoe or UV to complete");
         }
       },
       nullptr},
      // S_DONE: release WET lock when done
      {SubState::S_DONE,
       []() {
         FSM_DBG_PRINTLN("SUB1 ENTRY: DONE - releasing WET lock");
         if (g_wetLockOwner == 0) {
           g_wetLockOwner = -1;
           FSM_DBG_PRINTLN("SUB1: Released WET lock on DONE");
         }
       },
       nullptr}
  };

  static StateCBS sub2_cbs[] = {
      // S_WAITING: waiting for WET lock to become available
      {SubState::S_WAITING,
       []() {
         FSM_DBG_PRINTLN("SUB2 ENTRY: WAITING for WET lock");
       },
       []() {
         FSM_DBG_PRINTLN("SUB2 EXIT: leaving WAITING");
       }},
      {SubState::S_WET,
       []() {
         FSM_DBG_PRINTLN("SUB2 ENTRY: WET");
         motorStop(1);
         g_motorStarted[1] = false;
         motorSetDutyPercent(1, 0);
         heaterRun(1, true);
         g_subWetStartMs[1] = millis();
       },
       []() {
         FSM_DBG_PRINTLN("SUB2 EXIT: WET -> resetting PID and releasing lock");
         g_pidInitialized[1] = false;
         g_motorPID[1].reset();
         // Release WET lock since COOLING doesn't need it
         if (g_wetLockOwner == 1) {
           g_wetLockOwner = -1;
           FSM_DBG_PRINTLN("SUB2: Released WET lock on exit to COOLING");
         }
       }},
      {SubState::S_COOLING,
       []() {
         FSM_DBG_PRINTLN("SUB2 ENTRY: COOLING");
         heaterRun(1, false);
         motorSetDutyPercent(1, 80); // hold motor at 80% during COOLING
         g_subCoolingStartMs[1] = millis();
         g_coolingLocked[1] = true;
       },
       []() { FSM_DBG_PRINTLN("SUB2 EXIT: leaving COOLING"); }},
      {SubState::S_DRY,
       []() {
         FSM_DBG_PRINTLN("SUB2 ENTRY: DRY");
         motorStop(1);
         g_motorStarted[1] = false;
         if (fsmSub1.getState() == SubState::S_DRY && !g_uvComplete[0] && !uvIsStarted(0)) {
           FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> Both shoes dry, starting single UV on GPIO14");
           uvStart(0, 0);
         } else {
           FSM_DBG_PRINTLN("SUB2 ENTRY: DRY -> Waiting for other shoe or UV to complete");
         }
       },
       nullptr},
      // S_DONE: release WET lock when done
      {SubState::S_DONE,
       []() {
         FSM_DBG_PRINTLN("SUB2 ENTRY: DONE - releasing WET lock");
         if (g_wetLockOwner == 1) {
           g_wetLockOwner = -1;
           FSM_DBG_PRINTLN("SUB2: Released WET lock on DONE");
         }
       },
       nullptr}};

  for (auto &c : global_cbs) {
    if (c.entry)
      fsmGlobal.setEntry(c.s, c.entry);
    if (c.exit)
      fsmGlobal.setExit(c.s, c.exit);
  }
  for (auto &c : sub1_cbs) {
    if (c.entry)
      fsmSub1.setEntry(c.s, c.entry);
    if (c.exit)
      fsmSub1.setExit(c.s, c.exit);
  }
  for (auto &c : sub2_cbs) {
    if (c.entry)
      fsmSub2.setEntry(c.s, c.entry);
    if (c.exit)
      fsmSub2.setExit(c.s, c.exit);
  }

  // Per-state run callbacks to handle time-based progression and checks
  // S_WAITING run: priority-based WET lock acquisition
  // If WET lock is free, acquire it and transition to WET
  // Priority: wetter shoe (higher AH diff) gets lock first
  fsmSub1.setRun(SubState::S_WAITING, []() {
    if (g_wetLockOwner == -1) {
      // Lock is free, check priority vs sub2
      if (fsmSub2.getState() == SubState::S_WAITING) {
        // Both waiting, give priority to wetter shoe
        float diff0 = g_dhtAHDiff[0];
        float diff1 = g_dhtAHDiff[1];
        if (diff0 >= diff1) {
          // SUB1 is wetter or equal, acquire lock
          g_wetLockOwner = 0;
          FSM_DBG_PRINTLN("SUB1: Acquired WET lock (priority)");
          fsmSub1.handleEvent(Event::SubStart);
        }
        // else: wait for SUB2 to acquire lock
      } else {
        // SUB2 not waiting, acquire lock
        g_wetLockOwner = 0;
        FSM_DBG_PRINTLN("SUB1: Acquired WET lock");
        fsmSub1.handleEvent(Event::SubStart);
      }
    }
  });

  fsmSub2.setRun(SubState::S_WAITING, []() {
    if (g_wetLockOwner == -1) {
      // Lock is free, check priority vs sub1
      if (fsmSub1.getState() == SubState::S_WAITING) {
        // Both waiting, give priority to wetter shoe
        float diff0 = g_dhtAHDiff[0];
        float diff1 = g_dhtAHDiff[1];
        if (diff1 > diff0) {
          // SUB2 is wetter, acquire lock
          g_wetLockOwner = 1;
          FSM_DBG_PRINTLN("SUB2: Acquired WET lock (priority)");
          fsmSub2.handleEvent(Event::SubStart);
        }
        // else: wait for SUB1 to acquire lock
      } else {
        // SUB1 not waiting, acquire lock
        g_wetLockOwner = 1;
        FSM_DBG_PRINTLN("SUB2: Acquired WET lock");
        fsmSub2.handleEvent(Event::SubStart);
      }
    }
  });

  // S_WET run: start motor after heater warmup; monitor AH rate-of-change to detect peak evaporation
  fsmSub1.setRun(SubState::S_WET, []() {
    if (!g_motorStarted[0] && g_subWetStartMs[0] != 0) {
      uint32_t elapsed = (uint32_t)(millis() - g_subWetStartMs[0]);
      if (elapsed >= HEATER_WARMUP_MS) {
        FSM_DBG_PRINTLN("SUB1: WET warmup done -> start motor");
        motorStart(0);
        motorSetDutyPercent(0, 100);
        g_motorStarted[0] = true;
        // Reset AH rate tracking when motor starts
        g_ahRateSampleCount[0] = 0;
        g_lastAHRateSampleMs[0] = millis();
        g_prevAHRate[0] = 0.0f;
      }
    }
    // Sample AH rate every 2 seconds and check for peak (declining rate)
    // But wait AH_ACCEL_WARMUP_MS (30s) after WET start before checking for decline
    if (g_motorStarted[0] && g_subWetStartMs[0] != 0) {
      uint32_t now = millis();
      uint32_t wetElapsed = (uint32_t)(now - g_subWetStartMs[0]);
      
      if ((uint32_t)(now - g_lastAHRateSampleMs[0]) >= 2000) {
        g_lastAHRateSampleMs[0] = now;
        float currentRate = g_dhtAHDiff[0];  // Current AH diff (proxy for evaporation rate)
        
        if (g_ahRateSampleCount[0] > 0 && wetElapsed >= AH_ACCEL_WARMUP_MS) {
          // Calculate rate-of-change (second derivative)
          float rateChange = currentRate - g_prevAHRate[0];
          FSM_DBG_PRINT("SUB1: WET AH diff=");
          FSM_DBG_PRINT(currentRate);
          FSM_DBG_PRINT(" prev=");
          FSM_DBG_PRINT(g_prevAHRate[0]);
          FSM_DBG_PRINT(" change=");
          FSM_DBG_PRINTLN(rateChange);
          
          // Check if rate is declining after collecting enough samples
          if (g_ahRateSampleCount[0] >= MIN_AH_RATE_SAMPLES && rateChange < AH_RATE_DECLINE_THRESHOLD) {
            FSM_DBG_PRINTLN("SUB1: WET peak evaporation detected (declining rate) -> transition to COOLING");
            fsmSub1.handleEvent(Event::SubStart);
            g_ahRateSampleCount[0] = 0;  // Reset counter
          }
        }
        g_prevAHRate[0] = currentRate;
        g_ahRateSampleCount[0]++;
      }
    }
  });
  fsmSub2.setRun(SubState::S_WET, []() {
    if (!g_motorStarted[1] && g_subWetStartMs[1] != 0) {
      uint32_t elapsed = (uint32_t)(millis() - g_subWetStartMs[1]);
      if (elapsed >= HEATER_WARMUP_MS) {
        FSM_DBG_PRINTLN("SUB2: WET warmup done -> start motor");
        motorStart(1);
        motorSetDutyPercent(1, 100);
        g_motorStarted[1] = true;
        // Reset AH rate tracking when motor starts
        g_ahRateSampleCount[1] = 0;
        g_lastAHRateSampleMs[1] = millis();
        g_prevAHRate[1] = 0.0f;
      }
    }
    // Sample AH rate every 2 seconds and check for peak (declining rate)
    // But wait AH_ACCEL_WARMUP_MS (30s) after WET start before checking for decline
    if (g_motorStarted[1] && g_subWetStartMs[1] != 0) {
      uint32_t now = millis();
      uint32_t wetElapsed = (uint32_t)(now - g_subWetStartMs[1]);
      
      if ((uint32_t)(now - g_lastAHRateSampleMs[1]) >= 2000) {
        g_lastAHRateSampleMs[1] = now;
        float currentRate = g_dhtAHDiff[1];  // Current AH diff (proxy for evaporation rate)
        
        if (g_ahRateSampleCount[1] > 0 && wetElapsed >= AH_ACCEL_WARMUP_MS) {
          // Calculate rate-of-change (second derivative)
          float rateChange = currentRate - g_prevAHRate[1];
          FSM_DBG_PRINT("SUB2: WET AH diff=");
          FSM_DBG_PRINT(currentRate);
          FSM_DBG_PRINT(" prev=");
          FSM_DBG_PRINT(g_prevAHRate[1]);
          FSM_DBG_PRINT(" change=");
          FSM_DBG_PRINTLN(rateChange);
          
          // Check if rate is declining after collecting enough samples
          if (g_ahRateSampleCount[1] >= MIN_AH_RATE_SAMPLES && rateChange < AH_RATE_DECLINE_THRESHOLD) {
            FSM_DBG_PRINTLN("SUB2: WET peak evaporation detected (declining rate) -> transition to COOLING");
            fsmSub2.handleEvent(Event::SubStart);
            g_ahRateSampleCount[1] = 0;  // Reset counter
          }
        }
        g_prevAHRate[1] = currentRate;
        g_ahRateSampleCount[1]++;
      }
    }
  });

  // S_COOLING run: two-phase logic
  // Phase 1 (0-5s): motor at 80%, heater off
  // Phase 2 (5-10s): motor off, stabilization period
  // After 10s: perform dry-check and advance
  fsmSub1.setRun(SubState::S_COOLING, []() {
    if (g_subCoolingStartMs[0] == 0)
      return;
    uint32_t motorElapsed = (uint32_t)(millis() - g_subCoolingStartMs[0]);
    
    // Phase 1: motor running (0-5s)
    if (motorElapsed < DRY_COOL_MS) {
      return; // Still in motor-run phase
    }
    
    // Transition from motor phase to stabilization phase
    if (g_subCoolingStabilizeStartMs[0] == 0) {
      FSM_DBG_PRINTLN("SUB1: COOLING -> motor phase done, starting stabilization");
      motorStop(0);
      g_subCoolingStabilizeStartMs[0] = millis();
      return;
    }
    
    // Phase 2: stabilization (check if stabilization period elapsed)
    uint32_t stabilizeElapsed = (uint32_t)(millis() - g_subCoolingStabilizeStartMs[0]);
    if (stabilizeElapsed < DRY_STABILIZE_MS) {
      return; // Still in stabilization phase
    }
    
    // Stabilization complete, perform dry-check
    float diff = g_dhtAHDiff[0];
    bool stillWet = (diff > AH_DRY_THRESHOLD);
    FSM_DBG_PRINT("SUB1: COOLING stabilization done -> dry-check, diff=");
    FSM_DBG_PRINTLN(diff);
    g_subCoolingStartMs[0] = 0;
    g_subCoolingStabilizeStartMs[0] = 0;
    g_coolingLocked[0] = false;
    if (stillWet) {
      FSM_DBG_PRINTLN("SUB1: COOLING dry-check -> still wet, returning to WAITING");
      fsmSub1.handleEvent(Event::DryCheckFailed);
    } else {
      FSM_DBG_PRINTLN("SUB1: COOLING dry-check -> dry, advancing to DRY");
      fsmSub1.handleEvent(Event::SubStart);
    }
  });

  fsmSub2.setRun(SubState::S_COOLING, []() {
    if (g_subCoolingStartMs[1] == 0)
      return;
    uint32_t motorElapsed = (uint32_t)(millis() - g_subCoolingStartMs[1]);
    
    // Phase 1: motor running (0-5s)
    if (motorElapsed < DRY_COOL_MS) {
      return; // Still in motor-run phase
    }
    
    // Transition from motor phase to stabilization phase
    if (g_subCoolingStabilizeStartMs[1] == 0) {
      FSM_DBG_PRINTLN("SUB2: COOLING -> motor phase done, starting stabilization");
      motorStop(1);
      g_subCoolingStabilizeStartMs[1] = millis();
      return;
    }
    
    // Phase 2: stabilization (check if stabilization period elapsed)
    uint32_t stabilizeElapsed = (uint32_t)(millis() - g_subCoolingStabilizeStartMs[1]);
    if (stabilizeElapsed < DRY_STABILIZE_MS) {
      return; // Still in stabilization phase
    }
    
    // Stabilization complete, perform dry-check
    float diff = g_dhtAHDiff[1];
    bool stillWet = (diff > AH_DRY_THRESHOLD);
    FSM_DBG_PRINT("SUB2: COOLING stabilization done -> dry-check, diff=");
    FSM_DBG_PRINTLN(diff);
    g_subCoolingStartMs[1] = 0;
    g_subCoolingStabilizeStartMs[1] = 0;
    g_coolingLocked[1] = false;
    if (stillWet) {
      FSM_DBG_PRINTLN("SUB2: COOLING dry-check -> still wet, returning to WAITING");
      fsmSub2.handleEvent(Event::DryCheckFailed);
    } else {
      FSM_DBG_PRINTLN("SUB2: COOLING dry-check -> dry, advancing to DRY");
      fsmSub2.handleEvent(Event::SubStart);
    }
  });

  // S_DRY: on entry, motor should be turned off. If UV already finished during cooling
  // advance immediately. If UV isn't running (starting-from-DRY scenario), perform a
  // quick dry-check and start UV; otherwise wait for UV timer to complete.
  fsmSub1.setRun(SubState::S_DRY,
                 []() { /* no-op: advancement driven by UV timer or entry checks */ });

  // Detecting entry/exit
  fsmGlobal.setEntry(GlobalState::Detecting, []() {
    detectingStartMs = millis();
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Detecting - equalize timer started");
  });
  fsmGlobal.setExit(GlobalState::Detecting, []() {
    detectingStartMs = 0;
    FSM_DBG_PRINTLN("GLOBAL EXIT: Detecting - equalize timer cleared");
  });
  // Done entry: stop UVs but keep subs in their final states (COOLING/DRY/etc)
  // Subs will reset when user presses Start again (goes back to Idle first)
  fsmGlobal.setEntry(GlobalState::Done, []() {
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Done - stopping UVs");
    uvStop(0);
    uvStop(1);
    g_subDoneMask = 0;
    g_doneStartMs = millis();
  });

  // Ensure Done exit clears the done-start timestamp used for auto-reset
  fsmGlobal.setExit(GlobalState::Done, []() { g_doneStartMs = 0; });

  // Checking state: run callback to check battery before allowing transition to Running
  fsmGlobal.setRun(GlobalState::Checking, []() {
    // Battery check performed here; if user presses Start and battery is low, post BatteryLow event
    // The transition to Running will only occur if battery is OK
  });

  // Checking state entry: perform battery check immediately
  fsmGlobal.setEntry(GlobalState::Checking, []() {
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Checking - verifying battery voltage");
    // Turn on status LED when checking
    digitalWrite(HW_STATUS_LED_PIN, HIGH);
    digitalWrite(HW_ERROR_LED_PIN, LOW);
    if (!isBatteryOk()) {
      float vBat = readBatteryVoltage();
      FSM_DBG_PRINT("GLOBAL: Battery voltage low: ");
      if (Serial) Serial.printf("%.2f V\n", vBat);
      fsmPostEvent(Event::BatteryLow, false);
    }
  });

  // LowBattery state: continuously check battery and turn off status LED, turn on error LED
  fsmGlobal.setEntry(GlobalState::LowBattery, []() {
    FSM_DBG_PRINTLN("GLOBAL ENTRY: LowBattery - waiting for battery recovery");
    digitalWrite(HW_STATUS_LED_PIN, LOW);   // Status LED off
    digitalWrite(HW_ERROR_LED_PIN, HIGH);   // Error LED solid on
    g_lastBatteryCheckMs = millis();
  });

  fsmGlobal.setRun(GlobalState::LowBattery, []() {
    uint32_t now = millis();
    if ((uint32_t)(now - g_lastBatteryCheckMs) >= BATTERY_CHECK_INTERVAL_MS) {
      g_lastBatteryCheckMs = now;
      if (isBatteryRecovered()) {
        float vBat = readBatteryVoltage();
        FSM_DBG_PRINT("GLOBAL: Battery recovered: ");
        if (Serial) Serial.printf("%.2f V\n", vBat);
        fsmPostEvent(Event::BatteryRecovered, false);
      }
    }
  });

  fsmGlobal.setExit(GlobalState::LowBattery, []() {
    FSM_DBG_PRINTLN("GLOBAL EXIT: LowBattery");
    digitalWrite(HW_ERROR_LED_PIN, LOW);  // Turn off error LED
  });

  // Idle state: status LED on, error LED off
  fsmGlobal.setEntry(GlobalState::Idle, []() {
    detectingStartMs = 0;  // Clear detecting timer to prevent stale timeout
    
    // Full reset when entering Idle
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Idle - full reset");
    
    // Stop all motors, heaters, UVs
    motorStop(0);
    motorStop(1);
    heaterRun(0, false);
    heaterRun(1, false);
    uvStop(0);
    uvStop(1);
    
    // Reset WET lock
    g_wetLockOwner = -1;
    g_uvComplete[0] = g_uvComplete[1] = false;
    g_motorStarted[0] = g_motorStarted[1] = false;
    
    // Reset substates to IDLE via ResetPressed event
    fsmSub1.handleEvent(Event::ResetPressed);
    fsmSub2.handleEvent(Event::ResetPressed);
    
    // LEDs: Status on (idle), Error off
    digitalWrite(HW_STATUS_LED_PIN, HIGH);
    digitalWrite(HW_ERROR_LED_PIN, LOW);
  });

  // Running state: status LED off, error LED blinking + initialize subs
  fsmGlobal.setEntry(GlobalState::Running, []() {
    // Initialize substates
    bool s1Wet = g_dhtIsWet[0];
    bool s2Wet = g_dhtIsWet[1];
    FSM_DBG_PRINTLN("GLOBAL ENTRY: Running - initializing subs");
    g_uvComplete[0] = g_uvComplete[1] = false;
    g_motorStarted[0] = g_motorStarted[1] = false;
    g_wetLockOwner = -1; // Clear lock for new run
    // Directly handle init events to ensure substates transition immediately
    fsmSub1.handleEvent(s1Wet ? Event::Shoe0InitWet : Event::Shoe0InitDry);
    fsmSub2.handleEvent(s2Wet ? Event::Shoe1InitWet : Event::Shoe1InitDry);
    
    // LED initialization for running state
    digitalWrite(HW_STATUS_LED_PIN, LOW);   // Status LED off when running
    digitalWrite(HW_ERROR_LED_PIN, LOW);    // Start with error LED off
    g_ledBlinkMs = millis();
    g_ledBlinkState = false;
  });

  // Error state: status LED off, error LED solid on
  fsmGlobal.setEntry(GlobalState::Error, []() {
    digitalWrite(HW_STATUS_LED_PIN, LOW);
    digitalWrite(HW_ERROR_LED_PIN, HIGH);  // Solid on for error
  });
}

// FreeRTOS task that drives all FSMs
static void vStateMachineTask(void * /*pvParameters*/) {
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);
  
  // Initialize battery ADC
  analogReadResolution(12);
  analogSetPinAttenuation(HW_BATTERY_ADC_PIN, ADC_11db);
  pinMode(HW_BATTERY_ADC_PIN, INPUT);
  
  // Initialize status LEDs
  pinMode(HW_STATUS_LED_PIN, OUTPUT);
  pinMode(HW_ERROR_LED_PIN, OUTPUT);
  digitalWrite(HW_STATUS_LED_PIN, HIGH);  // Status LED on at startup
  digitalWrite(HW_ERROR_LED_PIN, LOW);    // Error LED off at startup
  
  if (!g_fsmEventQ)
    g_fsmEventQ = xQueueCreate(FSM_QUEUE_LEN, sizeof(EventMsg));
  uvInit();
  setupStateMachines();
  FSM_DBG_PRINTLN("FSM Task started");

  auto forwardToSubs = [](Event e) {
    fsmSub1.handleEvent(e);
    fsmSub2.handleEvent(e);
  };

  while (true) {
    if (readStart()) {
      GlobalState gs = fsmGlobal.getState();
      // Only allow start from Idle or Checking states
      if (gs == GlobalState::Idle || gs == GlobalState::Checking) {
        if (gs == GlobalState::Running)
          fsmPostEvent(Event::SubStart, false);
        else
          fsmPostEvent(Event::StartPressed, false);
      }
    }
    if (readReset())
      fsmPostEvent(Event::ResetPressed, true);

    if (detectingStartMs != 0) {
      uint32_t now = millis();
      if ((uint32_t)(now - detectingStartMs) >= SENSOR_EQUALIZE_MS) {
        FSM_DBG_PRINTLN("GLOBAL: Detecting timeout -> SensorTimeout");
        fsmPostEvent(Event::SensorTimeout, false);
        detectingStartMs = 0;
      }
    }

    if (g_fsmEventQ) {
      EventMsg m;
      if (xQueueReceive(g_fsmEventQ, &m, 0) == pdTRUE) {
        FSM_DBG_PRINT("FSM: dequeued event -> ");
        FSM_DBG_PRINT_INT("", (int)m.ev);
        bool globalConsumed = fsmGlobal.handleEvent(m.ev);
        if (m.broadcastAll)
          forwardToSubs(m.ev);
        else if (!globalConsumed) {
          switch (m.ev) {
          case Event::Shoe0InitWet:
          case Event::Shoe0InitDry:
            fsmSub1.handleEvent(m.ev);
            break;
          case Event::Shoe1InitWet:
          case Event::Shoe1InitDry:
            fsmSub2.handleEvent(m.ev);
            break;
          case Event::SubStart: {
            // Gate SubStart: only deliver to shoes in WET or WAITING (acquiring lock)
            // This prevents DRY shoe from advancing to DONE when another shoe posts SubStart
            SubState s1 = fsmSub1.getState();
            SubState s2 = fsmSub2.getState();
            if ((s1 == SubState::S_WET || s1 == SubState::S_WAITING) &&
                !(s1 == SubState::S_COOLING && g_coolingLocked[0])) {
              fsmSub1.handleEvent(m.ev);
            }
            if ((s2 == SubState::S_WET || s2 == SubState::S_WAITING) &&
                !(s2 == SubState::S_COOLING && g_coolingLocked[1])) {
              fsmSub2.handleEvent(m.ev);
            }
            break;
          }
          case Event::UVTimer0:
            // Single UV timer expired: advance BOTH subs if they are in DRY
            FSM_DBG_PRINTLN("UV timer expired on GPIO14");
            if (fsmSub1.getState() == SubState::S_DRY) {
              FSM_DBG_PRINTLN("SUB1: in DRY, advancing to DONE");
              fsmSub1.handleEvent(Event::SubStart);
            }
            if (fsmSub2.getState() == SubState::S_DRY) {
              FSM_DBG_PRINTLN("SUB2: in DRY, advancing to DONE");
              fsmSub2.handleEvent(Event::SubStart);
            }
            // Mark UV complete so we don't restart it
            g_uvComplete[0] = true;
            break;
          case Event::UVTimer1:
            // UVTimer1 not used with single UV, ignore
            FSM_DBG_PRINTLN("UVTimer1 ignored (single UV mode)");
            break;
          case Event::SubFSMDone:
            if (fsmSub1.getState() == SubState::S_DONE && fsmSub2.getState() == SubState::S_DONE)
              fsmGlobal.handleEvent(m.ev);
            break;
          default:
            break;
          }
        }
      }
    }
    // Auto-reset: if we entered Done, after DONE_TIMEOUT_MS reset to Idle (global only, not subs)
    if (g_doneStartMs != 0) {
      uint32_t now = millis();
      if ((uint32_t)(now - g_doneStartMs) >= DONE_TIMEOUT_MS) {
        FSM_DBG_PRINTLN("GLOBAL: Done timeout -> Reset to Idle (global only)");
        // Post ResetPressed with broadcastAll=false to only reset global FSM
        fsmPostEvent(Event::ResetPressed, /*broadcastAll=*/false);
        g_doneStartMs = 0;
      }
    }

    // Handle LED blinking for Running (error LED) and Done (status LED)
    auto gsNow = fsmGlobal.getState();
    if (gsNow == GlobalState::Running) {
      uint32_t now = millis();
      if ((uint32_t)(now - g_ledBlinkMs) >= 500u) {  // Blink every 500ms
        g_ledBlinkMs = now;
        g_ledBlinkState = !g_ledBlinkState;
        digitalWrite(HW_ERROR_LED_PIN, g_ledBlinkState ? HIGH : LOW);
      }
    } else if (gsNow == GlobalState::Done) {
      uint32_t now = millis();
      if ((uint32_t)(now - g_ledBlinkMs) >= 500u) {  // Blink every 500ms
        g_ledBlinkMs = now;
        g_ledBlinkState = !g_ledBlinkState;
        digitalWrite(HW_STATUS_LED_PIN, g_ledBlinkState ? HIGH : LOW);
      }
      // Ensure error LED stays off in Done
      digitalWrite(HW_ERROR_LED_PIN, LOW);
    } else if (gsNow == GlobalState::LowBattery) {
      // Keep error LED solid on while in LowBattery
      digitalWrite(HW_ERROR_LED_PIN, HIGH);
    } else {
      // Not Running or Done: keep error LED off unless state-specific entry sets it
      digitalWrite(HW_ERROR_LED_PIN, LOW);
    }

    fsmGlobal.run();
    vTaskDelay(pdMS_TO_TICKS(FSM_LOOP_DELAY_MS));
  }
}

void createStateMachineTask() {
  motorInit();
  xTaskCreatePinnedToCore(vStateMachineTask, "StateMachineTask", 4096, nullptr, 1, nullptr, 1);
}

GlobalState getGlobalState() {
  return fsmGlobal.getState();
}
SubState getSub1State() {
  return fsmSub1.getState();
}
SubState getSub2State() {
  return fsmSub2.getState();
}