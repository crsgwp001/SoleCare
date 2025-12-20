#include "tskFSM.h"
#include "global.h"
#include "fsm_debug.h"
#include "tskMotor.h"
#include "tskUV.h"
#include "tskUI.h"
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
// Track if early dry exit already triggered to prevent repeated events
static bool g_coolingEarlyExit[2] = {false, false};

// Track AH rate-of-change during WET to detect peak evaporation
// Used to exit WET state when drying rate starts declining
static float g_prevAHRate[2] = {0.0f, 0.0f};  // Previous AH rate-of-change sample
static uint32_t g_lastAHRateSampleMs[2] = {0, 0};  // Timestamp of last sample
static int g_ahRateSampleCount[2] = {0, 0};  // Number of samples collected
static int g_consecutiveNegativeCount[2] = {0, 0};  // Consecutive negative rate changes
static bool g_peakDetected[2] = {false, false};  // Flag: peak evaporation detected, in post-peak buffer
static uint32_t g_peakDetectedMs[2] = {0, 0};  // Timestamp when peak was first detected
// Moving-average buffers for robust decline detection
static float g_rateHistory[2][8] = {{0}}; // last 8 samples per shoe
static int g_rateHistoryIdx[2] = {0, 0};
static int g_rateHistoryCount[2] = {0, 0};
static uint32_t g_coolingMotorDurationMs[2] = {DRY_COOL_MS_BASE, DRY_COOL_MS_BASE};  // Adaptive COOLING motor phase duration
static uint8_t g_coolingRetryCount[2] = {0, 0};  // Number of cooling retries within a cycle
static float g_coolingDiffSamples[2][6] = {{0.0f}};  // Last 6 AH diff samples during stabilization
static int g_coolingDiffSampleIdx[2] = {0, 0};  // Current sample index
static int g_coolingDiffSampleCount[2] = {0, 0};  // Number of samples collected
// Re-evap short cycle tracking
static bool g_inReEvap[2] = {false, false};
static uint32_t g_reEvapStartMs[2] = {0, 0};
static float g_reEvapMinDiff[2] = {999.0f, 999.0f};
static uint32_t g_reEvapMinDiffMs[2] = {0, 0};
constexpr int MIN_AH_RATE_SAMPLES = 5;  // Need at least this many samples before checking decline
constexpr int MIN_CONSECUTIVE_NEGATIVE = 3;  // Need at least 3 consecutive negative samples to exit WET (robust to noise)
constexpr float AH_RATE_DECLINE_THRESHOLD = -0.01f;  // Rate-of-change decline to trigger exit (g/m³/min²)
// Additional robustness: for very wet shoes, require sustained decline over longer history
constexpr int MIN_SAMPLES_FOR_DECLINE = 6;  // Track at least 6 samples before considering decline valid
constexpr float AH_RATE_PEAK_MIN_THRESHOLD = 0.3f;  // Don't declare peak until rate drops below this (g/m³/min) - prevents early exit on very wet shoes
// Track whether the UV timer expired while the sub was in the COOLING phase.
static bool g_uvExpiredDuringCooling[2] = {false, false};
// UV complete flag: set when a UV timer has finished for a sub. Prevents
// restarting the UV timer on subsequent loops between cooling->wet.
static bool g_uvComplete[2] = {false, false};
// Track initial wetness per shoe to adapt minimum WET duration and peak criteria
static float g_initialWetDiff[2] = {0.0f, 0.0f};
static uint32_t g_wetMinDurationMs[2] = {WET_MODERATE_MS, WET_MODERATE_MS};  // Default to moderate; will be reassigned
// Post-peak buffer duration (adaptive based on initial wetness)
static uint32_t g_peakBufferMs[2] = {WET_BUFFER_MODERATE_MS, WET_BUFFER_MODERATE_MS};  // Default to moderate; will be reassigned
// Track last AH diff during WET phase (for continuous monitoring)
static float g_lastWetAHDiff[2] = {0.0f, 0.0f};
// Prevent accidental early exit if AH diff jumps down (sensor noise)
static uint32_t g_lastAHDiffCheckMs[2] = {0, 0};
static float g_lastValidAHDiff[2] = {0.0f, 0.0f};  // Last stable AH diff reading
// Track MINIMUM AH diff seen during WET (for detecting true evaporation peak)
static float g_minAHDiffSeen[2] = {999.0f, 999.0f};  // Initialize high
static uint32_t g_minAHDiffSeenMs[2] = {0, 0};  // When minimum was observed
// Protect g_uvComplete access since it's read from UI (core 0) and written by FSM (core 1)
// (g_uvComplete is internal to FSM)

// Track whether we have started the motor for a sub during the current wet->..->dry cycle.
static bool g_motorStarted[2] = {false, false};

// Sequential WET lock: only one shoe can be in S_WET at a time
// -1 = free, 0 = shoe 0 owns the lock, 1 = shoe 1 owns the lock
static int g_wetLockOwner = -1;
// Guard to prevent repeated handleEvent in WAITING run callbacks (watchdog protection)
static bool g_waitingEventPosted[2] = {false, false};

// Helper: Classify wetness level and assign adaptive durations
static void assignAdaptiveWETDurations(uint8_t idx, float ahDiff) {
  if (ahDiff < AH_DIFF_BARELY_WET) {
    // Barely wet - quick dry
    g_wetMinDurationMs[idx] = WET_BARELY_WET_MS;
    g_peakBufferMs[idx] = WET_BUFFER_BARELY_WET_MS;
    FSM_DBG_PRINT("BARELY_WET");
  } else if (ahDiff < AH_DIFF_MODERATE_WET) {
    // Moderately wet - normal dry
    g_wetMinDurationMs[idx] = WET_MODERATE_MS;
    g_peakBufferMs[idx] = WET_BUFFER_MODERATE_MS;
    FSM_DBG_PRINT("MODERATE_WET");
  } else if (ahDiff < AH_DIFF_VERY_WET) {
    // Very wet - extended dry
    g_wetMinDurationMs[idx] = WET_VERY_WET_MS;
    g_peakBufferMs[idx] = WET_BUFFER_VERY_WET_MS;
    FSM_DBG_PRINT("VERY_WET");
  } else {
    // Soaked - full dry
    g_wetMinDurationMs[idx] = WET_SOAKED_MS;
    g_peakBufferMs[idx] = WET_BUFFER_SOAKED_MS;
    FSM_DBG_PRINT("SOAKED");
  }
}

static inline bool coolingMotorPhaseActive(int idx) {
  // Motor phase is active if cooling started and stabilization hasn't begun yet
  return (g_subCoolingStartMs[idx] != 0 && g_subCoolingStabilizeStartMs[idx] == 0);
}

// Compute adaptive heater warmup based on shoe temperature
static uint32_t getAdaptiveWarmupMs(uint8_t idx) {
  // Sensor indices: 0=ambient, 1=shoe0, 2=shoe1
  float tempC = g_dhtTemp[idx + 1];
  if (isnan(tempC)) return HEATER_WARMUP_MS;
  if (tempC >= HEATER_WARMUP_FAST_35C) return HEATER_WARMUP_35C_MS;
  if (tempC >= HEATER_WARMUP_FAST_30C) return HEATER_WARMUP_30C_MS;
  if (tempC >= HEATER_WARMUP_FAST_25C) return HEATER_WARMUP_25C_MS;
  return HEATER_WARMUP_MS;
}

// Early heater efficiency with smart trend-based control
// Tracks temperature direction to prevent churn
static float g_heaterLastTemp[2] = {NAN, NAN};
static uint32_t g_heaterLastCheckMs[2] = {0, 0};
static uint8_t g_heaterTrendSamples[2] = {0, 0};  // Count of temp samples to establish trend
static bool g_heaterTempRising[2] = {false, false};  // true if temp is rising

// Warmup phase (heater + motor): runs until shoe reaches threshold temp, capped by time
// During this phase, maybeEarlyHeaterOff() is disabled to let heater warm shoe without interference
constexpr uint32_t HEATER_WARMUP_MIN_MS = 30u * 1000u;  // 30 seconds minimum
constexpr uint32_t HEATER_WARMUP_EXTENDED_MS = 50u * 1000u;  // 50 seconds for cold shoes
static uint32_t g_heaterWarmupStartMs[2] = {0, 0};  // 0 = not in warmup phase
static bool g_heaterWarmupDone[2] = {false, false};  // Track if warmup has been completed for this cycle
static constexpr float HEATER_WET_TEMP_THRESHOLD_C = 38.0f; // Temp to end unconditional heater-on

static void maybeEarlyHeaterOff(uint8_t idx, uint32_t /*wetElapsedMs*/) {
  // Smart bang-bang: OFF at 39°C, track trend (rising/falling), only turn ON if falling
  // This prevents churn and lets residual heat help evaporate before reheating
  
  float tempC = g_dhtTemp[idx + 1];
  if (isnan(tempC)) return;
  
  uint32_t now = millis();
  
  // ===== DETERMINE TEMPERATURE TREND (every 500ms) =====
  if ((now - g_heaterLastCheckMs[idx]) >= 500) {
    if (!isnan(g_heaterLastTemp[idx])) {
      // Compare to previous reading
      bool rising = (tempC > g_heaterLastTemp[idx]);
      if (rising == g_heaterTempRising[idx]) {
        // Same direction, accumulate confidence
        if (g_heaterTrendSamples[idx] < 5) g_heaterTrendSamples[idx]++;
      } else {
        // Direction changed, reset confidence
        g_heaterTempRising[idx] = rising;
        g_heaterTrendSamples[idx] = 1;
      }
    }
    g_heaterLastTemp[idx] = tempC;
    g_heaterLastCheckMs[idx] = now;
  }
  
  // ===== HEATER CONTROL =====
  // OFF threshold: 38°C (single threshold). Force OFF unconditionally to avoid state/race issues.
  // Throttle log spam to once every 5s or on state change
  static uint32_t s_lastHeaterLogMs[2] = {0, 0};
  static bool s_lastHeaterStateOn[2] = {false, false};
  if (tempC >= HEATER_WET_TEMP_THRESHOLD_C) {
    uint32_t nowMs = millis();
    bool shouldLog = (!s_lastHeaterStateOn[idx]) || (nowMs - s_lastHeaterLogMs[idx] >= 5000);
    if (shouldLog) {
      FSM_DBG_PRINT("SUB"); FSM_DBG_PRINT(idx);
      FSM_DBG_PRINT(": Heater OFF at ");
      FSM_DBG_PRINT(tempC, 1);
      FSM_DBG_PRINTLN("C (threshold reached)");
      s_lastHeaterLogMs[idx] = nowMs;
      s_lastHeaterStateOn[idx] = false;
    }
    heaterRun(idx, false);
    return;
  }
  
  // ON condition: Turn ON only if temp is falling (not rising/stable)
  // This prevents the heater from chasing upward or oscillating
  if (!heaterIsOn(idx) && tempC < HEATER_WET_TEMP_THRESHOLD_C) {
    // Temperature below 39°C and heater is OFF
    // Only turn ON if we have confidence that temp is FALLING
    bool shouldTurnOn = (g_heaterTrendSamples[idx] >= 2) && !g_heaterTempRising[idx];
    
    if (shouldTurnOn) {
      uint32_t nowMs = millis();
      bool shouldLog = (s_lastHeaterStateOn[idx] == false) || (nowMs - s_lastHeaterLogMs[idx] >= 5000);
      if (shouldLog) {
        FSM_DBG_PRINT("SUB"); FSM_DBG_PRINT(idx);
        FSM_DBG_PRINT(": Heater ON: temp ");
        FSM_DBG_PRINT(tempC, 1);
        FSM_DBG_PRINTLN("C falling, resuming until 39C");
        s_lastHeaterLogMs[idx] = nowMs;
        s_lastHeaterStateOn[idx] = true;
      }
      heaterRun(idx, true);
      return;
    }
  }
}

// Check if AH diff is consistently declining during stabilization
static bool isAHDiffDeclining(int idx) {
  if (g_coolingDiffSampleCount[idx] < 4) return false;  // Need at least 4 samples
  
  // Check last 4 samples for declining trend
  int declineCount = 0;
  for (int i = 1; i < 4; i++) {
    int currIdx = (g_coolingDiffSampleIdx[idx] - i + 6) % 6;
    int prevIdx = (g_coolingDiffSampleIdx[idx] - i - 1 + 6) % 6;
    if (g_coolingDiffSamples[idx][currIdx] < g_coolingDiffSamples[idx][prevIdx]) {
      declineCount++;
    }
  }
  return declineCount >= 2;  // At least 2 out of 3 transitions show decline
}

// Configure cooling motor duty and duration based on moisture level and retry status
static void startCoolingPhase(int idx, bool isRetry) {
  // Turn heater OFF during COOLING: temperature needs to drop for shoes to cool down
  // (keeping heater ON was causing temperature to hit 43°C and re-evaporate moisture)
  heaterRun(idx, false);
  
  // Clear heater warmup and trend state so heater stays OFF during COOLING
  // This prevents maybeEarlyHeaterOff() from turning heater back ON
  g_heaterWarmupStartMs[idx] = 0;
  g_heaterWarmupDone[idx] = false;
  g_heaterLastTemp[idx] = NAN;  // Reset trend tracking
  g_heaterLastCheckMs[idx] = 0;
  g_heaterTrendSamples[idx] = 0;
  g_heaterTempRising[idx] = false;

  const char *label = (idx == 0) ? "SUB1" : "SUB2";
  float coolingDiff = g_dhtAHDiff[idx];

  // Base selections - shorter since heavy lifting done in WET phase
  uint32_t durationMs = DRY_COOL_MS_BASE;
  int dutyPercent = 70;  // Moderate circulation (was 80, reduced to 70 to prevent excessive reheating)

  if (coolingDiff > 2.0f) {
    // Still very wet after extended WET: moderate cooling
    dutyPercent = 75;  // Increased from 50-60 (was 100) - balance evaporation with preventing reheating
    durationMs = DRY_COOL_MS_SOAKED;  // 180s
    FSM_DBG_VPRINT(label);
    FSM_DBG_VPRINT(" COOLING: Still very wet (diff=");
    FSM_DBG_VPRINT(coolingDiff);
    FSM_DBG_VPRINTLN(") -> 75% duty, extended timing");
  } else if (coolingDiff > 1.2f) {
    // Moderately wet: moderate cooling
    dutyPercent = 72;  // Increased from 50-55 (was 90)
    durationMs = DRY_COOL_MS_WET;  // 150s
    FSM_DBG_VPRINT(label);
    FSM_DBG_VPRINT(" COOLING: Moderate (diff=");
    FSM_DBG_VPRINT(coolingDiff);
    FSM_DBG_VPRINTLN(") -> 72% duty, standard timing");
  } else {
    // Nearly dry: light circulation
    dutyPercent = 70;  // Increased from 50 (was 80)
    durationMs = DRY_COOL_MS_BASE;  // 90s
    FSM_DBG_VPRINT(label);
    FSM_DBG_VPRINT(" COOLING: Nearly dry (diff=");
    FSM_DBG_VPRINT(coolingDiff);
    FSM_DBG_VPRINTLN(") -> 70% duty, base timing");
  }

  // Retry path: moderate increase
  if (isRetry) {
    if (dutyPercent < 75) dutyPercent = 75;  // Cap at 75% to prevent excessive reheating
    if (durationMs < DRY_COOL_MS_WET) durationMs = DRY_COOL_MS_WET;  // Max 150s on retry
    FSM_DBG_VPRINT(label);
    FSM_DBG_VPRINTLN(" COOLING: Retry -> boosting to 75% duty, extended timing");
  }

  motorSetDutyPercent(idx, dutyPercent);
  g_coolingMotorDurationMs[idx] = durationMs;
  g_subCoolingStartMs[idx] = millis();
  g_subCoolingStabilizeStartMs[idx] = 0;
  g_coolingLocked[idx] = true;
  g_coolingEarlyExit[idx] = false;
  g_coolingDiffSampleIdx[idx] = 0;
  g_coolingDiffSampleCount[idx] = 0;
}

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
         g_waitingEventPosted[0] = false;  // Reset guard on entry
       },
       []() {
         FSM_DBG_PRINTLN("SUB1 EXIT: leaving WAITING");
       }},
      // S_WET: warmup (30-50s), then normal WET with trend-gated heater control
      {SubState::S_WET,
       []() {
         FSM_DBG_PRINTLN("SUB1 ENTRY: WET");
         // Initialize warmup phase
         g_heaterWarmupStartMs[0] = 0;  // Will be set on first run iteration
         g_heaterWarmupDone[0] = false;
         // Reset heater trend tracking (will be re-initialized during warmup)
         g_heaterLastTemp[0] = NAN;
         g_heaterLastCheckMs[0] = 0;
         g_heaterTrendSamples[0] = 0;
         g_heaterTempRising[0] = false;
         
         motorStop(0);
         g_motorStarted[0] = false;
         motorSetDutyPercent(0, 0);
         g_subWetStartMs[0] = millis();
         g_initialWetDiff[0] = g_dhtAHDiff[0];
         assignAdaptiveWETDurations(0, g_initialWetDiff[0]);
         g_lastValidAHDiff[0] = g_initialWetDiff[0];
         g_lastAHDiffCheckMs[0] = g_subWetStartMs[0];
         // Reset peak detection for new WET cycle
         g_peakDetected[0] = false;
         g_peakDetectedMs[0] = 0;
         g_minAHDiffSeen[0] = g_initialWetDiff[0];  // Start tracking minimum from initial value
         g_minAHDiffSeenMs[0] = g_subWetStartMs[0];
         g_coolingRetryCount[0] = 0;  // Reset cooling retries for new cycle
         FSM_DBG_PRINT("SUB1: WET entry ("); FSM_DBG_PRINT(g_initialWetDiff[0], 2);
         FSM_DBG_PRINT("g/m^3) -> ");
         // (classification printed in helper function)
         FSM_DBG_PRINT(", minDuration="); FSM_DBG_PRINT(g_wetMinDurationMs[0]/1000); FSM_DBG_PRINT("s, buffer="); 
         FSM_DBG_PRINT(g_peakBufferMs[0]/1000); FSM_DBG_PRINTLN("s");
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
         startCoolingPhase(0, false);
       },
       []() {
         FSM_DBG_PRINTLN("SUB1 EXIT: leaving COOLING"); /* leaving cooling */
         // Stop motor explicitly when leaving COOLING to ensure fan is off in DRY
         motorStop(0);
         g_motorStarted[0] = false;
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
         g_waitingEventPosted[1] = false;  // Reset guard on entry
       },
       []() {
         FSM_DBG_PRINTLN("SUB2 EXIT: leaving WAITING");
       }},
      {SubState::S_WET,
       []() {
         FSM_DBG_PRINTLN("SUB2 ENTRY: WET");
         // Initialize warmup phase
         g_heaterWarmupStartMs[1] = 0;  // Will be set on first run iteration
         g_heaterWarmupDone[1] = false;
         // Reset heater trend tracking (will be re-initialized during warmup)
         g_heaterLastTemp[1] = NAN;
         g_heaterLastCheckMs[1] = 0;
         g_heaterTrendSamples[1] = 0;
         g_heaterTempRising[1] = false;
         
         motorStop(1);
         g_motorStarted[1] = false;
         motorSetDutyPercent(1, 0);
         g_subWetStartMs[1] = millis();
         g_initialWetDiff[1] = g_dhtAHDiff[1];
         assignAdaptiveWETDurations(1, g_initialWetDiff[1]);
         g_lastValidAHDiff[1] = g_initialWetDiff[1];
         g_lastAHDiffCheckMs[1] = g_subWetStartMs[1];
         // Reset peak detection for new WET cycle
         g_peakDetected[1] = false;
         g_peakDetectedMs[1] = 0;
         g_minAHDiffSeen[1] = g_initialWetDiff[1];  // Start tracking minimum from initial value
         g_minAHDiffSeenMs[1] = g_subWetStartMs[1];
         g_coolingRetryCount[1] = 0;  // Reset cooling retries for new cycle
         FSM_DBG_PRINT("SUB2: WET entry ("); FSM_DBG_PRINT(g_initialWetDiff[1], 2);
         FSM_DBG_PRINT("g/m^3) -> ");
         // (classification printed in helper function)
         FSM_DBG_PRINT(", minDuration="); FSM_DBG_PRINT(g_wetMinDurationMs[1]/1000); FSM_DBG_PRINT("s, buffer="); 
         FSM_DBG_PRINT(g_peakBufferMs[1]/1000); FSM_DBG_PRINTLN("s");
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
         startCoolingPhase(1, false);
       },
       []() {
         FSM_DBG_PRINTLN("SUB2 EXIT: leaving COOLING");
         // Stop motor explicitly when leaving COOLING to ensure fan is off in DRY
         motorStop(1);
         g_motorStarted[1] = false;
         g_coolingLocked[1] = false;
       }},
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
    // Guard against repeated event posting (watchdog protection)
    if (g_waitingEventPosted[0])
      return;

    // Do not start WET if the other shoe is in COOLING motor phase (avoid motor overlap)
    if (coolingMotorPhaseActive(1))
      return;

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
          g_waitingEventPosted[0] = true;
          fsmSub1.handleEvent(Event::SubStart);
        }
        // else: wait for SUB2 to acquire lock

          // Read and cache battery voltage for UI display
          g_lastBatteryVoltage = readBatteryVoltage();
      } else {
        // SUB2 not waiting, acquire lock
        g_wetLockOwner = 0;
        FSM_DBG_PRINTLN("SUB1: Acquired WET lock");
        g_waitingEventPosted[0] = true;
        fsmSub1.handleEvent(Event::SubStart);
      }
    }
  });

  fsmSub2.setRun(SubState::S_WAITING, []() {
    // Guard against repeated event posting (watchdog protection)
    if (g_waitingEventPosted[1])
      return;

    // Do not start WET if the other shoe is in COOLING motor phase (avoid motor overlap)
    if (coolingMotorPhaseActive(0))
      return;

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
          g_waitingEventPosted[1] = true;
          fsmSub2.handleEvent(Event::SubStart);
        }
        // else: wait for SUB1 to acquire lock
      } else {
        // SUB1 not waiting, acquire lock
        g_wetLockOwner = 1;
        FSM_DBG_PRINTLN("SUB2: Acquired WET lock");
        g_waitingEventPosted[1] = true;
        fsmSub2.handleEvent(Event::SubStart);
      }
    }
  });

  // S_WET run: WARMUP PHASE (30-50s), then normal WET with trend-gated heater control
  fsmSub1.setRun(SubState::S_WET, []() {
    uint32_t now = millis();
    uint32_t wetElapsed = (g_subWetStartMs[0] != 0) ? (now - g_subWetStartMs[0]) : 0;
    
    // ==================== WARMUP PHASE (30-50s at 60% motor + heater) ====================
    // During this phase, heater warms shoe WITHOUT trend-gated control interference
    // Motor runs at fixed 60% to avoid friction heating
    if (g_heaterWarmupStartMs[0] == 0 && !g_heaterWarmupDone[0]) {
      // First time entering WET - start warmup
      g_heaterWarmupStartMs[0] = now;
      heaterRun(0, true);  // Heater ON
      // NOTE: Do NOT call motorStart() yet - only set duty directly at 60%
      // motorStart() initializes PID which immediately overrides our 60% setpoint
      motorSetDutyPercent(0, 60);  // Manual 60% duty without PID
      FSM_DBG_PRINTLN("SUB1: Warmup phase START (30-50s at 60% motor + heater)");
    }
    
    // Monitor heater warmup phase progression
    if (g_heaterWarmupStartMs[0] != 0 && !g_heaterWarmupDone[0]) {
      uint32_t warmupElapsed = now - g_heaterWarmupStartMs[0];
      
      // Determine warmup duration (cold shoe gets extra time)
      uint32_t targetWarmupMs = HEATER_WARMUP_MIN_MS;  // 30s default
      float shoeTemp = g_dhtTemp[1];  // Sensor 1 = shoe 0
      if (!isnan(shoeTemp) && shoeTemp < 25.0f) {
        targetWarmupMs = HEATER_WARMUP_EXTENDED_MS;  // 50s for cold shoes
        if (warmupElapsed == 0 || (warmupElapsed >= HEATER_WARMUP_MIN_MS - 1000 && warmupElapsed < HEATER_WARMUP_MIN_MS)) {
          FSM_DBG_PRINT("SUB1: Cold shoe detected (");
          FSM_DBG_PRINT(shoeTemp, 1);
          FSM_DBG_PRINTLN("C) -> extended warmup 50s");
        }
      }

      // End warmup immediately once threshold temp is reached
      if (!isnan(shoeTemp) && shoeTemp >= HEATER_WET_TEMP_THRESHOLD_C) {
        FSM_DBG_PRINT("SUB1: Warmup threshold reached (");
        FSM_DBG_PRINT(shoeTemp, 1);
        FSM_DBG_PRINTLN("C) -> heater OFF, switch to trend-gated control");
        g_heaterWarmupDone[0] = true;
        // Turn heater OFF at threshold, then allow trend gating to manage it
        heaterRun(0, false);
        // Start motor PID control now
        motorStart(0);
        // Reset AH rate tracking when starting PID control
        g_ahRateSampleCount[0] = 0;
        g_consecutiveNegativeCount[0] = 0;
        g_lastAHRateSampleMs[0] = now;
        g_prevAHRate[0] = 0.0f;
      } else if (warmupElapsed < targetWarmupMs) {
        // Still in warmup phase - keep heater ON, motor at 60%
        return;  // Skip the rest of WET logic until warmup is done
      } else {
        // Time-based completion fallback
        FSM_DBG_PRINTLN("SUB1: Warmup time complete -> transition to trend-gated WET (PID motor control)");
        g_heaterWarmupDone[0] = true;
        motorStart(0);
        g_ahRateSampleCount[0] = 0;
        g_consecutiveNegativeCount[0] = 0;
        g_lastAHRateSampleMs[0] = now;
        g_prevAHRate[0] = 0.0f;
      }
    }
    
    // ==================== NORMAL WET PHASE (after warmup) ====================
    // Heater is now controlled by trend-gating (maybeEarlyHeaterOff)
    // Motor duty will be managed by PID control (AH rate monitoring)

    if (g_heaterWarmupDone[0] && g_subWetStartMs[0] != 0) {
      // Apply trend-based heater control throughout the remainder of WET
      maybeEarlyHeaterOff(0, wetElapsed);
    }

    // Sample AH rate every 2 seconds and check for peak (declining rate)
    // But wait AH_ACCEL_WARMUP_MS (30s) after WET start before checking for decline
    if (g_heaterWarmupDone[0] && g_subWetStartMs[0] != 0) {
      // Early return if sampling interval hasn't elapsed (prevent tight loop)
      if ((uint32_t)(now - g_lastAHRateSampleMs[0]) < 2000) {
        return;
      }
      
      g_lastAHRateSampleMs[0] = now;
      float currentRate = g_dhtAHRate[0];  // Use actual AH rate-of-change from motor control
      float currentAHDiff = g_dhtAHDiff[0];  // Get current AH diff for safety checks
      
      // Validate rate before processing (reject NaN/Inf from sensor glitches)
      if (isnan(currentRate) || isinf(currentRate)) {
        return;  // Skip this sample, wait for next valid rate
      }
      
      // ==================== PEAK-DETECTED: IN POST-PEAK BUFFER PHASE ====================
      if (g_peakDetected[0]) {
        uint32_t bufferElapsed = (uint32_t)(now - g_peakDetectedMs[0]);
        
        // SAFETY CHECK: Verify AH diff hasn't dropped to unexpected low values (sensor noise/glitch)
        if (!isnan(currentAHDiff) && currentAHDiff > 0.1f) {
          g_lastValidAHDiff[0] = currentAHDiff;
          g_lastAHDiffCheckMs[0] = now;
        }
        
        // Still in post-peak buffer period
        if (bufferElapsed < g_peakBufferMs[0]) {
          // Continue monitoring evaporation during buffer
          uint32_t bufferRemaining = g_peakBufferMs[0] - bufferElapsed;
          FSM_DBG_PRINT("SUB1: WET in post-peak buffer (");
          FSM_DBG_PRINT(bufferRemaining);
          FSM_DBG_PRINT("ms remaining, diff=");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3)");
          return;
        }
        
        // Buffer period expired - check safety conditions before exiting to COOLING
        uint32_t minDurationRemaining = (wetElapsed >= g_wetMinDurationMs[0]) ? 0 : (g_wetMinDurationMs[0] - wetElapsed);
        
        // Dynamic buffer extension: if current AH rose above initial, extend buffer duration
        // This handles shoes that got wetter during WET phase
        uint32_t adaptiveBufferMs = g_peakBufferMs[0];
        if (currentAHDiff > g_initialWetDiff[0] + 0.5f) {
          // Shoe got significantly wetter during WET phase - extend buffer
          // Use 4-tier thresholds based on CURRENT moisture level
          if (currentAHDiff < 1.5f) {
            adaptiveBufferMs = WET_BUFFER_BARELY_WET_MS;  // 40s
          } else if (currentAHDiff < 3.5f) {
            adaptiveBufferMs = WET_BUFFER_MODERATE_MS;    // 75s (normal)
          } else if (currentAHDiff < 5.0f) {
            adaptiveBufferMs = WET_BUFFER_VERY_WET_MS;    // 100s
          } else {
            adaptiveBufferMs = WET_BUFFER_SOAKED_MS;      // 120s
          }
          
          // Re-check if buffer has actually expired with the new duration
          if (bufferElapsed < adaptiveBufferMs) {
            uint32_t adaptiveRemaining = adaptiveBufferMs - bufferElapsed;
            FSM_DBG_PRINT("SUB1: WET buffer extended (moisture rose: ");
            FSM_DBG_PRINT(g_initialWetDiff[0], 1);
            FSM_DBG_PRINT(" -> ");
            FSM_DBG_PRINT(currentAHDiff, 1);
            FSM_DBG_PRINT("g/m^3), remaining=");
            FSM_DBG_PRINT(adaptiveRemaining);
            FSM_DBG_PRINTLN("ms");
            return;
          }
        }
      
        // Temperature-based buffer hold: if shoe is still hot, extend buffer by 30s
        float tC0 = g_dhtTemp[1];
        if (!isnan(tC0) && tC0 >= WET_BUFFER_TEMP_HOT_C) {
          if (bufferElapsed < g_peakBufferMs[0] + WET_BUFFER_TEMP_EXTEND_MS) {
            uint32_t remain = (g_peakBufferMs[0] + WET_BUFFER_TEMP_EXTEND_MS) - bufferElapsed;
            FSM_DBG_PRINT("SUB1: WET buffer temp-hold (t=");
            FSM_DBG_PRINT(tC0, 1);
            FSM_DBG_PRINT("C), remaining=");
            FSM_DBG_PRINT(remain);
            FSM_DBG_PRINTLN("ms");
            return;
          }
        }
        
        // SAFETY: Require AH diff to still be reasonable (not accidental low reading)
        bool safeAHLevel = (currentAHDiff > AH_DIFF_SAFETY_MARGIN) || (g_lastValidAHDiff[0] > AH_DIFF_SAFETY_MARGIN + 0.2f);
        bool minDurationMet = (minDurationRemaining == 0);
        
        if (minDurationMet && safeAHLevel) {
          FSM_DBG_PRINT("SUB1: WET peak + buffer complete, diff=");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3 -> transition to COOLING");
          fsmSub1.handleEvent(Event::SubStart);
          g_ahRateSampleCount[0] = 0;
          g_consecutiveNegativeCount[0] = 0;
          g_peakDetected[0] = false;
          g_peakDetectedMs[0] = 0;
          return;
        } else if (minDurationMet && !safeAHLevel) {
          // AH diff dropped too low - might be sensor glitch, wait longer
          FSM_DBG_PRINT("SUB1: WET safety check - diff dropped to ");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3 (below safety margin), waiting for stabilization...");
          return;
        } else {
          // Minimum duration not yet reached
          FSM_DBG_PRINT("SUB1: WET waiting for minimum duration (");
          FSM_DBG_PRINT(minDurationRemaining);
          FSM_DBG_PRINT("ms remaining, diff=");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3)");
          return;
        }
      }
      
      // ==================== BEFORE PEAK: MONITOR EVAPORATION RATE ====================
      // Track minimum AH diff seen (to detect true evaporation bottom)
      if (wetElapsed >= AH_ACCEL_WARMUP_MS && currentAHDiff < g_minAHDiffSeen[0]) {
        g_minAHDiffSeen[0] = currentAHDiff;
        g_minAHDiffSeenMs[0] = now;
      }
      
      // Early peak detection: if AH has risen significantly above minimum, we passed the peak
      // Adaptive gate by initial wetness
      uint32_t minRiseTime0;
      float riseThreshold0;
      if (g_initialWetDiff[0] < AH_DIFF_BARELY_WET) { // barely wet
        minRiseTime0 = 60000u; riseThreshold0 = 0.6f;
      } else if (g_initialWetDiff[0] < AH_DIFF_MODERATE_WET) { // moderate
        minRiseTime0 = 90000u; riseThreshold0 = 0.6f;
      } else if (g_initialWetDiff[0] < AH_DIFF_VERY_WET) { // very wet
        minRiseTime0 = 120000u; riseThreshold0 = 0.8f;
      } else { // soaked
        minRiseTime0 = 150000u; riseThreshold0 = 1.0f;
      }
      if (wetElapsed >= minRiseTime0 && g_minAHDiffSeen[0] < g_initialWetDiff[0] - 0.5f) {
        // We've seen a significant drop (good evaporation happened)
        float riseFromMin = currentAHDiff - g_minAHDiffSeen[0];
        if (riseFromMin > riseThreshold0) {
          // AH has risen >riseThreshold g/m^3 from minimum - we missed the peak!
          FSM_DBG_PRINT("SUB1: WET RISE detection - min was ");
          FSM_DBG_PRINT(g_minAHDiffSeen[0], 2);
          FSM_DBG_PRINT(" now ");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINT(" (rose +");
          FSM_DBG_PRINT(riseFromMin, 2);
          FSM_DBG_PRINTLN(") -> peak passed, starting buffer");
          g_peakDetected[0] = true;
          g_peakDetectedMs[0] = g_minAHDiffSeenMs[0];  // Use time when minimum was seen
          g_consecutiveNegativeCount[0] = 0;
          g_lastValidAHDiff[0] = currentAHDiff;
          heaterRun(0, false);
          return;
        }
      }
      
      // Peak detection using moving-average decline
      if (wetElapsed >= AH_ACCEL_WARMUP_MS) {
        // Update rate history buffer
        g_rateHistory[0][g_rateHistoryIdx[0]] = currentRate;
        g_rateHistoryIdx[0] = (g_rateHistoryIdx[0] + 1) % 8;
        if (g_rateHistoryCount[0] < 8) g_rateHistoryCount[0]++;

        if (g_rateHistoryCount[0] >= 6) { // need enough samples
          // Compute averages for two windows: recent (last 3) vs previous (prior 3)
          float recentAvg = 0.0f, previousAvg = 0.0f;
          // Indices for last 3 samples
          int i2 = (g_rateHistoryIdx[0] - 1 + 8) % 8;
          int i1 = (g_rateHistoryIdx[0] - 2 + 8) % 8;
          int i0 = (g_rateHistoryIdx[0] - 3 + 8) % 8;
          // Indices for previous 3 samples
          int j2 = (g_rateHistoryIdx[0] - 4 + 8) % 8;
          int j1 = (g_rateHistoryIdx[0] - 5 + 8) % 8;
          int j0 = (g_rateHistoryIdx[0] - 6 + 8) % 8;
          recentAvg = (g_rateHistory[0][i0] + g_rateHistory[0][i1] + g_rateHistory[0][i2]) / 3.0f;
          previousAvg = (g_rateHistory[0][j0] + g_rateHistory[0][j1] + g_rateHistory[0][j2]) / 3.0f;

          float avgChange = recentAvg - previousAvg;
          FSM_DBG_PRINT("SUB1: WET avg recent="); FSM_DBG_PRINT(recentAvg);
          FSM_DBG_PRINT(" prevAvg="); FSM_DBG_PRINT(previousAvg);
          FSM_DBG_PRINT(" change="); FSM_DBG_PRINT(avgChange);
          FSM_DBG_PRINT(" diff="); FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3");

          if (avgChange < AH_RATE_DECLINE_THRESHOLD) {
            g_consecutiveNegativeCount[0]++;
          } else {
            g_consecutiveNegativeCount[0] = 0;
          }

          // Robust peak detection with 4-tier adaptive thresholds
          uint32_t minPeakTimeMs;
          float peakRateThreshold;
          
          if (g_initialWetDiff[0] < AH_DIFF_BARELY_WET) {
            // Barely wet: quick peak detection to save power
            minPeakTimeMs = 60u * 1000u;  // 60s
            peakRateThreshold = 0.2f;     // Low threshold
          } else if (g_initialWetDiff[0] < AH_DIFF_MODERATE_WET) {
            // Moderate: normal thresholds
            minPeakTimeMs = AH_PEAK_NORMAL_MIN_TIME_MS;           // 120s
            peakRateThreshold = AH_RATE_NORMAL_PEAK_THRESHOLD;    // 0.35
          } else if (g_initialWetDiff[0] < AH_DIFF_VERY_WET) {
            // Very wet: stricter thresholds
            minPeakTimeMs = 180u * 1000u;    // 180s
            peakRateThreshold = 0.50f;       // 0.50
          } else {
            // Soaked: most stringent thresholds
            minPeakTimeMs = AH_PEAK_WET_MIN_TIME_MS;        // 240s
            peakRateThreshold = AH_RATE_WET_PEAK_THRESHOLD; // 0.60
          }

          bool decliningEnough = (g_consecutiveNegativeCount[0] >= MIN_CONSECUTIVE_NEGATIVE) && (avgChange < -0.05f);
          bool rateIsLow = (recentAvg < peakRateThreshold);
          bool hasSpentEnoughTime = (wetElapsed >= minPeakTimeMs);
          
          if (decliningEnough && rateIsLow && hasSpentEnoughTime) {
            FSM_DBG_PRINT("SUB1: WET peak (rate=");
            FSM_DBG_PRINT(recentAvg, 2);
            FSM_DBG_PRINT("<");
            FSM_DBG_PRINT(peakRateThreshold, 2);
            FSM_DBG_PRINT(", time=");
            FSM_DBG_PRINT(wetElapsed/1000);
            FSM_DBG_PRINT("s>=");
            FSM_DBG_PRINT(minPeakTimeMs/1000);
            FSM_DBG_PRINT("s) -> ");
            FSM_DBG_PRINT(g_peakBufferMs[0]/1000);
            FSM_DBG_PRINTLN("s buffer");
            g_peakDetected[0] = true;
            g_peakDetectedMs[0] = now;
            g_consecutiveNegativeCount[0] = 0;
            g_lastValidAHDiff[0] = currentAHDiff;
            // Heater continues during peak detection for extra heating time
            FSM_DBG_PRINTLN("SUB1: WET peak detected, entering post-peak buffer (heater continues)");
          }
        }
      }
      g_prevAHRate[0] = currentRate;
      g_ahRateSampleCount[0]++;
    }
  });
  fsmSub2.setRun(SubState::S_WET, []() {
    uint32_t now = millis();
    uint32_t wetElapsed = (g_subWetStartMs[1] != 0) ? (now - g_subWetStartMs[1]) : 0;
    
    // ==================== WARMUP PHASE (30-50s at 60% motor + heater) ====================
    // During this phase, heater warms shoe WITHOUT trend-gated control interference
    // Motor runs at fixed 60% to avoid friction heating
    if (g_heaterWarmupStartMs[1] == 0 && !g_heaterWarmupDone[1]) {
      // First time entering WET - start warmup
      g_heaterWarmupStartMs[1] = now;
      heaterRun(1, true);  // Heater ON
      // NOTE: Do NOT call motorStart() yet - only set duty directly at 60%
      // motorStart() initializes PID which immediately overrides our 60% setpoint
      motorSetDutyPercent(1, 60);  // Manual 60% duty without PID
      FSM_DBG_PRINTLN("SUB2: Warmup phase START (30-50s at 60% motor + heater)");
    }
    
    // Monitor heater warmup phase progression
    if (g_heaterWarmupStartMs[1] != 0 && !g_heaterWarmupDone[1]) {
      uint32_t warmupElapsed = now - g_heaterWarmupStartMs[1];
      
      // Determine warmup duration (cold shoe gets extra time)
      uint32_t targetWarmupMs = HEATER_WARMUP_MIN_MS;  // 30s default
      float shoeTemp = g_dhtTemp[2];  // Sensor 2 = shoe 1
      if (!isnan(shoeTemp) && shoeTemp < 25.0f) {
        targetWarmupMs = HEATER_WARMUP_EXTENDED_MS;  // 50s for cold shoes
        if (warmupElapsed == 0 || (warmupElapsed >= HEATER_WARMUP_MIN_MS - 1000 && warmupElapsed < HEATER_WARMUP_MIN_MS)) {
          FSM_DBG_PRINT("SUB2: Cold shoe detected (");
          FSM_DBG_PRINT(shoeTemp, 1);
          FSM_DBG_PRINTLN("C) -> extended warmup 50s");
        }
      }

      // End warmup immediately once threshold temp is reached
      if (!isnan(shoeTemp) && shoeTemp >= HEATER_WET_TEMP_THRESHOLD_C) {
        FSM_DBG_PRINT("SUB2: Warmup threshold reached (");
        FSM_DBG_PRINT(shoeTemp, 1);
        FSM_DBG_PRINTLN("C) -> heater OFF, switch to trend-gated control");
        g_heaterWarmupDone[1] = true;
        heaterRun(1, false);
        motorStart(1);
        g_ahRateSampleCount[1] = 0;
        g_consecutiveNegativeCount[1] = 0;
        g_lastAHRateSampleMs[1] = now;
        g_prevAHRate[1] = 0.0f;
      } else if (warmupElapsed < targetWarmupMs) {
        // Still in warmup phase - keep heater ON, motor at 60%
        return;  // Skip the rest of WET logic until warmup is done
      } else {
        // Time-based completion fallback
        FSM_DBG_PRINTLN("SUB2: Warmup time complete -> transition to trend-gated WET (PID motor control)");
        g_heaterWarmupDone[1] = true;
        motorStart(1);
        g_ahRateSampleCount[1] = 0;
        g_consecutiveNegativeCount[1] = 0;
        g_lastAHRateSampleMs[1] = now;
        g_prevAHRate[1] = 0.0f;
      }
    }
    
    // ==================== NORMAL WET PHASE (after warmup) ====================
    // Heater is now controlled by trend-gating (maybeEarlyHeaterOff)
    // Motor duty will be managed by PID control (AH rate monitoring)
    
    if (g_heaterWarmupDone[1] && g_subWetStartMs[1] != 0) {
      // Apply trend-based heater control throughout the remainder of WET
      maybeEarlyHeaterOff(1, wetElapsed);
    }

    // Sample AH rate every 2 seconds and check for peak (declining rate)
    // But wait AH_ACCEL_WARMUP_MS (30s) after WET start before checking for decline
    if (g_heaterWarmupDone[1] && g_subWetStartMs[1] != 0) {
      // Early return if sampling interval hasn't elapsed (prevent tight loop)
      if ((uint32_t)(now - g_lastAHRateSampleMs[1]) < 2000) {
        return;
      }
      
      g_lastAHRateSampleMs[1] = now;
      float currentRate = g_dhtAHRate[1];  // Use actual AH rate-of-change from motor control
      float currentAHDiff = g_dhtAHDiff[1];  // Get current AH diff for safety checks
      
      // Validate rate before processing (reject NaN/Inf from sensor glitches)
      if (isnan(currentRate) || isinf(currentRate)) {
        return;  // Skip this sample, wait for next valid rate
      }
      
      // ==================== PEAK-DETECTED: IN POST-PEAK BUFFER PHASE ====================
      if (g_peakDetected[1]) {
        uint32_t bufferElapsed = (uint32_t)(now - g_peakDetectedMs[1]);
        
        // SAFETY CHECK: Verify AH diff hasn't dropped to unexpected low values (sensor noise/glitch)
        if (!isnan(currentAHDiff) && currentAHDiff > 0.1f) {
          g_lastValidAHDiff[1] = currentAHDiff;
          g_lastAHDiffCheckMs[1] = now;
        }
        
        // Still in post-peak buffer period
        if (bufferElapsed < g_peakBufferMs[1]) {
          // Continue monitoring evaporation during buffer
          uint32_t bufferRemaining = g_peakBufferMs[1] - bufferElapsed;
          FSM_DBG_PRINT("SUB2: WET in post-peak buffer (");
          FSM_DBG_PRINT(bufferRemaining);
          FSM_DBG_PRINT("ms remaining, diff=");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3)");
          return;
        }
        
        // Buffer period expired - check safety conditions before exiting to COOLING
        uint32_t minDurationRemaining = (wetElapsed >= g_wetMinDurationMs[1]) ? 0 : (g_wetMinDurationMs[1] - wetElapsed);
        
        // Dynamic buffer extension: if current AH rose above initial, extend buffer duration
        // This handles shoes that got wetter during WET phase
        uint32_t adaptiveBufferMs = g_peakBufferMs[1];
        if (currentAHDiff > g_initialWetDiff[1] + 0.5f) {
          // Shoe got significantly wetter during WET phase - extend buffer
          // Use 4-tier thresholds based on CURRENT moisture level
          if (currentAHDiff < 1.5f) {
            adaptiveBufferMs = WET_BUFFER_BARELY_WET_MS;  // 40s
          } else if (currentAHDiff < 3.5f) {
            adaptiveBufferMs = WET_BUFFER_MODERATE_MS;    // 75s (normal)
          } else if (currentAHDiff < 5.0f) {
            adaptiveBufferMs = WET_BUFFER_VERY_WET_MS;    // 100s
          } else {
            adaptiveBufferMs = WET_BUFFER_SOAKED_MS;      // 120s
          }
          
          // Re-check if buffer has actually expired with the new duration
          if (bufferElapsed < adaptiveBufferMs) {
            uint32_t adaptiveRemaining = adaptiveBufferMs - bufferElapsed;
            FSM_DBG_PRINT("SUB2: WET buffer extended (moisture rose: ");
            FSM_DBG_PRINT(g_initialWetDiff[1], 1);
            FSM_DBG_PRINT(" -> ");
            FSM_DBG_PRINT(currentAHDiff, 1);
            FSM_DBG_PRINT("g/m^3), remaining=");
            FSM_DBG_PRINT(adaptiveRemaining);
            FSM_DBG_PRINTLN("ms");
            return;
          }
        }
        
        // Temperature-based buffer hold: if shoe is still hot, extend buffer by 30s
        float tC1 = g_dhtTemp[2];
        if (!isnan(tC1) && tC1 >= WET_BUFFER_TEMP_HOT_C) {
          if (bufferElapsed < g_peakBufferMs[1] + WET_BUFFER_TEMP_EXTEND_MS) {
            uint32_t remain = (g_peakBufferMs[1] + WET_BUFFER_TEMP_EXTEND_MS) - bufferElapsed;
            FSM_DBG_PRINT("SUB2: WET buffer temp-hold (t=");
            FSM_DBG_PRINT(tC1, 1);
            FSM_DBG_PRINT("C), remaining=");
            FSM_DBG_PRINT(remain);
            FSM_DBG_PRINTLN("ms");
            return;
          }
        }
        
        // SAFETY: Require AH diff to still be reasonable (not accidental low reading)
        bool safeAHLevel = (currentAHDiff > AH_DIFF_SAFETY_MARGIN) || (g_lastValidAHDiff[1] > AH_DIFF_SAFETY_MARGIN + 0.2f);
        bool minDurationMet = (minDurationRemaining == 0);
        
        if (minDurationMet && safeAHLevel) {
          FSM_DBG_PRINT("SUB2: WET peak + buffer complete, diff=");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3 -> transition to COOLING");
          fsmSub2.handleEvent(Event::SubStart);
          g_ahRateSampleCount[1] = 0;
          g_consecutiveNegativeCount[1] = 0;
          g_peakDetected[1] = false;
          g_peakDetectedMs[1] = 0;
          return;
        } else if (minDurationMet && !safeAHLevel) {
          // AH diff dropped too low - might be sensor glitch, wait longer
          FSM_DBG_PRINT("SUB2: WET safety check - diff dropped to ");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3 (below safety margin), waiting for stabilization...");
          return;
        } else {
          // Minimum duration not yet reached
          FSM_DBG_PRINT("SUB2: WET waiting for minimum duration (");
          FSM_DBG_PRINT(minDurationRemaining);
          FSM_DBG_PRINT("ms remaining, diff=");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3)");
          return;
        }
      }
      
      // ==================== BEFORE PEAK: MONITOR EVAPORATION RATE ====================
      // Track minimum AH diff seen (to detect true evaporation bottom)
      if (wetElapsed >= AH_ACCEL_WARMUP_MS && currentAHDiff < g_minAHDiffSeen[1]) {
        g_minAHDiffSeen[1] = currentAHDiff;
        g_minAHDiffSeenMs[1] = now;
      }
      
      // Early peak detection: if AH has risen significantly above minimum, we passed the peak
      // Adaptive gate by initial wetness
      uint32_t minRiseTime1;
      float riseThreshold1;
      if (g_initialWetDiff[1] < AH_DIFF_BARELY_WET) { // barely wet
        minRiseTime1 = 60000u; riseThreshold1 = 0.6f;
      } else if (g_initialWetDiff[1] < AH_DIFF_MODERATE_WET) { // moderate
        minRiseTime1 = 90000u; riseThreshold1 = 0.6f;
      } else if (g_initialWetDiff[1] < AH_DIFF_VERY_WET) { // very wet
        minRiseTime1 = 120000u; riseThreshold1 = 0.8f;
      } else { // soaked
        minRiseTime1 = 150000u; riseThreshold1 = 1.0f;
      }
      if (wetElapsed >= minRiseTime1 && g_minAHDiffSeen[1] < g_initialWetDiff[1] - 0.5f) {
        // We've seen a significant drop (good evaporation happened)
        float riseFromMin = currentAHDiff - g_minAHDiffSeen[1];
        if (riseFromMin > riseThreshold1) {
          // AH has risen >riseThreshold g/m^3 from minimum - we missed the peak!
          FSM_DBG_PRINT("SUB2: WET RISE detection - min was ");
          FSM_DBG_PRINT(g_minAHDiffSeen[1], 2);
          FSM_DBG_PRINT(" now ");
          FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINT(" (rose +");
          FSM_DBG_PRINT(riseFromMin, 2);
          FSM_DBG_PRINTLN(") -> peak passed, starting buffer");
          g_peakDetected[1] = true;
          g_peakDetectedMs[1] = g_minAHDiffSeenMs[1];  // Use time when minimum was seen
          g_consecutiveNegativeCount[1] = 0;
          g_lastValidAHDiff[1] = currentAHDiff;
          heaterRun(1, false);
          return;
        }
      }
      
      // Peak detection using moving-average decline
      if (wetElapsed >= AH_ACCEL_WARMUP_MS) {
        // Update rate history buffer
        g_rateHistory[1][g_rateHistoryIdx[1]] = currentRate;
        g_rateHistoryIdx[1] = (g_rateHistoryIdx[1] + 1) % 8;
        if (g_rateHistoryCount[1] < 8) g_rateHistoryCount[1]++;

        if (g_rateHistoryCount[1] >= 6) {
          float recentAvg = 0.0f, previousAvg = 0.0f;
          int i2 = (g_rateHistoryIdx[1] - 1 + 8) % 8;
          int i1 = (g_rateHistoryIdx[1] - 2 + 8) % 8;
          int i0 = (g_rateHistoryIdx[1] - 3 + 8) % 8;
          int j2 = (g_rateHistoryIdx[1] - 4 + 8) % 8;
          int j1 = (g_rateHistoryIdx[1] - 5 + 8) % 8;
          int j0 = (g_rateHistoryIdx[1] - 6 + 8) % 8;
          recentAvg = (g_rateHistory[1][i0] + g_rateHistory[1][i1] + g_rateHistory[1][i2]) / 3.0f;
          previousAvg = (g_rateHistory[1][j0] + g_rateHistory[1][j1] + g_rateHistory[1][j2]) / 3.0f;

          float avgChange = recentAvg - previousAvg;
          FSM_DBG_PRINT("SUB2: WET avg recent="); FSM_DBG_PRINT(recentAvg);
          FSM_DBG_PRINT(" prevAvg="); FSM_DBG_PRINT(previousAvg);
          FSM_DBG_PRINT(" change="); FSM_DBG_PRINT(avgChange);
          FSM_DBG_PRINT(" diff="); FSM_DBG_PRINT(currentAHDiff, 2);
          FSM_DBG_PRINTLN("g/m^3");

          if (avgChange < AH_RATE_DECLINE_THRESHOLD) {
            g_consecutiveNegativeCount[1]++;
          } else {
            g_consecutiveNegativeCount[1] = 0;
          }

          // Robust peak detection with 4-tier adaptive thresholds
          uint32_t minPeakTimeMs;
          float peakRateThreshold;
          
          if (g_initialWetDiff[1] < AH_DIFF_BARELY_WET) {
            // Barely wet: quick peak detection to save power
            minPeakTimeMs = 60u * 1000u;  // 60s
            peakRateThreshold = 0.2f;     // Low threshold
          } else if (g_initialWetDiff[1] < AH_DIFF_MODERATE_WET) {
            // Moderate: normal thresholds
            minPeakTimeMs = AH_PEAK_NORMAL_MIN_TIME_MS;           // 120s
            peakRateThreshold = AH_RATE_NORMAL_PEAK_THRESHOLD;    // 0.35
          } else if (g_initialWetDiff[1] < AH_DIFF_VERY_WET) {
            // Very wet: stricter thresholds
            minPeakTimeMs = 180u * 1000u;    // 180s
            peakRateThreshold = 0.50f;       // 0.50
          } else {
            // Soaked: most stringent thresholds
            minPeakTimeMs = AH_PEAK_WET_MIN_TIME_MS;        // 240s
            peakRateThreshold = AH_RATE_WET_PEAK_THRESHOLD; // 0.60
          }

          bool decliningEnough = (g_consecutiveNegativeCount[1] >= MIN_CONSECUTIVE_NEGATIVE) && (avgChange < -0.05f);
          bool rateIsLow = (recentAvg < peakRateThreshold);
          bool hasSpentEnoughTime = (wetElapsed >= minPeakTimeMs);
          
          if (decliningEnough && rateIsLow && hasSpentEnoughTime) {
            FSM_DBG_PRINT("SUB2: WET peak (rate=");
            FSM_DBG_PRINT(recentAvg, 2);
            FSM_DBG_PRINT("<");
            FSM_DBG_PRINT(peakRateThreshold, 2);
            FSM_DBG_PRINT(", time=");
            FSM_DBG_PRINT(wetElapsed/1000);
            FSM_DBG_PRINT("s>=");
            FSM_DBG_PRINT(minPeakTimeMs/1000);
            FSM_DBG_PRINT("s) -> ");
            FSM_DBG_PRINT(g_peakBufferMs[1]/1000);
            FSM_DBG_PRINTLN("s buffer");
            g_peakDetected[1] = true;
            g_peakDetectedMs[1] = now;
            g_consecutiveNegativeCount[1] = 0;
            g_lastValidAHDiff[1] = currentAHDiff;
            // Heater continues during peak detection for extra heating time
            FSM_DBG_PRINTLN("SUB2: WET peak detected, entering post-peak buffer (heater continues)");
          }
        }
      }
        g_prevAHRate[1] = currentRate;
        g_ahRateSampleCount[1]++;
      }
  });

  // S_COOLING run: two-phase logic
  // Phase 1 (0-5s): motor at 80%, heater off
  // Phase 2 (5-10s): motor off, stabilization period
  // After 10s: perform dry-check and advance
  // OPTIMIZATION: If already dry, immediately advance to DRY state
  fsmSub1.setRun(SubState::S_COOLING, []() {
    if (g_subCoolingStartMs[0] == 0)
      return;

    // ================= RE-EVAP SHORT CYCLE =================
    if (g_inReEvap[0]) {
      // Heater ON (below cutoff), motor at fixed duty; looking for lenient rise-from-min or timeout
      float t = g_dhtTemp[1];
      if (!isnan(t) && t >= HEATER_WET_TEMP_THRESHOLD_C) {
        heaterRun(0, false);
      } else {
        heaterRun(0, true);
      }
      if (!g_motorStarted[0]) { motorStart(0); g_motorStarted[0] = true; }
      motorSetDutyPercent(0, RE_EVAP_MOTOR_DUTY);
      uint32_t now = millis();
      uint32_t elapsed = (uint32_t)(now - g_reEvapStartMs[0]);
      float d = g_dhtAHDiff[0];
      if (!isnan(d) && d < g_reEvapMinDiff[0]) { g_reEvapMinDiff[0] = d; g_reEvapMinDiffMs[0] = now; }
      // Adaptive lenient gates
      float init = g_initialWetDiff[0];
      uint32_t minTime; float riseThresh;
      if (init < AH_DIFF_BARELY_WET) { minTime = RE_EVAP_MIN_TIME_BARE_MOD; riseThresh = RE_EVAP_RISE_BARE_MOD; }
      else if (init < AH_DIFF_MODERATE_WET) { minTime = RE_EVAP_MIN_TIME_BARE_MOD; riseThresh = RE_EVAP_RISE_BARE_MOD; }
      else if (init < AH_DIFF_VERY_WET) { minTime = RE_EVAP_MIN_TIME_VERY; riseThresh = RE_EVAP_RISE_VERY; }
      else { minTime = RE_EVAP_MIN_TIME_SOAKED; riseThresh = RE_EVAP_RISE_SOAKED; }
      bool timeout = (elapsed >= RE_EVAP_MAX_MS);
      bool risePassed = (elapsed >= minTime) && (d - g_reEvapMinDiff[0] > riseThresh);
      if (timeout || risePassed) {
        FSM_DBG_PRINT("SUB1: RE-EVAP done (" ); FSM_DBG_PRINT(timeout ? "timeout" : "rise"); FSM_DBG_PRINTLN(") -> back to COOLING");
        heaterRun(0, false);
        g_inReEvap[0] = false;
        g_reEvapStartMs[0] = 0; g_reEvapMinDiff[0] = 999.0f; g_reEvapMinDiffMs[0] = 0;
        // Restart COOLING motor phase fresh (retry semantics)
        startCoolingPhase(0, true);
        return;
      }
      return; // stay in re-evap until exit conditions
    }
    
    // Check if shoe is already dry - guard with flag to prevent infinite loop
    if (!g_coolingEarlyExit[0]) {
      float earlyDiff = g_dhtAHDiff[0];
      if (earlyDiff <= AH_DRY_THRESHOLD) {
        FSM_DBG_PRINT("SUB1: COOLING early dry-check -> already dry (diff=");
        FSM_DBG_PRINT(earlyDiff);
        FSM_DBG_PRINTLN("), advancing immediately");
        g_subCoolingStartMs[0] = 0;
        g_subCoolingStabilizeStartMs[0] = 0;
        g_coolingLocked[0] = false;
        g_coolingEarlyExit[0] = true;
        motorStop(0);
        fsmSub1.handleEvent(Event::SubStart);
        return;
      }
    }
    
    uint32_t nowMs0 = millis();
    uint32_t motorElapsed = (uint32_t)(nowMs0 - g_subCoolingStartMs[0]);
    float tempC0 = g_dhtTemp[1];
    float ambC0 = g_dhtTemp[0];
    float targetC0 = (!isnan(ambC0) ? (ambC0 + COOLING_AMBIENT_DELTA_C) : COOLING_TEMP_RELEASE_C);
    
    // Phase 1: motor running with duty based on temperature delta (shoe vs ambient)
    // Goal: Use motor to cool when shoe is HOT; OFF when ambient is warmer (passive cooling/heating sufficient)
    // Heater is already OFF; motor circulation is the ONLY cooling mechanism
    if (!isnan(tempC0) && !isnan(ambC0)) {
      float delta = tempC0 - ambC0;  // Positive = shoe hotter, negative = ambient hotter
      int duty = 0;  // Default: motor OFF
      
      if (delta > 5.0f) {
        // Shoe much hotter than ambient: aggressive motor cooling needed
        duty = 90;  // Increased to speed cooling
      } else if (delta > 2.0f) {
        // Shoe moderately hotter: medium-high motor speed
        duty = 75;  // Increased from 60
      } else if (delta > 0.5f) {
        // Shoe slightly hotter: ensure meaningful airflow
        duty = 55;  // Increased from 40
      }
      // If delta <= 0.5: ambient is at least as warm as shoe, motor OFF (duty = 0)
      
      motorSetDutyPercent(0, duty);
    } else if (!isnan(tempC0) && tempC0 >= COOLING_TEMP_FAN_BOOST_ON) {
      // Fallback: high shoe temp detected, use moderate cooling (60%)
      motorSetDutyPercent(0, 60);
    } else {
      // Safety: if can't determine temperature reliably, use low motor (40%)
      motorSetDutyPercent(0, 40);
    }
    if (motorElapsed < g_coolingMotorDurationMs[0]) {
      return; // Still in motor-run phase
    }
    
    // If motor phase time elapsed but shoe is still hot, extend motor phase (bounded to prevent watchdog timeout)
    if (!isnan(tempC0) && tempC0 > targetC0) {
      if (motorElapsed < g_coolingMotorDurationMs[0] + COOLING_TEMP_EXTEND_MAX_MS) {
        static uint32_t lastHoldLog0 = 0;
        if ((uint32_t)(nowMs0 - lastHoldLog0) >= 10000u || lastHoldLog0 == 0) {
          lastHoldLog0 = nowMs0;
          FSM_DBG_PRINT("SUB1: COOLING hold - temp=");
          FSM_DBG_PRINT(tempC0, 1);
          FSM_DBG_PRINT("C, target=");
          FSM_DBG_PRINT(targetC0, 1);
          FSM_DBG_PRINTLN("C, extending motor run");
        }
        // Adjust motor duty based on current temp delta during extension
        float delta = tempC0 - ambC0;
        if (delta > 5.0f) {
          motorSetDutyPercent(0, 80);  // Still hot, use high duty
        } else if (delta > 2.0f) {
          motorSetDutyPercent(0, 60);  // Moderately hot
        } else {
          motorSetDutyPercent(0, 40);  // Just barely above target
        }
        return;
      }
      // Max extension reached: force stabilization to prevent infinite watchdog timeout
      FSM_DBG_PRINT("SUB1: COOLING -> max motor extension (");
      FSM_DBG_PRINT(motorElapsed);
      FSM_DBG_PRINTLN("ms) reached, forcing stabilization to prevent watchdog");
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
      // Sample AH diff every 15 seconds during stabilization
      static uint32_t lastSampleMs0 = 0;
      if (stabilizeElapsed - lastSampleMs0 >= 15000 || lastSampleMs0 == 0) {
        lastSampleMs0 = stabilizeElapsed;
        g_coolingDiffSamples[0][g_coolingDiffSampleIdx[0]] = g_dhtAHDiff[0];
        g_coolingDiffSampleIdx[0] = (g_coolingDiffSampleIdx[0] + 1) % 6;
        if (g_coolingDiffSampleCount[0] < 6) g_coolingDiffSampleCount[0]++;
      }
      return; // Still in stabilization phase
    }
    
    // Stabilization complete, perform dry-check with adaptive threshold
    float diff = g_dhtAHDiff[0];
    bool isDeclining = isAHDiffDeclining(0);
    float threshold = isDeclining ? AH_DRY_THRESHOLD_LENIENT : AH_DRY_THRESHOLD;
    // Use median of last 3 stabilization samples if available to reduce noise
    float evalDiff0 = diff;
    if (g_coolingDiffSampleCount[0] >= 3) {
      float a = g_coolingDiffSamples[0][(g_coolingDiffSampleIdx[0] - 1 + 6) % 6];
      float b = g_coolingDiffSamples[0][(g_coolingDiffSampleIdx[0] - 2 + 6) % 6];
      float c = g_coolingDiffSamples[0][(g_coolingDiffSampleIdx[0] - 3 + 6) % 6];
      // median of a,b,c
      float minab = (a < b) ? a : b;
      float maxab = (a > b) ? a : b;
      float med = (c < minab) ? minab : (c > maxab ? maxab : c);
      evalDiff0 = med;
    }
    bool stillWet = (evalDiff0 > threshold);
    // Temperature guard: don't declare dry if still warm (> 36.5C)
    float tC0_final = g_dhtTemp[1];
    float tC0_amb = g_dhtTemp[0];
    float tC0_target = (!isnan(tC0_amb) ? (tC0_amb + COOLING_AMBIENT_DELTA_C) : (COOLING_TEMP_RELEASE_C - 0.5f));
    if (!isnan(tC0_final) && tC0_final > tC0_target) {
      stillWet = true;
    }
    FSM_DBG_PRINT("SUB1: COOLING stabilization done -> dry-check, diff=");
    FSM_DBG_PRINT(evalDiff0);
    FSM_DBG_PRINT(isDeclining ? " (declining, lenient threshold=" : " (threshold=");
    FSM_DBG_PRINT(threshold);
    FSM_DBG_PRINTLN(")");
    g_subCoolingStartMs[0] = 0;
    g_subCoolingStabilizeStartMs[0] = 0;
    g_coolingLocked[0] = false;
    if (stillWet) {
      // Immediately perform short re-evap cycle (no cooling retries)
      FSM_DBG_PRINTLN("SUB1: COOLING -> invoking RE-EVAP short cycle");
      g_inReEvap[0] = true;
      g_reEvapStartMs[0] = millis();
      g_reEvapMinDiff[0] = g_dhtAHDiff[0];
      g_reEvapMinDiffMs[0] = g_reEvapStartMs[0];
      return;
    } else {
      FSM_DBG_PRINTLN("SUB1: COOLING dry-check -> dry, advancing to DRY");
      fsmSub1.handleEvent(Event::SubStart);
    }
  });

  fsmSub2.setRun(SubState::S_COOLING, []() {
    if (g_subCoolingStartMs[1] == 0)
      return;

    // ================= RE-EVAP SHORT CYCLE =================
    if (g_inReEvap[1]) {
      float t = g_dhtTemp[2];
      if (!isnan(t) && t >= HEATER_WET_TEMP_THRESHOLD_C) {
        heaterRun(1, false);
      } else {
        heaterRun(1, true);
      }
      if (!g_motorStarted[1]) { motorStart(1); g_motorStarted[1] = true; }
      motorSetDutyPercent(1, RE_EVAP_MOTOR_DUTY);
      uint32_t now = millis();
      uint32_t elapsed = (uint32_t)(now - g_reEvapStartMs[1]);
      float d = g_dhtAHDiff[1];
      if (!isnan(d) && d < g_reEvapMinDiff[1]) { g_reEvapMinDiff[1] = d; g_reEvapMinDiffMs[1] = now; }
      float init = g_initialWetDiff[1];
      uint32_t minTime; float riseThresh;
      if (init < AH_DIFF_BARELY_WET) { minTime = RE_EVAP_MIN_TIME_BARE_MOD; riseThresh = RE_EVAP_RISE_BARE_MOD; }
      else if (init < AH_DIFF_MODERATE_WET) { minTime = RE_EVAP_MIN_TIME_BARE_MOD; riseThresh = RE_EVAP_RISE_BARE_MOD; }
      else if (init < AH_DIFF_VERY_WET) { minTime = RE_EVAP_MIN_TIME_VERY; riseThresh = RE_EVAP_RISE_VERY; }
      else { minTime = RE_EVAP_MIN_TIME_SOAKED; riseThresh = RE_EVAP_RISE_SOAKED; }
      bool timeout = (elapsed >= RE_EVAP_MAX_MS);
      bool risePassed = (elapsed >= minTime) && (d - g_reEvapMinDiff[1] > riseThresh);
      if (timeout || risePassed) {
        FSM_DBG_PRINT("SUB2: RE-EVAP done (" ); FSM_DBG_PRINT(timeout ? "timeout" : "rise"); FSM_DBG_PRINTLN(") -> back to COOLING");
        heaterRun(1, false);
        g_inReEvap[1] = false;
        g_reEvapStartMs[1] = 0; g_reEvapMinDiff[1] = 999.0f; g_reEvapMinDiffMs[1] = 0;
        startCoolingPhase(1, true);
        return;
      }
      return;
    }
    
    // Check if shoe is already dry - guard with flag to prevent infinite loop
    if (!g_coolingEarlyExit[1]) {
      float earlyDiff = g_dhtAHDiff[1];
      if (earlyDiff <= AH_DRY_THRESHOLD) {
        FSM_DBG_PRINT("SUB2: COOLING early dry-check -> already dry (diff=");
        FSM_DBG_PRINT(earlyDiff);
        FSM_DBG_PRINTLN("), advancing immediately");
        g_subCoolingStartMs[1] = 0;
        g_subCoolingStabilizeStartMs[1] = 0;
        g_coolingLocked[1] = false;
        g_coolingEarlyExit[1] = true;
        motorStop(1);
        fsmSub2.handleEvent(Event::SubStart);
        return;
      }
    }
    
    uint32_t nowMs1 = millis();
    uint32_t motorElapsed = (uint32_t)(nowMs1 - g_subCoolingStartMs[1]);
    float tempC1 = g_dhtTemp[2];
    float ambC1 = g_dhtTemp[0];
    float targetC1 = (!isnan(ambC1) ? (ambC1 + COOLING_AMBIENT_DELTA_C) : COOLING_TEMP_RELEASE_C);
    
    // Phase 1: motor running with duty based on temperature delta (shoe vs ambient)
    // Goal: Use motor to cool when shoe is HOT; OFF when ambient is warmer (passive cooling/heating sufficient)
    // Heater is already OFF; motor circulation is the ONLY cooling mechanism
    if (!isnan(tempC1) && !isnan(ambC1)) {
      float delta = tempC1 - ambC1;  // Positive = shoe hotter, negative = ambient hotter
      int duty = 0;  // Default: motor OFF
      
      if (delta > 5.0f) {
        // Shoe much hotter than ambient: aggressive motor cooling needed
        duty = 90;  // Increased to speed cooling
      } else if (delta > 2.0f) {
        // Shoe moderately hotter: medium-high motor speed
        duty = 75;  // Increased from 60
      } else if (delta > 0.5f) {
        // Shoe slightly hotter: ensure meaningful airflow
        duty = 55;  // Increased from 40
      }
      // If delta <= 0.5: ambient is at least as warm as shoe, motor OFF (duty = 0)
      
      motorSetDutyPercent(1, duty);
    } else if (!isnan(tempC1) && tempC1 >= COOLING_TEMP_FAN_BOOST_ON) {
      // Fallback: high shoe temp detected, use moderate cooling (60%)
      motorSetDutyPercent(1, 60);
    } else {
      // Safety: if can't determine temperature reliably, use low motor (40%)
      motorSetDutyPercent(1, 40);
    }
    if (motorElapsed < g_coolingMotorDurationMs[1]) {
      return; // Still in motor-run phase
    }
    
    // If motor phase time elapsed but shoe is still hot, extend motor phase (bounded to prevent watchdog timeout)
    if (!isnan(tempC1) && tempC1 > targetC1) {
      if (motorElapsed < g_coolingMotorDurationMs[1] + COOLING_TEMP_EXTEND_MAX_MS) {
        static uint32_t lastHoldLog1 = 0;
        if ((uint32_t)(nowMs1 - lastHoldLog1) >= 10000u || lastHoldLog1 == 0) {
          lastHoldLog1 = nowMs1;
          FSM_DBG_PRINT("SUB2: COOLING hold - temp=");
          FSM_DBG_PRINT(tempC1, 1);
          FSM_DBG_PRINT("C, target=");
          FSM_DBG_PRINT(targetC1, 1);
          FSM_DBG_PRINTLN("C, extending motor run");
        }
        // Adjust motor duty based on current temp delta during extension
        float delta = tempC1 - ambC1;
        if (delta > 5.0f) {
          motorSetDutyPercent(1, 80);  // Still hot, use high duty
        } else if (delta > 2.0f) {
          motorSetDutyPercent(1, 60);  // Moderately hot
        } else {
          motorSetDutyPercent(1, 40);  // Just barely above target
        }
        return;
      }
      // Max extension reached: force stabilization to prevent infinite watchdog timeout
      FSM_DBG_PRINT("SUB2: COOLING -> max motor extension (");
      FSM_DBG_PRINT(motorElapsed);
      FSM_DBG_PRINTLN("ms) reached, forcing stabilization to prevent watchdog");
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
      // Sample AH diff every 15 seconds during stabilization
      static uint32_t lastSampleMs1 = 0;
      if (stabilizeElapsed - lastSampleMs1 >= 15000 || lastSampleMs1 == 0) {
        lastSampleMs1 = stabilizeElapsed;
        g_coolingDiffSamples[1][g_coolingDiffSampleIdx[1]] = g_dhtAHDiff[1];
        g_coolingDiffSampleIdx[1] = (g_coolingDiffSampleIdx[1] + 1) % 6;
        if (g_coolingDiffSampleCount[1] < 6) g_coolingDiffSampleCount[1]++;
      }
      return; // Still in stabilization phase
    }
    
    // Stabilization complete, perform dry-check with adaptive threshold
    float diff = g_dhtAHDiff[1];
    bool isDeclining = isAHDiffDeclining(1);
    float threshold = isDeclining ? AH_DRY_THRESHOLD_LENIENT : AH_DRY_THRESHOLD;
    // Use median of last 3 stabilization samples if available to reduce noise
    float evalDiff1 = diff;
    if (g_coolingDiffSampleCount[1] >= 3) {
      float a = g_coolingDiffSamples[1][(g_coolingDiffSampleIdx[1] - 1 + 6) % 6];
      float b = g_coolingDiffSamples[1][(g_coolingDiffSampleIdx[1] - 2 + 6) % 6];
      float c = g_coolingDiffSamples[1][(g_coolingDiffSampleIdx[1] - 3 + 6) % 6];
      float minab = (a < b) ? a : b;
      float maxab = (a > b) ? a : b;
      float med = (c < minab) ? minab : (c > maxab ? maxab : c);
      evalDiff1 = med;
    }
    bool stillWet = (evalDiff1 > threshold);
    // Temperature guard: don't declare dry if still warm (> 36.5C)
    float tC1_final = g_dhtTemp[2];
    float tC1_amb = g_dhtTemp[0];
    float tC1_target = (!isnan(tC1_amb) ? (tC1_amb + COOLING_AMBIENT_DELTA_C) : (COOLING_TEMP_RELEASE_C - 0.5f));
    if (!isnan(tC1_final) && tC1_final > tC1_target) {
      stillWet = true;
    }
    FSM_DBG_PRINT("SUB2: COOLING stabilization done -> dry-check, diff=");
    FSM_DBG_PRINT(evalDiff1);
    FSM_DBG_PRINT(isDeclining ? " (declining, lenient threshold=" : " (threshold=");
    FSM_DBG_PRINT(threshold);
    FSM_DBG_PRINTLN(")");
    g_subCoolingStartMs[1] = 0;
    g_subCoolingStabilizeStartMs[1] = 0;
    g_coolingLocked[1] = false;
    if (stillWet) {
      // Immediately perform short re-evap cycle (no cooling retries)
      FSM_DBG_PRINTLN("SUB2: COOLING -> invoking RE-EVAP short cycle");
      g_inReEvap[1] = true;
      g_reEvapStartMs[1] = millis();
      g_reEvapMinDiff[1] = g_dhtAHDiff[1];
      g_reEvapMinDiffMs[1] = g_reEvapStartMs[1];
      return;
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
    // Update cached battery voltage
    g_lastBatteryVoltage = readBatteryVoltage();
    if (!isBatteryOk()) {
      FSM_DBG_PRINT("GLOBAL: Battery voltage low: ");
      if (Serial) Serial.printf("%.2f V\n", g_lastBatteryVoltage);
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

    // Play entry-only SOLE/CARE splash on Idle entry (after reset)
    triggerSplashEntryOnly();
    
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

// Getter functions for timing tracking used in UI progress calculations
uint32_t getSubWetStartMs(int shoeIdx) {
  if (shoeIdx < 0 || shoeIdx >= 2) return 0;
  return g_subWetStartMs[shoeIdx];
}

uint32_t getSubCoolingStartMs(int shoeIdx) {
  if (shoeIdx < 0 || shoeIdx >= 2) return 0;
  return g_subCoolingStartMs[shoeIdx];
}

uint32_t getCoolingMotorDurationMs(int shoeIdx) {
  if (shoeIdx < 0 || shoeIdx >= 2) return 0;
  return g_coolingMotorDurationMs[shoeIdx];
}