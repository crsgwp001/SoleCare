// tskMotor.cpp - Motor and heater control task
// Motor uses PWM (MOSFET), Heater uses relay control
#include "tskMotor.h"
#include "global.h"
#include "dev_debug.h"
#include "tskFSM.h"
#include "PIDcontrol.h"
#include "pidLog.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

enum class MotorCmd : uint8_t { Start = 0, Stop = 1, Heater = 2, SetDutyPct = 3 };
struct MotorMsg {
  MotorCmd cmd;
  uint8_t idx;
  bool on;
  int value; // duty percent when SetDutyPct
};

static QueueHandle_t g_motorQ = nullptr;
static volatile bool g_motorActive[2] = {false, false};
static volatile bool g_motorOn[2] = {false, false};
static volatile bool g_heaterOn[2] = {false, false};
static volatile uint32_t g_motorStartMs[2] = {0, 0};

// For motor pins we use LEDC PWM (ESP32) to drive a MOSFET gate. 
// Heaters now use relay control (simple on/off).
static const int MOTOR_PWM_FREQ = 5000; // Hz
static const int MOTOR_PWM_RES = 9;    // bits
static const int MOTOR_PWM_MAX = (1 << MOTOR_PWM_RES) - 1;
static const int MOTOR_PWM_TARGET = MOTOR_PWM_MAX;
static const int MOTOR_PWM_STEP = 16;
static const uint8_t MOTOR_PWM_CH[2] = {0, 1};

static volatile int g_motorDuty[2] = {0, 0};
static volatile int g_motorTargetDuty[2] = {0, 0};

// ==================== PID MOTOR CONTROL ====================
// Non-static to allow external linkage (used by tskFSM.cpp)
PIDcontrol g_motorPID[2] = {
  PIDcontrol(PID_KP, PID_KI, PID_KD, PID_SAMPLE_MS),
  PIDcontrol(PID_KP, PID_KI, PID_KD, PID_SAMPLE_MS)
};

bool g_pidInitialized[2] = {false, false};  // Track PID init per shoe

static float g_lastAH[2] = {0.0f, 0.0f};           // For AH rate calculation
static unsigned long g_lastAHTime[2] = {0, 0};     // Timestamp of last AH sample
static float g_lastValidRate[2] = {0.0f, 0.0f};    // Store last calculated rate for consistency
static bool g_rateInitialized[2] = {false, false}; // Track if we have first valid rate

static inline void setActuator(int pin, bool on) {
  digitalWrite(pin, (HW_ACTUATOR_ACTIVE_LOW) ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

// Heater now uses relay control
static inline void setHeaterRelay(uint8_t idx, bool on) {
  const int pin = (idx == 0) ? HW_HEATER_PIN_0 : HW_HEATER_PIN_1;
  digitalWrite(pin, (HW_RELAY_ACTIVE_LOW) ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

static inline void setMotorPWM(uint8_t idx, int duty) {
  if (idx > 1)
    return;
  ledcWrite(MOTOR_PWM_CH[idx], duty);
  g_motorDuty[idx] = duty;
}

// Calculate AH rate-of-change in g/m³/min
// Returns 0.0 during first second (warming up), then stable rates after
static float calculateAHRate(uint8_t idx) {
  float currentAH = g_dhtAH_ema[idx + 1];  // idx+1 because sensor 0 is ambient
  if (isnan(currentAH)) {
    // If sensor EMA is invalid, hold last valid rate
    return g_lastValidRate[idx];
  }
  unsigned long now = millis();
  
  // First call - initialize timestamp
  if (g_lastAHTime[idx] == 0) {
    g_lastAH[idx] = currentAH;
    g_lastAHTime[idx] = now;
    g_rateInitialized[idx] = false;  // Mark as not ready yet
    return 0.0f;
  }
  
  unsigned long dt = now - g_lastAHTime[idx];
  if (dt >= 1000) {
    // Enough time has passed - calculate and store new rate
    float ahDelta = currentAH - g_lastAH[idx];
    float ratePerMin = (ahDelta / (dt / 1000.0f)) * 60.0f;
    
    // Guard against NaN or Inf from computation errors
    if (isnan(ratePerMin) || isinf(ratePerMin)) {
      ratePerMin = g_lastValidRate[idx];  // Hold last valid rate if computation fails
    }
    
    // Clamp rate to physically reasonable bounds (±120 g/m³/min = 2 g/m³/s)
    // Extreme values indicate sensor noise and should be filtered
    if (ratePerMin > 120.0f) ratePerMin = 120.0f;
    if (ratePerMin < -120.0f) ratePerMin = -120.0f;
    
    g_lastAH[idx] = currentAH;
    g_lastAHTime[idx] = now;
    g_lastValidRate[idx] = ratePerMin;
    g_dhtAHRate[idx] = ratePerMin;  // Update global for FSM access
    g_rateInitialized[idx] = true;  // Mark as ready
    return ratePerMin;  // Return fresh calculation
  }
  
  // Not enough time yet
  if (!g_rateInitialized[idx]) {
    return 0.0f;  // Still warming up, no valid rate yet
  }
  
  // Return last valid rate (stable between updates)
  return g_lastValidRate[idx];
}

static void motorTask(void * /*pv*/) {
  // configure pins
  pinMode(HW_MOTOR_PIN_0, OUTPUT);
  pinMode(HW_MOTOR_PIN_1, OUTPUT);
  pinMode(HW_HEATER_PIN_0, OUTPUT);
  pinMode(HW_HEATER_PIN_1, OUTPUT);
  // ensure off
  setActuator(HW_MOTOR_PIN_0, false);
  setActuator(HW_MOTOR_PIN_1, false);
  setHeaterRelay(0, false); // Heater now uses relay control
  setHeaterRelay(1, false);

  // Configure LEDC PWM for motors
  ledcSetup(MOTOR_PWM_CH[0], MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(HW_MOTOR_PIN_0, MOTOR_PWM_CH[0]);
  ledcSetup(MOTOR_PWM_CH[1], MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttachPin(HW_MOTOR_PIN_1, MOTOR_PWM_CH[1]);

  MotorMsg msg;
  for (;;) {
    // Check for commands with a short timeout so we also poll sensors
    if (xQueueReceive(g_motorQ, &msg, pdMS_TO_TICKS(200)) == pdTRUE) {
      uint8_t i = (msg.idx < 2) ? msg.idx : 0;
      switch (msg.cmd) {
      case MotorCmd::Start:
        g_motorActive[i] = true;
        g_motorStartMs[i] = millis();
        g_motorOn[i] = true;
        g_heaterOn[i] = true;
        g_motorTargetDuty[i] = MOTOR_PWM_TARGET;
        setMotorPWM(i, g_motorDuty[i]);
        setHeaterRelay(i, true);
        DEV_DBG_PRINT("MOTOR: started for idx=");
        DEV_DBG_PRINTLN(i);
        break;
      case MotorCmd::Stop:
        g_motorOn[i] = false;
        g_heaterOn[i] = false;
        g_motorTargetDuty[i] = 0;
        setHeaterRelay(i, false);
        g_motorActive[i] = false;
        g_motorStartMs[i] = 0;
        DEV_DBG_PRINT("MOTOR: stopped for idx=");
        DEV_DBG_PRINTLN(i);
        break;
      case MotorCmd::Heater:
        g_heaterOn[i] = msg.on;
        setHeaterRelay(i, msg.on);
        DEV_DBG_PRINT("MOTOR: set heater on=");
        DEV_DBG_PRINTLN(msg.on);
        break;
      case MotorCmd::SetDutyPct: {
        int pct = msg.value;
        if (pct < 0)
          pct = 0;
        if (pct > 100)
          pct = 100;
        int tgt = (MOTOR_PWM_MAX * pct) / 100;
        g_motorTargetDuty[i] = tgt;
        // Only log significant duty changes (>5% change)
        if (g_motorDuty[i] == 0 || abs(tgt - g_motorDuty[i]) > (MOTOR_PWM_MAX * 5 / 100)) {
          DEV_DBG_PRINT("MOTOR: set duty %=");
          DEV_DBG_PRINTLN(pct);
        }
        break;
      }
      }
    }

    // ==================== AH RATE CALCULATION (continuous for logging) ====================
    // Storage for PID outputs and rates (updated continuously)
    static float ahRates[2] = {0.0f, 0.0f};
    static double pidOutputs[2] = {0.5, 0.5};  // Default to midpoint
    
    // Always calculate AH rates for logging (even when motor inactive)
    for (int i = 0; i < 2; ++i) {
      ahRates[i] = calculateAHRate(i);
    }
    
    // ==================== PID MOTOR CONTROL ====================
    for (int i = 0; i < 2; ++i) {
      if (!g_motorActive[i])
        continue;
      
      // Check if shoe is in COOLING state - if so, skip PID (use fixed duty set by FSM)
      SubState currentState = (i == 0) ? getSub1State() : getSub2State();
      if (currentState == SubState::S_COOLING) {
        // During COOLING, FSM controls motor duty directly (80% then OFF)
        // Store current duty for logging, but don't run PID
        pidOutputs[i] = g_motorTargetDuty[i] / (float)MOTOR_PWM_MAX;
        continue;
      }
        
      unsigned long wetElapsed = millis() - g_motorStartMs[i];
      
      if (wetElapsed < PID_CONTROL_START_MS) {
        // Phase 1: Warmup/initial phase - fixed duty percent
        motorSetDutyPercent(i, PID_FIXED_DUTY_PERCENT);
        pidOutputs[i] = PID_FIXED_DUTY_PERCENT / 100.0;  // Store for logging
      } else {
        // Phase 2: PID control based on AH rate-of-change
        
        // Initialize PID on first run in this phase
        if (!g_pidInitialized[i]) {
          g_motorPID[i].setMode(PIDcontrol::AUTOMATIC);
          g_pidInitialized[i] = true;
          DEV_DBG_PRINT("PID: activated for shoe ");
          DEV_DBG_PRINTLN(i);
        }
        
        // Use already-calculated AH rate from above
        // Compute PID output (already limited to PID_OUT_MIN-PID_OUT_MAX: 0.5-1.0)
        pidOutputs[i] = g_motorPID[i].compute(ahRates[i]);
        
        // Convert to duty percent (pidOutput is 0.5-1.0, so multiply by 100 for 50-100%)
        int dutyPercent = (int)round(pidOutputs[i] * 100.0);
        
        motorSetDutyPercent(i, dutyPercent);
      }
    }
    
    // ==================== CONSOLIDATED PID LOGGING (every 1 second, matches rate update interval) ====================
    {
      static unsigned long lastLogMs = 0;
      unsigned long now = millis();
      if (now - lastLogMs >= 1000) {  // 1-second interval (matches AH rate sampling)
        // Get current shoe states
        SubState sub0State = getSub1State();
        SubState sub1State = getSub2State();
        
        // Log comprehensive data: AH values, diffs, states, rates, PID outputs
        pidLogData(
          g_dhtAH_ema[0], g_dhtAH_ema[1], g_dhtAH_ema[2],       // AH0, AH1, AH2
          g_dhtAHDiff_ema[0], sub0State, ahRates[0], pidOutputs[0] * 100.0,  // Shoe 0
          g_dhtAHDiff_ema[1], sub1State, ahRates[1], pidOutputs[1] * 100.0   // Shoe 1
        );
        lastLogMs = now;
      }
    }

    // Check sensor conditions and timeouts
    for (int i = 0; i < 2; ++i) {
      if (!g_motorActive[i])
        continue;
      
      // Dry threshold: when sensor reads dry, advance WET->COOLING
      float diff = g_dhtAHDiff[i];
      if (diff < AH_DRY_THRESHOLD) {
        DEV_DBG_PRINT("MOTOR: dry threshold reached for idx=");
        DEV_DBG_PRINTLN(i);
        // Don't stop motor/heater here - let COOLING state handle it
        // Just post event to advance to COOLING
        fsmExternalPost(Event::SubStart);
        g_motorActive[i] = false; // Clear flag to avoid repeated posts
      }
      
      // Safety timeout: force advance WET->COOLING after MOTOR_SAFETY_MS
      if (g_motorActive[i] && g_motorStartMs[i] != 0 &&
          (uint32_t)(millis() - g_motorStartMs[i]) >= MOTOR_SAFETY_MS) {
        DEV_DBG_PRINT("MOTOR: safety timeout for idx=");
        DEV_DBG_PRINTLN(i);
        g_motorOn[i] = false;
        g_heaterOn[i] = false;
        g_motorTargetDuty[i] = 0;
        setHeaterRelay(i, false);
        g_motorActive[i] = false;
        g_motorStartMs[i] = 0;
        fsmExternalPost(Event::SubStart);
      }
    }

    // PWM ramping: move current duty towards target duty
    for (int i = 0; i < 2; ++i) {
      int cur = g_motorDuty[i];
      int tgt = g_motorTargetDuty[i];
      if (cur < tgt) {
        cur += MOTOR_PWM_STEP;
        if (cur > tgt)
          cur = tgt;
        setMotorPWM(i, cur);
      } else if (cur > tgt) {
        cur -= MOTOR_PWM_STEP;
        if (cur < tgt)
          cur = tgt;
        setMotorPWM(i, cur);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

bool motorInit() {
  if (g_motorQ)
    return true;
  g_motorQ = xQueueCreate(8, sizeof(MotorMsg));
  if (!g_motorQ)
    return false;
  
  // Initialize PID controllers for motor control
  for (int i = 0; i < 2; i++) {
    g_motorPID[i].setOutputLimits(PID_OUT_MIN, PID_OUT_MAX);
    g_motorPID[i].setSampleTime(PID_SAMPLE_MS);
    g_motorPID[i].setSetpoint(TARGET_AH_RATE);
    g_motorPID[i].setMode(PIDcontrol::MANUAL);  // Start in MANUAL
  }
  
  pidLogInit();  // Initialize PID logging
  
  return xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, nullptr, 2, nullptr, 1) == pdPASS;
}
bool motorStart(uint8_t idx) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Start, idx, false, 0};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}
bool motorStop(uint8_t idx) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Stop, idx, false, 0};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}


bool heaterRun(uint8_t idx, bool on) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Heater, idx, on, 0};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}

bool motorSetDutyPercent(uint8_t idx, int percent) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::SetDutyPct, idx, false, percent};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}

uint32_t motorActiveMs(uint8_t idx) {
  if (idx >= 2)
    return 0;
  if (!g_motorActive[idx])
    return 0;
  return (uint32_t)(millis() - g_motorStartMs[idx]);
}

bool motorIsActive(uint8_t idx) {
  return (idx < 2) ? g_motorActive[idx] : false;
}

bool motorIsOn(uint8_t idx) {
  return (idx < 2) ? g_motorOn[idx] : false;
}
bool heaterIsOn(uint8_t idx) {
  return (idx < 2) ? g_heaterOn[idx] : false;
}
