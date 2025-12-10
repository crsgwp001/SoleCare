// tskMotor.cpp - Motor and heater control task
// Motor uses PWM (MOSFET), Heater uses relay control
#include "tskMotor.h"
#include "global.h"
#include "dev_debug.h"
#include "tskFSM.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

enum class MotorCmd : uint8_t { Start = 0, Stop = 1, Run = 2, Heater = 3 };
struct MotorMsg {
  MotorCmd cmd;
  uint8_t idx;
  bool on;
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
static const int MOTOR_PWM_TARGET = MOTOR_PWM_MAX * 4 / 5; // 80%
static const int MOTOR_PWM_STEP = 16; // duty step per loop (rough ramp)
static const uint8_t MOTOR_PWM_CH[2] = {0, 1};

static volatile int g_motorDuty[2] = {0, 0};
static volatile int g_motorTargetDuty[2] = {0, 0};

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
        // enable motor + heater for this shoe while active
        g_motorOn[i] = true;
        g_heaterOn[i] = true;
        // ramp to 50% target
        g_motorTargetDuty[i] = MOTOR_PWM_TARGET;
        // ensure PWM channel updated immediately
        setMotorPWM(i, g_motorDuty[i]);
        setHeaterRelay(i, true); // Heater uses relay control
        DEV_DBG_PRINT("MOTOR: started for idx=");
        DEV_DBG_PRINTLN(i);
        break;
      case MotorCmd::Stop:
        // disable actuators and mark inactive
        g_motorOn[i] = false;
        g_heaterOn[i] = false;
        // ramp down to 0 and detach PWM channel
        g_motorTargetDuty[i] = 0;
        setHeaterRelay(i, false); // Heater uses relay control
        g_motorActive[i] = false;
        g_motorStartMs[i] = 0;
        DEV_DBG_PRINT("MOTOR: stopped for idx=");
        DEV_DBG_PRINTLN(i);
        break;
      case MotorCmd::Run:
        g_motorOn[i] = msg.on;
        g_motorTargetDuty[i] = msg.on ? MOTOR_PWM_TARGET : 0;
        // ensure PWM updated next loop
        DEV_DBG_PRINT("MOTOR: set motor on=");
        DEV_DBG_PRINTLN(msg.on);
        break;
      case MotorCmd::Heater:
        g_heaterOn[i] = msg.on;
        setHeaterRelay(i, msg.on); // Heater uses relay control
        DEV_DBG_PRINT("MOTOR: set heater on=");
        DEV_DBG_PRINTLN(msg.on);
        break;
      }
    }

    // If a motor is active, check its sensor diff condition
    for (int i = 0; i < 2; ++i) {
      if (!g_motorActive[i])
        continue;
      // g_dhtAHDiff holds differences sensor1-0 (idx0) and sensor2-0 (idx1)
      float diff = g_dhtAHDiff[i];
      // If the diff has dropped below DRY threshold (i.e., we are now dry), advance the sub
      if (diff < AH_DRY_THRESHOLD) {
        DEV_DBG_PRINT("MOTOR: threshold reached for idx=");
        DEV_DBG_PRINTLN(i);
        // stop actuators for that shoe
        setActuator((i == 0) ? HW_MOTOR_PIN_0 : HW_MOTOR_PIN_1, false);
        setHeaterRelay(i, false); // Heater uses relay control
        g_motorActive[i] = false;
        // notify FSM to advance the sub: SubStart is the normal internal event
        // used to advance a sub-FSM from S_WET -> S_COOLING (or the next state).
        fsmExternalPost(Event::SubStart);
      }
      // Enforce motor safety timeout
      if (g_motorActive[i] && g_motorStartMs[i] != 0 &&
          (uint32_t)(millis() - g_motorStartMs[i]) >= MOTOR_SAFETY_MS) {
        DEV_DBG_PRINT("MOTOR: safety timeout reached - forcing stop for idx=");
        DEV_DBG_PRINTLN(i);
        // stop actuators and mark inactive
        g_motorOn[i] = false;
        g_heaterOn[i] = false;
        g_motorTargetDuty[i] = 0;
        setHeaterRelay(i, false); // Heater uses relay control
        g_motorActive[i] = false;
        g_motorStartMs[i] = 0;
        // Optionally notify FSM about forced stop (use Reset or Error if desired). For now just
        // log.
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
  return xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, nullptr, 2, nullptr, 1) == pdPASS;
}
bool motorStart(uint8_t idx) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Start, idx, false};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}
bool motorStop(uint8_t idx) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Stop, idx, false};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}

bool motorRun(uint8_t idx, bool on) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Run, idx, on};
  return xQueueSend(g_motorQ, &m, 0) == pdTRUE;
}
bool heaterRun(uint8_t idx, bool on) {
  if (!g_motorQ)
    return false;
  MotorMsg m{MotorCmd::Heater, idx, on};
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
