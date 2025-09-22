// tskMotor.cpp - Motor and heater control task
#include "tskMotor.h"
#include "dev_debug.h"
#include "global.h"
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

static inline void setActuator(int pin, bool on) {
  digitalWrite(pin, (HW_ACTUATOR_ACTIVE_LOW) ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
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
  setActuator(HW_HEATER_PIN_0, false);
  setActuator(HW_HEATER_PIN_1, false);

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
        setActuator((i == 0) ? HW_MOTOR_PIN_0 : HW_MOTOR_PIN_1, true);
        setActuator((i == 0) ? HW_HEATER_PIN_0 : HW_HEATER_PIN_1, true);
        DEV_DBG_PRINT("MOTOR: started for idx=");
        DEV_DBG_PRINTLN(i);
        break;
      case MotorCmd::Stop:
        // disable actuators and mark inactive
        g_motorOn[i] = false;
        g_heaterOn[i] = false;
        setActuator((i == 0) ? HW_MOTOR_PIN_0 : HW_MOTOR_PIN_1, false);
        setActuator((i == 0) ? HW_HEATER_PIN_0 : HW_HEATER_PIN_1, false);
        g_motorActive[i] = false;
        g_motorStartMs[i] = 0;
        DEV_DBG_PRINT("MOTOR: stopped for idx=");
        DEV_DBG_PRINTLN(i);
        break;
      case MotorCmd::Run:
        g_motorOn[i] = msg.on;
        setActuator((i == 0) ? HW_MOTOR_PIN_0 : HW_MOTOR_PIN_1, msg.on);
        DEV_DBG_PRINT("MOTOR: set motor on=");
        DEV_DBG_PRINTLN(msg.on);
        break;
      case MotorCmd::Heater:
        g_heaterOn[i] = msg.on;
        setActuator((i == 0) ? HW_HEATER_PIN_0 : HW_HEATER_PIN_1, msg.on);
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
        setActuator((i == 0) ? HW_HEATER_PIN_0 : HW_HEATER_PIN_1, false);
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
        setActuator((i == 0) ? HW_MOTOR_PIN_0 : HW_MOTOR_PIN_1, false);
        setActuator((i == 0) ? HW_HEATER_PIN_0 : HW_HEATER_PIN_1, false);
        g_motorActive[i] = false;
        g_motorStartMs[i] = 0;
        // Optionally notify FSM about forced stop (use Reset or Error if desired). For now just
        // log.
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
