// tskUV.cpp - UV task implementation (now using PWM for MOSFET control)
#include "tskUV.h"
#include "tskFSM.h"
#include "config.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

// hardware configuration (moved to include/config.h)
static constexpr int UV_PIN_0 = HW_UV_PIN_0;
static constexpr int UV_PIN_1 = HW_UV_PIN_1;
static constexpr uint32_t UV_DEFAULT_MS = HW_UV_DEFAULT_MS;

// UV now uses PWM for MOSFET control
static const int UV_PWM_FREQ = 5000; // Hz
static const int UV_PWM_RES = 9;     // bits
static const int UV_PWM_MAX = (1 << UV_PWM_RES) - 1;
static const int UV_PWM_TARGET = UV_PWM_MAX; // 100% duty for UV
static const uint8_t UV_PWM_CH[2] = {2, 3}; // PWM channels 2 and 3 for UV

// Delay UV turn-on and ramp to avoid instant full power
static const uint32_t UV_START_DELAY_MS = 5000;  // 5s grace before UV enables
static const uint32_t UV_RAMP_MS = 2000;         // Ramp duration to full duty

enum class UVCmd : uint8_t { Start = 0, Stop = 1 };
struct UVMsg {
  UVCmd cmd;
  uint8_t idx;
  uint32_t durationMs;
};

static QueueHandle_t g_uvQ = nullptr;
static volatile bool g_uvStarted[2] = {false, false};
// g_uvEndMs = absolute end time while running
static volatile uint32_t g_uvEndMs[2] = {0, 0};
static volatile uint32_t g_uvRemainingMs[2] = {0, 0};
static volatile uint32_t g_uvStartAtMs[2] = {0, 0};      // absolute time to start after delay
static volatile uint32_t g_uvRampStartMs[2] = {0, 0};    // ramp start time (after delay)
static volatile int g_uvCurrentDuty[2] = {0, 0};
static portMUX_TYPE g_uvMux = portMUX_INITIALIZER_UNLOCKED;

static inline void setUVPWM(uint8_t idx, int duty) {
  if (idx > 1)
    return;
  ledcWrite(UV_PWM_CH[idx], duty);
}

static void uvTask(void * /*pv*/) {
  // Configure LEDC PWM for UV MOSFETs
  ledcSetup(UV_PWM_CH[0], UV_PWM_FREQ, UV_PWM_RES);
  ledcAttachPin(UV_PIN_0, UV_PWM_CH[0]);
  ledcSetup(UV_PWM_CH[1], UV_PWM_FREQ, UV_PWM_RES);
  ledcAttachPin(UV_PIN_1, UV_PWM_CH[1]);
  
  // Start with UV off
  setUVPWM(0, 0);
  setUVPWM(1, 0);

  UVMsg msg;
  for (;;) {
    if (xQueueReceive(g_uvQ, &msg, pdMS_TO_TICKS(200)) == pdTRUE) {
      uint8_t i = (msg.idx < 2) ? msg.idx : 0;
      if (msg.cmd == UVCmd::Start) {
        taskENTER_CRITICAL(&g_uvMux);
        g_uvStarted[i] = true;
        g_uvRemainingMs[i] = (msg.durationMs ? msg.durationMs : UV_DEFAULT_MS);
        uint32_t now = millis();
        g_uvStartAtMs[i] = now + UV_START_DELAY_MS;
        g_uvRampStartMs[i] = g_uvStartAtMs[i];
        g_uvEndMs[i] = g_uvStartAtMs[i] + g_uvRemainingMs[i];
        taskEXIT_CRITICAL(&g_uvMux);
        g_uvCurrentDuty[i] = 0;
        setUVPWM(i, 0); // hold off until delay elapses, then ramp
      } else {
        // Stop: clear all state and notify
        taskENTER_CRITICAL(&g_uvMux);
        g_uvStarted[i] = false;
        g_uvEndMs[i] = 0;
        g_uvRemainingMs[i] = 0;
        g_uvStartAtMs[i] = 0;
        g_uvRampStartMs[i] = 0;
        g_uvCurrentDuty[i] = 0;
        taskEXIT_CRITICAL(&g_uvMux);
        setUVPWM(i, 0); // Turn off UV
        fsmExternalPost(i == 0 ? Event::UVTimer0 : Event::UVTimer1);
      }
    }

    uint32_t now = millis();
    // Apply delayed start and ramp-up to full duty
    for (int i = 0; i < 2; ++i) {
      int duty = 0;
      bool active;
      uint32_t startAt;
      uint32_t rampStart;
      taskENTER_CRITICAL(&g_uvMux);
      active = g_uvStarted[i];
      startAt = g_uvStartAtMs[i];
      rampStart = g_uvRampStartMs[i];
      taskEXIT_CRITICAL(&g_uvMux);

      if (active) {
        if ((int32_t)(now - startAt) >= 0) {
          uint32_t rampElapsed = (now > rampStart) ? (now - rampStart) : 0;
          if (rampElapsed >= UV_RAMP_MS) {
            duty = UV_PWM_TARGET;
          } else {
            duty = (int)((uint64_t)UV_PWM_TARGET * rampElapsed / UV_RAMP_MS);
          }
        } else {
          duty = 0; // Still waiting for delay to elapse
        }
      }

      if (duty != g_uvCurrentDuty[i]) {
        g_uvCurrentDuty[i] = duty;
        setUVPWM(i, duty);
      }
    }

    for (int i = 0; i < 2; ++i) {
      bool expired = false;
      taskENTER_CRITICAL(&g_uvMux);
      if (g_uvStarted[i] && g_uvEndMs[i] != 0 &&
          (int32_t)(now - g_uvEndMs[i]) >= 0) {
        g_uvStarted[i] = false;
        g_uvEndMs[i] = 0;
        g_uvRemainingMs[i] = 0;
        g_uvStartAtMs[i] = 0;
        g_uvRampStartMs[i] = 0;
        g_uvCurrentDuty[i] = 0;
        expired = true;
      }
      taskEXIT_CRITICAL(&g_uvMux);
      if (expired) {
        setUVPWM(i, 0); // Turn off UV
        fsmExternalPost(i == 0 ? Event::UVTimer0 : Event::UVTimer1);
      }
    }
  }
}

bool uvInit() {
  if (g_uvQ)
    return true;
  g_uvQ = xQueueCreate(4, sizeof(UVMsg));
  if (!g_uvQ)
    return false;
  return xTaskCreatePinnedToCore(uvTask, "UVTask", 2048, nullptr, 2, nullptr, 1) == pdPASS;
}

bool uvStart(uint8_t idx, uint32_t durationMs) {
  if (!g_uvQ)
    return false;
  UVMsg m{UVCmd::Start, idx, durationMs};
  return xQueueSend(g_uvQ, &m, 0) == pdTRUE;
}
bool uvStop(uint8_t idx) {
  if (!g_uvQ)
    return false;
  UVMsg m{UVCmd::Stop, idx, 0};
  return xQueueSend(g_uvQ, &m, 0) == pdTRUE;
}

bool uvTimerFinished(uint8_t idx) {
  bool started;
  taskENTER_CRITICAL(&g_uvMux);
  started = g_uvStarted[idx];
  taskEXIT_CRITICAL(&g_uvMux);
  return !started;
}
bool uvIsStarted(uint8_t idx) {
  bool started;
  taskENTER_CRITICAL(&g_uvMux);
  started = g_uvStarted[idx];
  taskEXIT_CRITICAL(&g_uvMux);
  return started;
}
bool uvPause(uint8_t idx) {
  // Pause not supported; UV runs uninterrupted
  (void)idx;
  return false;
}

bool uvResume(uint8_t idx) {
  // Resume not supported; UV runs uninterrupted
  (void)idx;
  return false;
}

bool uvIsPaused(uint8_t idx) {
  (void)idx;
  return false;
}

uint32_t uvRemainingMs(uint8_t idx) {
  uint32_t rem = 0;
  taskENTER_CRITICAL(&g_uvMux);
  if (g_uvStarted[idx]) {
    uint32_t now = millis();
    if (g_uvEndMs[idx] > now)
      rem = (uint32_t)(g_uvEndMs[idx] - now);
    else
      rem = 0;
  }
  taskEXIT_CRITICAL(&g_uvMux);
  return rem;
}