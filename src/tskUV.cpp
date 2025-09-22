// tskUV.cpp - UV task implementation
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
static constexpr bool RELAY_ACTIVE_LOW = HW_RELAY_ACTIVE_LOW;
static constexpr uint32_t UV_DEFAULT_MS = HW_UV_DEFAULT_MS;

enum class UVCmd : uint8_t { Start = 0, Stop = 1 };
struct UVMsg {
  UVCmd cmd;
  uint8_t idx;
  uint32_t durationMs;
};

static QueueHandle_t g_uvQ = nullptr;
static volatile bool g_uvStarted[2] = {false, false};
static volatile bool g_uvPaused[2] = {false, false};
// when running and not paused, g_uvEndMs = absolute end time; when paused, g_uvRemainingMs stores
// remaining time
static volatile uint32_t g_uvEndMs[2] = {0, 0};
static volatile uint32_t g_uvRemainingMs[2] = {0, 0};
static portMUX_TYPE g_uvMux = portMUX_INITIALIZER_UNLOCKED;

static inline void setRelay(uint8_t idx, bool on) {
  const int pin = (idx == 0) ? UV_PIN_0 : UV_PIN_1;
  digitalWrite(pin, (RELAY_ACTIVE_LOW) ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

static void uvTask(void * /*pv*/) {
  pinMode(UV_PIN_0, OUTPUT);
  pinMode(UV_PIN_1, OUTPUT);
  setRelay(0, false);
  setRelay(1, false);

  UVMsg msg;
  for (;;) {
    if (xQueueReceive(g_uvQ, &msg, pdMS_TO_TICKS(200)) == pdTRUE) {
      uint8_t i = (msg.idx < 2) ? msg.idx : 0;
      if (msg.cmd == UVCmd::Start) {
        taskENTER_CRITICAL(&g_uvMux);
        g_uvPaused[i] = false;
        g_uvStarted[i] = true;
        g_uvRemainingMs[i] = (msg.durationMs ? msg.durationMs : UV_DEFAULT_MS);
        g_uvEndMs[i] = millis() + g_uvRemainingMs[i];
        taskEXIT_CRITICAL(&g_uvMux);
        setRelay(i, true);
      } else {
        // Stop: clear all state and notify
        taskENTER_CRITICAL(&g_uvMux);
        g_uvStarted[i] = false;
        g_uvPaused[i] = false;
        g_uvEndMs[i] = 0;
        g_uvRemainingMs[i] = 0;
        taskEXIT_CRITICAL(&g_uvMux);
        setRelay(i, false);
        fsmExternalPost(i == 0 ? Event::UVTimer0 : Event::UVTimer1);
      }
    }

    uint32_t now = millis();
    for (int i = 0; i < 2; ++i) {
      bool expired = false;
      taskENTER_CRITICAL(&g_uvMux);
      if (g_uvStarted[i] && !g_uvPaused[i] && g_uvEndMs[i] != 0 &&
          (int32_t)(now - g_uvEndMs[i]) >= 0) {
        g_uvStarted[i] = false;
        g_uvEndMs[i] = 0;
        g_uvRemainingMs[i] = 0;
        expired = true;
      }
      taskEXIT_CRITICAL(&g_uvMux);
      if (expired) {
        setRelay(i, false);
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
  if (!g_uvQ)
    return false;
  bool ok = false;
  taskENTER_CRITICAL(&g_uvMux);
  if (g_uvStarted[idx] && !g_uvPaused[idx]) {
    uint32_t now = millis();
    if (g_uvEndMs[idx] > now) {
      g_uvRemainingMs[idx] = (uint32_t)(g_uvEndMs[idx] - now);
      g_uvPaused[idx] = true;
      ok = true;
    } else {
      // Timer already expired or about to expire: treat as expired instead of pausing
      g_uvStarted[idx] = false;
      g_uvPaused[idx] = false;
      g_uvEndMs[idx] = 0;
      g_uvRemainingMs[idx] = 0;
      // We'll post the UVTimer event so FSM can handle expiry deterministically
      taskEXIT_CRITICAL(&g_uvMux);
      setRelay(idx, false);
      fsmExternalPost(idx == 0 ? Event::UVTimer0 : Event::UVTimer1);
      return false;
    }
  }
  taskEXIT_CRITICAL(&g_uvMux);
  if (ok)
    setRelay(idx, false);
  return ok;
}

bool uvResume(uint8_t idx) {
  if (!g_uvQ)
    return false;
  bool ok = false;
  taskENTER_CRITICAL(&g_uvMux);
  if (g_uvStarted[idx] && g_uvPaused[idx]) {
    if (g_uvRemainingMs[idx] > 0) {
      g_uvEndMs[idx] = millis() + g_uvRemainingMs[idx];
      g_uvPaused[idx] = false;
      ok = true;
    } else {
      // no remaining time -> treat as expired
      g_uvStarted[idx] = false;
      g_uvPaused[idx] = false;
      g_uvEndMs[idx] = 0;
      g_uvRemainingMs[idx] = 0;
      taskEXIT_CRITICAL(&g_uvMux);
      setRelay(idx, false);
      fsmExternalPost(idx == 0 ? Event::UVTimer0 : Event::UVTimer1);
      return false;
    }
  }
  taskEXIT_CRITICAL(&g_uvMux);
  if (ok)
    setRelay(idx, true);
  return ok;
}

bool uvIsPaused(uint8_t idx) {
  bool p;
  taskENTER_CRITICAL(&g_uvMux);
  p = g_uvPaused[idx];
  taskEXIT_CRITICAL(&g_uvMux);
  return p;
}

uint32_t uvRemainingMs(uint8_t idx) {
  uint32_t rem = 0;
  taskENTER_CRITICAL(&g_uvMux);
  if (g_uvStarted[idx]) {
    if (g_uvPaused[idx])
      rem = g_uvRemainingMs[idx];
    else {
      uint32_t now = millis();
      if (g_uvEndMs[idx] > now)
        rem = (uint32_t)(g_uvEndMs[idx] - now);
      else
        rem = 0;
    }
  }
  taskEXIT_CRITICAL(&g_uvMux);
  return rem;
}