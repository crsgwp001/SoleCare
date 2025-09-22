#include "tskUI.h"
#include <ui.h>
#include <Arduino.h>
#include "global.h"
#include <events.h>
#include "dev_debug.h"
#include "tskMotor.h"
#include "tskUV.h"

// Remapped OLED1 to pins 25 (SDA) and 26 (SCL) to avoid conflicts with actuator pins
DisplayUnit oled1(23, 19, 0); // SDA, SCL, Wire0
DisplayUnit oled2(22, 21, 1); // SDA, SCL, Wire1
#include "tskFSM.h"

void vMainOledTask(void* /*pv*/) {
  if (!oled1.begin()) { DEV_DBG_PRINTLN("vMainOledTask: init failed, stopping task"); vTaskDelete(NULL); }
  DEV_DBG_PRINTLN("OLED1 (Humidity) task started");

  while (true) {
    // FSM state strings
    auto gs = getGlobalState();
    auto ss1 = getSub1State();
    auto ss2 = getSub2State();
    const char* gss = "?";
    switch (gs) {
      case GlobalState::Idle: gss = "IDLE"; break;
      case GlobalState::Detecting: gss = "DETECT"; break;
      case GlobalState::Checking: gss = "CHECK"; break;
      case GlobalState::Running: gss = "RUN"; break;
      case GlobalState::Done: gss = "DONE"; break;
      case GlobalState::Error: gss = "ERR"; break;
      case GlobalState::Debug: gss = "DBG"; break;
      default: break;
    }
    const char* sss1 = (ss1==SubState::S_WET)?"WET":(ss1==SubState::S_COOLING)?"COOL":(ss1==SubState::S_DRY)?"DRY":(ss1==SubState::S_DONE)?"DONE":"IDLE";
    const char* sss2 = (ss2==SubState::S_WET)?"WET":(ss2==SubState::S_COOLING)?"COOL":(ss2==SubState::S_DRY)?"DRY":(ss2==SubState::S_DONE)?"DONE":"IDLE";

  // get actuator and UV states
  bool uv0 = uvIsStarted(0), uv1 = uvIsStarted(1);
  bool uv0p = uvIsPaused(0), uv1p = uvIsPaused(1);
    bool m0 = motorIsOn(0), m1 = motorIsOn(1);
    bool ht0 = heaterIsOn(0), ht1 = heaterIsOn(1);
    uint32_t m0ms = motorActiveMs(0), m1ms = motorActiveMs(1);
  uint32_t uv0rem = uvRemainingMs(0), uv1rem = uvRemainingMs(1);

    // Compose a compact indicators view for OLED1
    char msg[128];
    // show UV remaining seconds and paused marker
    char uv0s[16], uv1s[16];
    if (uv0) {
      if (uv0p) snprintf(uv0s, sizeof(uv0s), "PAUS %2lus", (unsigned long)(uv0rem/1000u));
      else snprintf(uv0s, sizeof(uv0s), "%2lus", (unsigned long)(uv0rem/1000u));
    } else snprintf(uv0s, sizeof(uv0s), "--");
    if (uv1) {
      if (uv1p) snprintf(uv1s, sizeof(uv1s), "PAUS %2lus", (unsigned long)(uv1rem/1000u));
      else snprintf(uv1s, sizeof(uv1s), "%2lus", (unsigned long)(uv1rem/1000u));
    } else snprintf(uv1s, sizeof(uv1s), "--");

    snprintf(msg, sizeof(msg), "G:%-4s S1:%-4s S2:%-4s\nM0:%s H0:%s t:%2lus  M1:%s H1:%s t:%2lus\nUV0:%6s UV1:%6s",
             gss, sss1, sss2,
             m0?"ON":"off", ht0?"ON":"off", (unsigned long)(m0ms/1000u),
             m1?"ON":"off", ht1?"ON":"off", (unsigned long)(m1ms/1000u),
             uv0s, uv1s);
  oled1.showMessage(String(msg), uv0, uv1, uv0p, uv1p);
    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}

void vAuxOledTask(void* /*pv*/) {
  if (!oled2.begin()) { DEV_DBG_PRINTLN("vAuxOledTask: init failed, stopping task"); vTaskDelete(NULL); }
  DEV_DBG_PRINTLN("OLED2 (Temp) task started");
  while (true) {
  // Show absolute-humidity (EMA), diffs, and shoe status on OLED2
  char ah0s[16], ah1s[16], ah2s[16], d0s[16], d1s[16];
  const char* shoe0 = g_dhtIsWet[0] ? "WET" : "DRY";
  const char* shoe1 = g_dhtIsWet[1] ? "WET" : "DRY";
    if (isnan(g_dhtAH_ema[0])) snprintf(ah0s, sizeof(ah0s), "--"); else snprintf(ah0s, sizeof(ah0s), "%.2f", g_dhtAH_ema[0]);
    if (isnan(g_dhtAH_ema[1])) snprintf(ah1s, sizeof(ah1s), "--"); else snprintf(ah1s, sizeof(ah1s), "%.2f", g_dhtAH_ema[1]);
    if (isnan(g_dhtAH_ema[2])) snprintf(ah2s, sizeof(ah2s), "--"); else snprintf(ah2s, sizeof(ah2s), "%.2f", g_dhtAH_ema[2]);
    if (isnan(g_dhtAHDiff[0])) snprintf(d0s, sizeof(d0s), "--"); else snprintf(d0s, sizeof(d0s), "%+.2f", g_dhtAHDiff[0]);
    if (isnan(g_dhtAHDiff[1])) snprintf(d1s, sizeof(d1s), "--"); else snprintf(d1s, sizeof(d1s), "%+.2f", g_dhtAHDiff[1]);

    char msg[128];
  // Line1: AH for sensors 0/1 and shoe status for shoe0
  // Line2: AH for sensor2, diffs and shoe1 status
  snprintf(msg, sizeof(msg), "AH0:%6s AH1:%6s S0:%-4s\nAH2:%6s Δ1:%6s S1:%-4s",
       ah0s, ah1s, shoe0,
       ah2s, d0s, shoe1);
    oled2.showMessage(String(msg));
    vTaskDelay(pdMS_TO_TICKS(1200));
  }
}

void createOledTasks() {
  xTaskCreatePinnedToCore(vMainOledTask, "OLED1", 4096, NULL, 1, NULL, 0);
  // Run the auxiliary OLED task on core 0 as well to avoid contention with
  // sensor/DHT work which runs on core 1.
  xTaskCreatePinnedToCore(vAuxOledTask,  "OLED2", 4096, NULL, 1, NULL, 0);
}
