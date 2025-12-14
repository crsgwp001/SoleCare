#include "tskUI.h"
#include "global.h"
#include "dev_debug.h"
#include "tskMotor.h"
#include "tskUV.h"
#include "tskFSM.h"
#include <ui.h>
#include <Arduino.h>
#include <cmath>

//==============================================================================
// GLOBAL STATE & DISPLAY INSTANCES
//==============================================================================

DisplayUnit leftScreen(23, 21, 0);   // SDA, SCL, Wire0
DisplayUnit rightScreen(22, 19, 1);  // SDA, SCL, Wire1

uint32_t g_cycleStartMs = 0;

static GlobalState g_lastGlobalState = GlobalState::Idle;
static uint32_t g_stateStartMs = 0;
static volatile bool g_rightReady = false;

// State abbreviation lookup tables
static const char* GLOBAL_STATE_ABBR[] = {
  "IDL", "DET", "CHK", "RUN", "DON", "BAT", "ERR", "DBG"
};

static const char* SUB_STATE_ABBR[] = {
  "IDL", "WAI", "WET", "COL", "DRY", "DON"
};

//==============================================================================
// HELPER FUNCTIONS - Progress Calculation & Formatting
//==============================================================================

static void drawProgressBar(int percent, char* buf, size_t buflen) {
  int filled = (percent * 6 + 50) / 100;
  if (filled < 0) filled = 0;
  if (filled > 6) filled = 6;
  snprintf(buf, buflen, "[");
  for (int i = 0; i < filled; i++) strcat(buf, "#");
  for (int i = filled; i < 6; i++) strcat(buf, "-");
  strcat(buf, "]");
}

static const char* getGlobalStateAbbr(GlobalState gs) {
  int idx = (int)gs;
  if (idx < 0 || idx >= 8) return "?";
  return GLOBAL_STATE_ABBR[idx];
}

static const char* getSubStateAbbr(SubState ss) {
  int idx = (int)ss;
  if (idx < 0 || idx >= 6) return "?";
  return SUB_STATE_ABBR[idx];
}

// Weighted progress calculation: WET path = full cycle, DRY-only = UV progress
static int getShoeProgress(int shoeIdx, uint32_t nowMs) {
  SubState state = (shoeIdx == 0) ? getSub1State() : getSub2State();
  
  if (state == SubState::S_DONE) return 100;
  
  uint32_t wetStartMs = getSubWetStartMs(shoeIdx);
  bool wentThroughWet = (wetStartMs > 0);
  
  if (!wentThroughWet) {
    // DRY-only: progress based on UV countdown (account for 5s delay)
    if (state == SubState::S_DRY && uvIsStarted(0)) {
      uint32_t uvRemaining = uvRemainingMs(0);
      if (uvRemaining >= (HW_UV_DEFAULT_MS - 100)) return 0;
      int progress = (int)(100.0 * (HW_UV_DEFAULT_MS - uvRemaining) / HW_UV_DEFAULT_MS);
      return (progress > 100) ? 100 : progress;
    }
    return 0;
  }
  
  // WET path: weighted through WET + COOLING + UV phases
  const float WET_WEIGHT = 360.0;
  float coolingWeight = getCoolingMotorDurationMs(shoeIdx) / 1000.0;
  if (coolingWeight < 1.0) coolingWeight = 150.0;
  const float UV_WEIGHT = 10.0;
  const float TOTAL = WET_WEIGHT + coolingWeight + UV_WEIGHT;
  
  float completed = 0.0;
  if (state == SubState::S_WET) {
    completed = fminf((float)((nowMs - wetStartMs) / 1000), WET_WEIGHT);
  } else if ((int)state >= (int)SubState::S_COOLING) {
    completed = WET_WEIGHT;
    if (state == SubState::S_COOLING) {
      uint32_t coolStartMs = getSubCoolingStartMs(shoeIdx);
      if (coolStartMs > 0) {
        completed += fminf((float)((nowMs - coolStartMs) / 1000), coolingWeight);
      }
    } else if ((int)state >= (int)SubState::S_DRY) {
      completed += coolingWeight;
      if (state == SubState::S_DRY && uvIsStarted(0)) {
        uint32_t uvRemaining = uvRemainingMs(0);
        if (uvRemaining < (HW_UV_DEFAULT_MS - 100)) {
          completed += fminf((float)((HW_UV_DEFAULT_MS - uvRemaining) / 1000.0), UV_WEIGHT);
        }
      }
    }
  }
  
  int progress = (int)(100.0 * completed / TOTAL);
  return (progress > 100) ? 100 : progress;
}

//==============================================================================
// SPLASH ANIMATION - Used on startup and reset
//==============================================================================

static void playSplashAnimation() {
  // Animate both screens simultaneously
  bool rightAtStart = g_rightReady;  // latch readiness to avoid mid-animation races
  leftScreen.showSplash("SOLE", 100, 1000, 48, true);
  if (rightAtStart) {
    rightScreen.showSplash("CARE", 100, 1000, 5, true);
  }
  
  vTaskDelay(pdMS_TO_TICKS(2000));
  
  // Exit animation: letters disappear from left, SOLE shifts right, CARE stays fixed
  const char* sole_variants[] = {"SOLE", "OLE", "LE", "E", ""};
  const int sole_x_pos[] = {48, 66, 84, 102, 48};
  
  const char* care_variants[] = {"CARE", "CAR", "CA", "C", ""};
  const int care_x_pos[] = {5, 5, 5, 5, 5};
  
  for (int i = 0; i < 5; i++) {
    leftScreen.directShow(sole_variants[i], sole_x_pos[i]);
    vTaskDelay(pdMS_TO_TICKS(200));
    if (rightAtStart) {
      rightScreen.directShow(care_variants[i], care_x_pos[i]);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
  
  leftScreen.directClear();
  rightScreen.directClear();
  vTaskDelay(pdMS_TO_TICKS(500));
}

void triggerSplashAnimation() {
  playSplashAnimation();
}

// Entry-only splash (no exit animation) for reset button
void triggerSplashEntryOnly() {
  // Faster typing and shorter hold than boot
  leftScreen.showSplash("SOLE", 60, 300, 48, true);
  if (g_rightReady) {
    rightScreen.showSplash("CARE", 60, 300, 5, true);
  }
  vTaskDelay(pdMS_TO_TICKS(300));
}

//==============================================================================
// LEFT SCREEN TASK - Progress Tracking
//==============================================================================

void vLeftScreenTask(void * /*pv*/) {
  if (!leftScreen.begin()) {
    DEV_DBG_PRINTLN("vLeftScreenTask: init failed");
    vTaskDelete(NULL);
  }
  DEV_DBG_PRINTLN("Left Screen task started");
  
  playSplashAnimation();
  vTaskDelay(pdMS_TO_TICKS(500));

  while (true) {
    uint32_t now = millis();
    
    GlobalState gs = getGlobalState();
    
    // Track state changes for elapsed time timer
    if (gs != g_lastGlobalState) {
      g_lastGlobalState = gs;
      g_stateStartMs = now;
      // Refresh battery reading when entering Idle so UI shows latest value
      if (gs == GlobalState::Idle) {
        g_lastBatteryVoltage = readBatteryVoltage();
      }
    }
    
    uint32_t stateElapsedSec = (now - g_stateStartMs) / 1000;
    uint32_t mins = stateElapsedSec / 60;
    uint32_t secs = stateElapsedSec % 60;
    
    // Get progress and format bars
    int progress1 = getShoeProgress(0, now);
    int progress2 = getShoeProgress(1, now);
    char pb1[16], pb2[16];
    drawProgressBar(progress1, pb1, sizeof(pb1));
    drawProgressBar(progress2, pb2, sizeof(pb2));
    
    // Battery: refresh while in Idle so reset button shows latest
    float batteryV = g_lastBatteryVoltage;
    if (gs == GlobalState::Idle) {
      batteryV = readBatteryVoltage();
      g_lastBatteryVoltage = batteryV;
    }

    // Format display message
    char msg[128];
    snprintf(msg, sizeof(msg),
             "%s - %02lu:%02lu\n"
             "---\n"
             "S1[%s] %s %d%%\n"
             "S2[%s] %s %d%%\n"
             "---\n"
             "UV: %lus\n"
             "Bat: %.1fV",
             getGlobalStateAbbr(gs), mins, secs,
             (g_dhtIsWet[0] ? "WET" : "DRY"), pb1, progress1,
             (g_dhtIsWet[1] ? "WET" : "DRY"), pb2, progress2,
             uvRemainingMs(0) / 1000,
             batteryV);
      if (gs == GlobalState::Idle) {
        batteryV = readBatteryVoltage();
        g_lastBatteryVoltage = batteryV;
      }
    
    leftScreen.showMessage(String(msg));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//==============================================================================
// RIGHT SCREEN TASK - Sensor Data Display
//==============================================================================

void vRightScreenTask(void * /*pv*/) {
  if (!rightScreen.begin()) {
    DEV_DBG_PRINTLN("vRightScreenTask: init failed");
    vTaskDelete(NULL);
  }
  DEV_DBG_PRINTLN("Right Screen task started");
  g_rightReady = true;
  
  // Play CARE splash at startup to match SOLE
  rightScreen.showSplash("CARE", 100, 1000, 5, true);
  vTaskDelay(pdMS_TO_TICKS(2000));
  const char* care_variants[] = {"CARE", "CAR", "CA", "C", ""};
  const int care_x_pos[] = {5, 5, 5, 5, 5};
  for (int i = 0; i < 5; i++) {
    rightScreen.directShow(care_variants[i], care_x_pos[i]);
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  rightScreen.directClear();
  vTaskDelay(pdMS_TO_TICKS(500));

  while (true) {
    uint32_t now = millis();
    
    GlobalState gs = getGlobalState();
    SubState ss1 = getSub1State();
    SubState ss2 = getSub2State();
    
    // Format sensor values with NaN handling
    char ah0s[8], ah1s[8], ah2s[8];
    char t0s[8], t1s[8], t2s[8];
    char d0s[8], d1s[8];
    
    snprintf(ah0s, sizeof(ah0s), isnan(g_dhtAH_ema[0]) ? "--" : "%.1f", g_dhtAH_ema[0]);
    snprintf(ah1s, sizeof(ah1s), isnan(g_dhtAH_ema[1]) ? "--" : "%.1f", g_dhtAH_ema[1]);
    snprintf(ah2s, sizeof(ah2s), isnan(g_dhtAH_ema[2]) ? "--" : "%.1f", g_dhtAH_ema[2]);
    
    snprintf(t0s, sizeof(t0s), isnan(g_dhtTemp[0]) ? "--" : "%.1f", g_dhtTemp[0]);
    snprintf(t1s, sizeof(t1s), isnan(g_dhtTemp[1]) ? "--" : "%.1f", g_dhtTemp[1]);
    snprintf(t2s, sizeof(t2s), isnan(g_dhtTemp[2]) ? "--" : "%.1f", g_dhtTemp[2]);
    
    snprintf(d0s, sizeof(d0s), isnan(g_dhtAHDiff[0]) ? "--" : "%+.1f", g_dhtAHDiff[0]);
    snprintf(d1s, sizeof(d1s), isnan(g_dhtAHDiff[1]) ? "--" : "%+.1f", g_dhtAHDiff[1]);
    
    // Control states
    bool m0 = motorIsOn(0), m1 = motorIsOn(1);
    bool ht0 = heaterIsOn(0), ht1 = heaterIsOn(1);
    uint32_t m0pct = m0 ? getMotorDutyCycle(0) : 0;
    uint32_t m1pct = m1 ? getMotorDutyCycle(1) : 0;
    
    // Format display message
    char msg[128];
    snprintf(msg, sizeof(msg),
             "AH: %s %s %s\n"
             "T:  %s %s %s\n"
             "D:       %s %s\n"
             "---\n"
             "M: %d%% %d%%\n"
             "H: %s / %s\n"
             "%s/%s/%s",
             ah0s, ah1s, ah2s,
             t0s, t1s, t2s,
             d0s, d1s,
             (int)m0pct, (int)m1pct,
             (ht0 ? "ON" : "--"), (ht1 ? "ON" : "--"),
             getGlobalStateAbbr(gs),
             getSubStateAbbr(ss1),
             getSubStateAbbr(ss2));
    
    rightScreen.showMessage(String(msg));
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//==============================================================================
// TASK CREATION
//==============================================================================

void createOledTasks() {
  xTaskCreatePinnedToCore(vLeftScreenTask, "LeftScreen", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vRightScreenTask, "RightScreen", 4096, NULL, 1, NULL, 0);
}
