// tskDHT.cpp
#include "tskDHT.h"
#include "global.h"
#include "dev_debug.h"
#include <Sensor.h>
#include <cstring> // strcpy()

// three DHTSensor instances on configurable pins
DHTSensor sensor1(HW_DHT_PIN_0), sensor2(HW_DHT_PIN_1), sensor3(HW_DHT_PIN_2);

// Track last valid AH and NaN streaks to survive noisy reads
static float g_lastValidAH[3] = {NAN, NAN, NAN};
static int g_nanStreak[3] = {0, 0, 0};
static constexpr int MAX_NAN_STREAK = 3;  // After this, reuse last valid AH

// Track last valid EMA for rate-of-change limiting (EMI protection)
static float g_lastValidEMA[3] = {NAN, NAN, NAN};

// Track last valid temperature for EMI failure recovery
static float g_lastValidTemp[3] = {NAN, NAN, NAN};
static int g_tempFailStreak[3] = {0, 0, 0};
static constexpr int MAX_TEMP_FAIL_STREAK = 3;  // Reject bad temps after a few fails

// Track last valid humidity for EMI failure recovery
static float g_lastValidHum[3] = {NAN, NAN, NAN};
static int g_humFailStreak[3] = {0, 0, 0};

static void vSensorTask(void * /*pvParameters*/) {
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();
  while (true) {
    // 1) Raw reads + yield to avoid hogging interrupts
    // Add delays between reads to reduce EMI interference from motor PWM
    // Extra care for sensor 0 (ambient) which is most susceptible to motor noise
    
    // Sensor 0 (ambient): Read with retry on failure
    g_dhtTemp[0] = sensor1.readTemperature();
    vTaskDelay(pdMS_TO_TICKS(10));
    if (isnan(g_dhtTemp[0]) || g_dhtTemp[0] == 0.0f) {
      // Sensor 0 read failed, retry immediately
      vTaskDelay(pdMS_TO_TICKS(15));
      g_dhtTemp[0] = sensor1.readTemperature();
    }
    g_dhtTemp[0] += 0.2f; // calibration
    
    g_dhtHum[0] = sensor1.readHumidity();
    vTaskDelay(pdMS_TO_TICKS(10));
    g_dhtHum[0] += 2.0f;
    
    // Sensor 1 (shoe 1)
    g_dhtTemp[1] = sensor2.readTemperature();
    vTaskDelay(pdMS_TO_TICKS(8));
    g_dhtHum[1] = sensor2.readHumidity();
    vTaskDelay(pdMS_TO_TICKS(8));
    
    // Sensor 2 (shoe 2)
    g_dhtTemp[2] = sensor3.readTemperature();
    vTaskDelay(pdMS_TO_TICKS(8));
    g_dhtHum[2] = sensor3.readHumidity();
    vTaskDelay(pdMS_TO_TICKS(8));

    // Validate temperature reads: reject NaN (0.00 after formatting) or unreasonable values
    for (int i = 0; i < 3; ++i) {
      // NaN check
      if (isnan(g_dhtTemp[i])) {
        g_tempFailStreak[i]++;
        if (!isnan(g_lastValidTemp[i])) {
          // Use last valid temp immediately on first NaN
          g_dhtTemp[i] = g_lastValidTemp[i];
        }
        continue;
      }
      
      // Reject physically impossible temps:
      // - 0°C is almost never correct during normal operation (31°C shoes)
      // - Out of valid range (-40 to 85°C)
      if (g_dhtTemp[i] < -35.0f || g_dhtTemp[i] > 85.0f || 
          (g_dhtTemp[i] < 10.0f && !isnan(g_lastValidTemp[i]) && g_lastValidTemp[i] > 20.0f)) {
        // Out of range or sudden drop - likely EMI/read error
        g_tempFailStreak[i]++;
        if (!isnan(g_lastValidTemp[i])) {
          // Hold last valid temp (EMI rejection)
          g_dhtTemp[i] = g_lastValidTemp[i];
        } else {
          g_dhtTemp[i] = NAN;  // Mark invalid if we have no fallback
        }
      } else {
        // Valid temp
        g_tempFailStreak[i] = 0;
        g_lastValidTemp[i] = g_dhtTemp[i];
      }
    }

    // 2) Validate humidity reads: reject NaN or out-of-range values
    for (int i = 0; i < 3; ++i) {
      // NaN check
      if (isnan(g_dhtHum[i])) {
        g_humFailStreak[i]++;
        if (!isnan(g_lastValidHum[i])) {
          // Use last valid humidity immediately on first NaN
          g_dhtHum[i] = g_lastValidHum[i];
        }
        continue;
      }
      
      // Reject out-of-range or sudden jump values:
      // - Valid humidity is 0-100%
      // - Sudden jumps >30% = likely EMI
      if (g_dhtHum[i] < 0.0f || g_dhtHum[i] > 100.0f ||
          (!isnan(g_lastValidHum[i]) && fabs(g_dhtHum[i] - g_lastValidHum[i]) > 30.0f)) {
        // Out of range or sudden spike - likely EMI/read error
        g_humFailStreak[i]++;
        if (!isnan(g_lastValidHum[i])) {
          // Hold last valid humidity (EMI rejection)
          g_dhtHum[i] = g_lastValidHum[i];
        } else {
          g_dhtHum[i] = NAN;  // Mark invalid if we have no fallback
        }
      } else {
        // Valid humidity
        g_humFailStreak[i] = 0;
        g_lastValidHum[i] = g_dhtHum[i];
      }
    }

    // 3) Compute absolute humidity & EMA (skip/replace invalid reads to avoid NaN propagation)
    for (int i = 0; i < 3; ++i) {
      float t = g_dhtTemp[i];
      float h = g_dhtHum[i];

      // Clamp obviously bad inputs (noise may produce out-of-range values)
      if (!isnan(h)) {
        if (h < 0.0f) h = 0.0f;
        if (h > 100.0f) h = 100.0f;
      }
      if (!isnan(t)) {
        if (t < -40.0f) t = -40.0f;
        if (t > 85.0f) t = 85.0f;
      }

      float ah = computeAH(t, h);

      // If computeAH returned NaN (invalid inputs), fall back to last valid AH after a few streaks
      if (isnan(ah)) {
        g_nanStreak[i]++;
        if (!isnan(g_lastValidAH[i]) && g_nanStreak[i] > MAX_NAN_STREAK) {
          ah = g_lastValidAH[i];  // reuse last good value to keep EMA alive
        } else {
          continue;  // still waiting for valid
        }
      } else {
        g_nanStreak[i] = 0;  // valid read resets streak
        g_lastValidAH[i] = ah;
      }
      if (i == 0)
        ah += kAmbAhOffset; // only sensor 0 gets the ambient offset
      g_dhtAH[i] = ah;
      
      // EMI Protection: Rate-limit AH changes to reject sudden spikes from motor interference
      if (!isnan(g_lastValidEMA[i])) {
        float delta = ah - g_lastValidEMA[i];
        if (fabs(delta) > MAX_AH_DELTA_PER_SAMPLE) {
          // Change too large - likely EMI noise, reject this sample and hold EMA
          continue;
        }
      }
      
      if (isnan(g_dhtAH_ema[i])) {
        // Seed EMA on first valid reading
        g_dhtAH_ema[i] = ah;
        g_lastValidEMA[i] = ah;
      } else {
        g_dhtAH_ema[i] = EMA_ALPHA * ah + (1.0f - EMA_ALPHA) * g_dhtAH_ema[i];
        g_lastValidEMA[i] = g_dhtAH_ema[i];
      }
    }

    // 4) Compute raw diff + EMA diff + status (only when we have valid EMA values)
    for (int i = 1; i < 3; ++i) {
      if (isnan(g_dhtAH_ema[0]) || isnan(g_dhtAH_ema[i])) {
        // Skip this iteration if either ambient or shoe EMA is invalid
        continue;
      }

      float diff = g_dhtAH_ema[i] - g_dhtAH_ema[0];
      g_dhtAHDiff[i - 1] = diff;
      
      // Apply EMA to the diff as well
      if (isnan(g_dhtAHDiff_ema[i - 1])) {
        // Seed EMA on first valid reading
        g_dhtAHDiff_ema[i - 1] = diff;
      } else {
        g_dhtAHDiff_ema[i - 1] = EMA_ALPHA * diff + (1.0f - EMA_ALPHA) * g_dhtAHDiff_ema[i - 1];
      }
      
      // Set boolean wet/dry flag
      g_dhtIsWet[i - 1] = (diff > AH_WET_THRESHOLD);
    }

    // 5) Debug prints
    DEV_DBG_PRINT("S0: ");
    DEV_DBG_PRINT(g_dhtTemp[0], 1);
    DEV_DBG_PRINT("C ");
    DEV_DBG_PRINT(g_dhtHum[0], 1);
    DEV_DBG_PRINT("% AH: ");
    DEV_DBG_PRINTLN(g_dhtAH_ema[0], 2);
    taskYIELD();
    DEV_DBG_PRINT("S1: ");
    DEV_DBG_PRINT(g_dhtTemp[1], 1);
    DEV_DBG_PRINT("C ");
    DEV_DBG_PRINT(g_dhtHum[1], 1);
    DEV_DBG_PRINT("% AH: ");
    DEV_DBG_PRINT(g_dhtAH_ema[1], 2);
    DEV_DBG_PRINT(" Δ: ");
    DEV_DBG_PRINT(g_dhtAHDiff[0], 2);
    DEV_DBG_PRINT(" ");
    DEV_DBG_PRINTLN(g_dhtIsWet[0] ? "WET" : "DRY");
    taskYIELD();
    DEV_DBG_PRINT("S2: ");
    DEV_DBG_PRINT(g_dhtTemp[2], 1);
    DEV_DBG_PRINT("C ");
    DEV_DBG_PRINT(g_dhtHum[2], 1);
    DEV_DBG_PRINT("% AH: ");
    DEV_DBG_PRINT(g_dhtAH_ema[2], 2);
    DEV_DBG_PRINT(" Δ: ");
    DEV_DBG_PRINT(g_dhtAHDiff[1], 2);
    DEV_DBG_PRINT(" ");
    DEV_DBG_PRINTLN(g_dhtIsWet[1] ? "WET" : "DRY");
    taskYIELD();

    // 6) Delay to free CPU & reset task watchdog
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void createSensorTask() {
  xTaskCreatePinnedToCore(vSensorTask, "SensorTask", 4096, nullptr, 1, nullptr, 1);
}