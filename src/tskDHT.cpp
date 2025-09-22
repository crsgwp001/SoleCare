// tskDHT.cpp
#include "tskDHT.h"
#include "dev_debug.h"
#include "global.h"
#include <Sensor.h>
#include <cstring> // strcpy()

// three DHTSensor instances on configurable pins
DHTSensor sensor1(HW_DHT_PIN_0), sensor2(HW_DHT_PIN_1), sensor3(HW_DHT_PIN_2);

static void vSensorTask(void * /*pvParameters*/) {
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();
  while (true) {
    // 1) Raw reads + yield to avoid hogging interrupts
    g_dhtTemp[0] = sensor1.readTemperature();
    taskYIELD();
    g_dhtTemp[0] += 0.2f; // calibration
    g_dhtHum[0] = sensor1.readHumidity();
    taskYIELD();
    g_dhtHum[0] += 2.0f;
    g_dhtTemp[1] = sensor2.readTemperature();
    taskYIELD();
    g_dhtHum[1] = sensor2.readHumidity();
    taskYIELD();
    g_dhtTemp[2] = sensor3.readTemperature();
    taskYIELD();
    g_dhtHum[2] = sensor3.readHumidity();
    taskYIELD();

    // 2) Compute absolute humidity & EMA
    for (int i = 0; i < 3; ++i) {
      float ah = computeAH(g_dhtTemp[i], g_dhtHum[i]);
      // If computeAH returned NAN (invalid inputs), skip updating AH/EMA
      if (isnan(ah)) {
        // Preserve previous values (don't stomp with NAN). Continue to next sensor.
        DEV_DBG_PRINTLN("Sensor ");
        DEV_DBG_PRINTLN(i); // lightweight debug
        continue;
      }
      if (i == 0)
        ah += kAmbAhOffset; // only sensor 0 gets the ambient offset
      g_dhtAH[i] = ah;
      if (isnan(g_dhtAH_ema[i])) {
        // Seed EMA on first valid reading
        g_dhtAH_ema[i] = ah;
      } else {
        g_dhtAH_ema[i] = EMA_ALPHA * ah + (1.0f - EMA_ALPHA) * g_dhtAH_ema[i];
      }
    }

    // 3) Compute raw diff (no EMA) + status
    for (int i = 1; i < 3; ++i) {
      float diff = g_dhtAH_ema[i] - g_dhtAH_ema[0];
      g_dhtAHDiff[i - 1] = diff;
      // Set boolean wet/dry flag
      g_dhtIsWet[i - 1] = (diff > AH_WET_THRESHOLD);
    }

    // 4) Debug prints
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

    // 5) Delay to free CPU & reset task watchdog
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void createSensorTask() {
  xTaskCreatePinnedToCore(vSensorTask, "SensorTask", 4096, nullptr, 1, nullptr, 1);
}