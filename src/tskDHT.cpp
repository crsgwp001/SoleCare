// tskDHT.cpp
#include "tskDHT.h"
#include "global.h"
#include "dev_debug.h"
#include <Sensor.h>          // for computeAH
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <cstring>

// Use Adafruit DHT directly (wrapper removed to reduce timing overhead)
static DHT dht0(HW_DHT_PIN_0, DHT22);
static DHT dht1(HW_DHT_PIN_1, DHT22);
static DHT dht2(HW_DHT_PIN_2, DHT22);

// Protect DHT bitbang timing from other ISRs
static portMUX_TYPE g_dhtMux = portMUX_INITIALIZER_UNLOCKED;

static void vSensorTask(void * /*pvParameters*/) {
  dht0.begin();
  dht1.begin();
  dht2.begin();
  // DHT22 needs at least 2s after power-up before first read
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (true) {
    // Sensor 0 (ambient)
    taskENTER_CRITICAL(&g_dhtMux);
    float t0 = dht0.readTemperature();
    float h0 = dht0.readHumidity();
    taskEXIT_CRITICAL(&g_dhtMux);

    DEV_DBG_PRINT("DHT0 GPIO"); DEV_DBG_PRINT(HW_DHT_PIN_0); DEV_DBG_PRINT(" -> T="); DEV_DBG_PRINT(t0); DEV_DBG_PRINT(" H="); DEV_DBG_PRINTLN(h0);
    g_dhtTemp[0] = t0;
    g_dhtHum[0] = h0;

    // Sensor 1
    taskENTER_CRITICAL(&g_dhtMux);
    float t1 = dht1.readTemperature();
    float h1 = dht1.readHumidity();
    taskEXIT_CRITICAL(&g_dhtMux);

    DEV_DBG_PRINT("DHT1 GPIO"); DEV_DBG_PRINT(HW_DHT_PIN_1); DEV_DBG_PRINT(" -> T="); DEV_DBG_PRINT(t1); DEV_DBG_PRINT(" H="); DEV_DBG_PRINTLN(h1);
    g_dhtTemp[1] = t1;
    g_dhtHum[1] = h1;

    // Sensor 2
    taskENTER_CRITICAL(&g_dhtMux);
    float t2 = dht2.readTemperature();
    float h2 = dht2.readHumidity();
    taskEXIT_CRITICAL(&g_dhtMux);

    DEV_DBG_PRINT("DHT2 GPIO"); DEV_DBG_PRINT(HW_DHT_PIN_2); DEV_DBG_PRINT(" -> T="); DEV_DBG_PRINT(t2); DEV_DBG_PRINT(" H="); DEV_DBG_PRINTLN(h2);
    g_dhtTemp[2] = t2;
    g_dhtHum[2] = h2;

    // Compute AH and simple wet flags (no EMA/no filtering)
    for (int i = 0; i < 3; ++i) {
      if (!isnan(g_dhtTemp[i]) && !isnan(g_dhtHum[i])) {
        float ah = computeAH(g_dhtTemp[i], g_dhtHum[i]);
        // Apply offset to ambient sensor (sensor 0)
        if (i == 0) {
          ah += AMB_AH_OFFSET;
        }
        g_dhtAH[i] = ah;
        g_dhtAH_ema[i] = ah;
      } else {
        g_dhtAH[i] = NAN;
        g_dhtAH_ema[i] = NAN;
      }
    }

    for (int i = 1; i < 3; ++i) {
      if (!isnan(g_dhtAH[0]) && !isnan(g_dhtAH[i])) {
        float diff = g_dhtAH[i] - g_dhtAH[0];
        g_dhtAHDiff[i - 1] = diff;
        g_dhtAHDiff_ema[i - 1] = diff;
        g_dhtIsWet[i - 1] = (diff > AH_WET_THRESHOLD);
      } else {
        // Invalid reading from ambient or shoe sensor; do not force DRY.
        // Preserve last valid wet/dry state and mark diffs as NAN so
        // downstream checks ignore this cycle.
        g_dhtAHDiff[i - 1] = NAN;
        g_dhtAHDiff_ema[i - 1] = NAN;
        // Intentionally leave g_dhtIsWet[i - 1] unchanged
      }
    }

    // 3s interval like your working sketch
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void createSensorTask() {
  // Run on core 0 with slightly higher priority to minimize jitter during DHT bitbang
  xTaskCreatePinnedToCore(vSensorTask, "SensorTask", 4096, nullptr, 2, nullptr, 0);
}