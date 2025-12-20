// tskDHT.cpp
#include "tskDHT.h"
#include "global.h"
#include "dev_debug.h"
#include <Sensor.h>          // for computeAH
#include <DHT.h>
#include "tskMotor.h"  // for optional EMI-aware adjustments and duty checks
#include <Adafruit_Sensor.h>
#include <cstring>

// Use Adafruit DHT directly (wrapper removed to reduce timing overhead)
static DHT dht0(HW_DHT_PIN_0, DHT22);
static DHT dht1(HW_DHT_PIN_1, DHT22);
static DHT dht2(HW_DHT_PIN_2, DHT22);

// Note: Avoid long critical sections around DHT reads on ESP32.
// The Adafruit DHT library already manages timing/interrupts; wrapping reads
// in critical sections can trigger the Interrupt WDT. We instead add brief
// yields between sensor reads and retry once on failure.

static inline void readDHTSafe(DHT &d, float &t, float &h) {
  // Attempt up to 3 reads with short delays; DHT22 can glitch near PWM edges
  t = NAN; h = NAN;
  for (int attempt = 0; attempt < 3; ++attempt) {
    float tr = d.readTemperature();
    float hr = d.readHumidity();
    if (!isnan(tr) && !isnan(hr)) {
      t = tr; h = hr; break;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}

static void vSensorTask(void * /*pvParameters*/) {
  dht0.begin();
  dht1.begin();
  dht2.begin();
  // DHT22 needs at least 2s after power-up before first read
  vTaskDelay(pdMS_TO_TICKS(2000));
  // Track if we've ever had a valid sample per sensor (to avoid using 0.0 on cold start)
  static bool s_hasValid[3] = {false, false, false};

  while (true) {
    // Sensor 0 (ambient)
    float t0, h0;
    readDHTSafe(dht0, t0, h0);

    DEV_DBG_PRINT("DHT0 GPIO"); DEV_DBG_PRINT(HW_DHT_PIN_0); DEV_DBG_PRINT(" -> T="); DEV_DBG_PRINT(t0); DEV_DBG_PRINT(" H="); DEV_DBG_PRINTLN(h0);
    if (!isnan(t0) && !isnan(h0)) {
      g_dhtTemp[0] = t0;
      g_dhtHum[0] = h0;
      s_hasValid[0] = true;
    }
    // Yield briefly to feed WDT/IDLE
    vTaskDelay(pdMS_TO_TICKS(1));

    // Sensor 1
    float t1, h1;
    readDHTSafe(dht1, t1, h1);

    DEV_DBG_PRINT("DHT1 GPIO"); DEV_DBG_PRINT(HW_DHT_PIN_1); DEV_DBG_PRINT(" -> T="); DEV_DBG_PRINT(t1); DEV_DBG_PRINT(" H="); DEV_DBG_PRINTLN(h1);
    if (!isnan(t1) && !isnan(h1)) {
      g_dhtTemp[1] = t1;
      g_dhtHum[1] = h1;
      s_hasValid[1] = true;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    // Sensor 2
    float t2, h2;
    readDHTSafe(dht2, t2, h2);

    DEV_DBG_PRINT("DHT2 GPIO"); DEV_DBG_PRINT(HW_DHT_PIN_2); DEV_DBG_PRINT(" -> T="); DEV_DBG_PRINT(t2); DEV_DBG_PRINT(" H="); DEV_DBG_PRINTLN(h2);
    if (!isnan(t2) && !isnan(h2)) {
      g_dhtTemp[2] = t2;
      g_dhtHum[2] = h2;
      s_hasValid[2] = true;
    }

    // Compute AH and simple wet flags (no EMA/no filtering)
    for (int i = 0; i < 3; ++i) {
      if (s_hasValid[i]) {
        float ah = computeAH(g_dhtTemp[i], g_dhtHum[i]);
        if (i == 0) {
          ah += AMB_AH_OFFSET;
        }
        float prev = g_dhtAH[i];
        if (!isnan(prev) && prev != 0.0f) {
          float delta = ah - prev;
          if (delta > MAX_AH_DELTA_PER_SAMPLE) ah = prev + MAX_AH_DELTA_PER_SAMPLE;
          else if (delta < -MAX_AH_DELTA_PER_SAMPLE) ah = prev - MAX_AH_DELTA_PER_SAMPLE;
        }
        g_dhtAH[i] = ah;
        g_dhtAH_ema[i] = ah;
      } else {
        // No valid sample ever seen for this sensor yet; hold as NAN
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
        // Preserve last diff and wet state when either AH is invalid (hold-last)
        // Intentionally no updates here
      }
    }

    // 3s interval like your working sketch
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void createSensorTask() {
  // Pin to core 1 to reduce contention with WiFi/IDLE on core 0 and avoid WDT
  xTaskCreatePinnedToCore(vSensorTask, "SensorTask", 4096, nullptr, 2, nullptr, 1);
}