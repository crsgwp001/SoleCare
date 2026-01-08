#include "global.h"

// raw readings
volatile float g_dhtTemp[3] = {0};
volatile float g_dhtHum[3] = {0};

// instantaneous AH
volatile float g_dhtAH[3] = {0};

// diffs vs. sensor0
volatile float g_dhtAHDiff[2] = {0};

// EMAs (seed to NAN so first sample initializes)
volatile float g_dhtAH_ema[3] = {NAN, NAN, NAN};
volatile float g_dhtAHDiff_ema[2] = {NAN, NAN};

// boolean status: true == wet
volatile bool g_dhtIsWet[2] = {false, false};

// AH rate-of-change in g/m³/min for sensors 1 and 2
float g_dhtAHRate[2] = {0.0f, 0.0f};

// NaN counters per DHT sensor
volatile uint32_t g_dhtNaNCount[3] = {0, 0, 0};

// Cached battery voltage
volatile float g_lastBatteryVoltage = 0.0f;

// Battery voltage monitoring implementation
float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    sum += analogRead(HW_BATTERY_ADC_PIN);
    delay(2);
  }
  int raw = sum / BATTERY_ADC_SAMPLES;
  
  float vAdc = (raw / 4095.0f) * BATTERY_VFS;
  float vBat = vAdc * (BATTERY_R1 + BATTERY_R2) / BATTERY_R2;
  // Apply fixed offset to address consistent under-read under load
  vBat += BATTERY_OFFSET_V;
  
  // Battery debug disabled
  
  return vBat;
}

bool isBatteryOk() {
  float voltage = readBatteryVoltage();
  return voltage >= BATTERY_LOW_THRESHOLD;
}

bool isBatteryRecovered() {
  float voltage = readBatteryVoltage();
  return voltage >= BATTERY_RECOVERY_THRESHOLD;
}

// LED status control - called from FSM
void updateStatusLEDs() {
  // This is a placeholder - actual LED control is done directly in FSM state callbacks
  // to avoid circular dependencies
}
