#pragma once
#include <cstdint>

// Absolute-humidity thresholds (g/m^3) used to classify wet/dry
constexpr float AH_WET_THRESHOLD = 1.0f;
constexpr float AH_DRY_THRESHOLD = -1.0f;

// Exponential moving average smoothing factor (0..1). Larger = faster response.
constexpr float EMA_ALPHA = 0.2f;

// Ambient absolute-humidity offset applied only to sensor 0. Use a constexpr
// so the value participates in compile-time expressions and has proper type.
constexpr float AMB_AH_OFFSET = 0.0f; // adjust ambient AH reading here

// How long (ms) to remain in GLOBAL Done state before automatically
// resetting back to Idle. Default 10 seconds — adjust if you want a
// different auto-reset interval.
constexpr uint32_t DONE_TIMEOUT_MS = 10u * 1000u; // 10s

// --------------------- Hardware configuration ---------------------
// Place hardware-pin and polarity choices here so they are easy to
// change for different boards without touching source files.

// UV relay pins (two channels)
constexpr int HW_UV_PIN_0 = 18; // UV relay for channel 0
constexpr int HW_UV_PIN_1 = 5;  // UV relay for channel 1

// Set true if relay module is active-low (drive LOW to enable relay)
constexpr bool HW_RELAY_ACTIVE_LOW = false;

// Default UV on-duration (ms) when caller passes 0
constexpr uint32_t HW_UV_DEFAULT_MS = 10u * 1000u; // 10s (set to 10 seconds per request)

// Timeouts (ms) used by FSM and motor task; make these tunable
constexpr uint32_t WET_TIMEOUT_MS = 5u * 1000u; // safety timeout while in WET
// DRYER_RUNTIME_MS is legacy; cooling duration used is DRY_COOL_MS below
constexpr uint32_t DRYER_RUNTIME_MS = 20u * 1000u; // legacy; not used
constexpr uint32_t DRY_COOL_MS = 5u * 1000u;       // cooling period (5s)
constexpr uint32_t MOTOR_SAFETY_MS =
    120u * 1000u; // increased safety window to 120s for WET+COOLING

// DHT sensor GPIO pins (sensor0, sensor1, sensor2)
constexpr int HW_DHT_PIN_0 = 17;
constexpr int HW_DHT_PIN_1 = 16;
constexpr int HW_DHT_PIN_2 = 4;

// Push-button pins
constexpr int HW_START_PIN = 35;
constexpr int HW_RESET_PIN = 34;

// Motor & heater control pins (one pair per shoe)
// Change these to match your hardware wiring.
constexpr int HW_MOTOR_PIN_0 = 25;
constexpr int HW_MOTOR_PIN_1 = 26;
constexpr int HW_HEATER_PIN_0 = 14;
constexpr int HW_HEATER_PIN_1 = 12;
// Set true if outputs are active-low
constexpr bool HW_ACTUATOR_ACTIVE_LOW = false;

// Battery voltage monitoring (voltage divider on ADC)
constexpr int HW_BATTERY_ADC_PIN = 39;  // GPIO39 (ADC1_CH3)
constexpr float BATTERY_R1 = 33000.0f;  // Upper resistor in voltage divider (Ohms)
constexpr float BATTERY_R2 = 10000.0f;  // Lower resistor in voltage divider (Ohms)
constexpr float BATTERY_VFS = 3.28f;    // ADC full-scale voltage (measure your 3.3V rail)
constexpr float BATTERY_LOW_THRESHOLD = 3.0f;  // Minimum battery voltage to run (V)
constexpr float BATTERY_RECOVERY_THRESHOLD = 3.2f;  // Battery voltage to exit LowBattery state (V)
constexpr int BATTERY_ADC_SAMPLES = 32; // Number of samples to average
constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000u; // Check battery every 1 second in LowBattery state

// Status LED pins
constexpr int HW_STATUS_LED_PIN = 32;   // Green LED - on when idle/ready, off when running
constexpr int HW_ERROR_LED_PIN = 33;    // Red LED - blinks when running, solid on error

// ------------------------------------------------------------------
