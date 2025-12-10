#pragma once
#include <cstdint>

// ==================== SENSOR THRESHOLDS ====================
constexpr float AH_WET_THRESHOLD = 1.0f;
constexpr float AH_DRY_THRESHOLD = -1.0f;
constexpr float EMA_ALPHA = 0.2f;
constexpr float AMB_AH_OFFSET = 0.0f;

// ==================== TIMING ====================
constexpr uint32_t DONE_TIMEOUT_MS = 10u * 1000u;
constexpr uint32_t WET_TIMEOUT_MS = 5u * 1000u;
constexpr uint32_t DRY_COOL_MS = 5u * 1000u;
constexpr uint32_t MOTOR_SAFETY_MS = 120u * 1000u;
constexpr uint32_t HW_UV_DEFAULT_MS = 10u * 1000u;

// ==================== GPIO PINS ====================
// Sensors
constexpr int HW_DHT_PIN_0 = 17;
constexpr int HW_DHT_PIN_1 = 16;
constexpr int HW_DHT_PIN_2 = 4;

// Buttons
constexpr int HW_START_PIN = 35;
constexpr int HW_RESET_PIN = 34;

// Motors (PWM/MOSFET)
constexpr int HW_MOTOR_PIN_0 = 25;
constexpr int HW_MOTOR_PIN_1 = 26;

// Heaters (Relay)
constexpr int HW_HEATER_PIN_0 = 18;
constexpr int HW_HEATER_PIN_1 = 5;

// UV (PWM/MOSFET, single channel)
constexpr int HW_UV_PIN_0 = 14;
constexpr int HW_UV_PIN_1 = 12;

// LEDs
constexpr int HW_STATUS_LED_PIN = 32;
constexpr int HW_ERROR_LED_PIN = 33;

// Battery
constexpr int HW_BATTERY_ADC_PIN = 39;

// ==================== HARDWARE POLARITY ====================
constexpr bool HW_RELAY_ACTIVE_LOW = false;
constexpr bool HW_ACTUATOR_ACTIVE_LOW = false;

// ==================== BATTERY MONITORING ====================
constexpr float BATTERY_R1 = 33000.0f;
constexpr float BATTERY_R2 = 10000.0f;
constexpr float BATTERY_VFS = 3.28f;
constexpr float BATTERY_LOW_THRESHOLD = 3.0f;
constexpr float BATTERY_RECOVERY_THRESHOLD = 3.2f;
constexpr int BATTERY_ADC_SAMPLES = 32;
constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000u;
