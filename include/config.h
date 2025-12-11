#pragma once
#include <cstdint>

// ==================== SENSOR THRESHOLDS ====================
constexpr float AH_WET_THRESHOLD = 1.0f;   // Shoe is wet when AH diff > this value
constexpr float AH_DRY_THRESHOLD = 1.0f;   // Shoe is dry when AH diff < this value (tunable)
constexpr float EMA_ALPHA = 0.2f;
constexpr float AMB_AH_OFFSET = 0.0f;

// ==================== TIMING ====================
constexpr uint32_t DONE_TIMEOUT_MS = 10u * 1000u;
constexpr uint32_t WET_TIMEOUT_MS = 5u * 1000u;
constexpr uint32_t DRY_COOL_MS = 5u * 1000u;
constexpr uint32_t DRY_STABILIZE_MS = 5u * 1000u;
constexpr uint32_t MOTOR_SAFETY_MS = 180u * 1000u;
constexpr uint32_t HW_UV_DEFAULT_MS = 10u * 1000u;
constexpr uint32_t HEATER_WARMUP_MS = 5u * 1000u;

// ==================== PID MOTOR CONTROL ====================
// Phase 1: P-only control with fixed setpoint and logging
#define PID_LOGGING_ENABLED 1  // Toggle PID logging on/off (0 = disabled, 1 = enabled)

// P-only tuning (Phase 1)
constexpr double PID_KP = 5.0;          // Proportional gain (conservative)
constexpr double PID_KI = 0.0;          // Integral gain (disabled for Phase 1)
constexpr double PID_KD = 0.0;          // Derivative gain (disabled for Phase 1)
constexpr unsigned long PID_SAMPLE_MS = 2000;  // 2-second sample interval

// Control parameters
constexpr double TARGET_AH_RATE = 0.4;  // Target AH rate (g/m³/min) - Phase 1 fixed
constexpr unsigned long PID_CONTROL_START_MS = 30000;  // Skip first 30s, use fixed 75%
constexpr double PID_OUT_MIN = 0.3;     // Minimum duty (30%)
constexpr double PID_OUT_MAX = 1.0;     // Maximum duty (100%)
constexpr int PID_FIXED_DUTY_PERCENT = 75;  // Fixed duty during warmup phase

// Phase 2: Dual-phase setpoint switching (ready for implementation)
// Uncomment and modify when ready to implement Phase 2
// constexpr double TARGET_AH_RATE_EVAP = 0.4;    // Phase 2A: Aggressive evaporation
// constexpr double TARGET_AH_RATE_STABLE = -0.1; // Phase 2B: Gentle stabilization
// constexpr double AH_RATE_PHASE_THRESHOLD = 0.1; // Threshold to switch phases

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
constexpr float BATTERY_LOW_THRESHOLD = 10.0f;
constexpr float BATTERY_RECOVERY_THRESHOLD = 10.8f;
constexpr int BATTERY_ADC_SAMPLES = 32;
constexpr uint32_t BATTERY_CHECK_INTERVAL_MS = 1000u;
