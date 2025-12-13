#pragma once
#include <cstdint>

// ==================== SENSOR THRESHOLDS ====================
constexpr float AH_WET_THRESHOLD = 1.0f;   // Shoe is wet when AH diff > this value (enter WET)
constexpr float AH_DRY_THRESHOLD = 0.7f;   // Shoe is dry when AH diff < this value (exit COOLING to DRY, hysteresis from WET)
constexpr float AH_DRY_THRESHOLD_LENIENT = 1.0f;  // Lenient threshold when AH diff is consistently declining
constexpr float EMA_ALPHA = 0.2f;
constexpr float AMB_AH_OFFSET = 0.0f;

// EMI Protection: Maximum allowed AH change per sample (g/m³)
// Normal changes are < 0.5 g/m³/sample; anything larger is likely EMI noise
constexpr float MAX_AH_DELTA_PER_SAMPLE = 2.0f;

// ==================== TIMING ====================
constexpr uint32_t DONE_TIMEOUT_MS = 10u * 1000u;
constexpr uint32_t WET_TIMEOUT_MS = 5u * 1000u;
constexpr uint32_t WET_MIN_DURATION_MS = 360u * 1000u;  // Minimum 6 minutes of WET phase (aggressive heater-on evaporation)
constexpr uint32_t WET_PEAK_BUFFER_MS = 75u * 1000u;  // Extra 75s buffer after peak detected for additional drying
constexpr uint32_t DRY_COOL_MS_BASE = 90u * 1000u;  // Base COOLING motor duration (reduced - most work done in WET)
constexpr uint32_t DRY_COOL_MS_WET = 150u * 1000u;  // Extended COOLING motor duration (reduced)
constexpr uint32_t DRY_COOL_MS_SOAKED = 180u * 1000u;  // Extended COOLING motor duration (reduced)
constexpr uint32_t DRY_STABILIZE_MS = 90u * 1000u;  // Stabilization phase after motor stops
constexpr uint32_t MOTOR_SAFETY_MS = 600u * 1000u;
constexpr uint32_t HW_UV_DEFAULT_MS = 10u * 1000u;
constexpr uint32_t HEATER_WARMUP_MS = 10u * 1000u;
constexpr uint32_t AH_ACCEL_WARMUP_MS = 180u * 1000u;  // Wait 3mins before checking AH acceleration for WET exit

// ==================== PID MOTOR CONTROL ====================
// Phase 1: P-only control with fixed setpoint and logging
#define PID_LOGGING_ENABLED 1  // Toggle PID logging on/off (0 = disabled, 1 = enabled)

// P+I+D tuning - balanced for responsive evaporation control
constexpr double PID_KP = 0.15;         // Proportional gain (increased for faster response)
constexpr double PID_KI = 0.03;         // Integral gain (increased slightly to eliminate steady-state error)
constexpr double PID_KD = 0.08;         // Derivative gain (increased for better damping)
constexpr unsigned long PID_SAMPLE_MS = 3000;  // 3-second sample interval (more stable measurements)

// Control parameters
constexpr double TARGET_AH_RATE = 0.4;  // Target AH rate (g/m³/min) - Phase 1 fixed
constexpr unsigned long PID_CONTROL_START_MS = 30000;  // Skip first 30s, use fixed 75%
constexpr double PID_OUT_MIN = 0.5;     // Minimum duty (50%)
constexpr double PID_OUT_MAX = 1.0;     // Maximum duty (100%)
constexpr int PID_FIXED_DUTY_PERCENT = 75;  // Fixed duty during warmup phase

// Phase 2: Dual-phase setpoint switching (implemented)
constexpr double TARGET_AH_RATE_EVAP = 0.45;     // Phase 2A: Aggressive evaporation target (increased)
constexpr double TARGET_AH_RATE_STABLE = 0.08;   // Phase 2B: Gentle stabilization target (increased slightly)
constexpr double AH_RATE_PHASE_THRESHOLD = 0.25; // Switch from EVAP to STABLE when rate < 0.25 (tighter)
constexpr unsigned long PID_PHASE1_MIN_MS = 120000; // Ensure at least 120s in Phase 1 before switching

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
constexpr int HW_STATUS_LED_PIN = 33;
constexpr int HW_ERROR_LED_PIN = 32;

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
