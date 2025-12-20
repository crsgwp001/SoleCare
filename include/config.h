#pragma once
#include <cstdint>

// ==================== SENSOR THRESHOLDS ====================
constexpr float AH_WET_THRESHOLD = 1.0f;   // Shoe is wet when AH diff > this value (enter WET)
constexpr float AH_DRY_THRESHOLD = 0.7f;   // Shoe is dry when AH diff < this value (exit COOLING to DRY, hysteresis from WET)
constexpr float AH_DRY_THRESHOLD_LENIENT = 1.0f;  // Lenient threshold when AH diff is consistently declining
constexpr float EMA_ALPHA = 0.2f;
constexpr float AMB_AH_OFFSET = 0.7f;

// EMI Protection: Maximum allowed AH change per sample (g/m³)
// Normal changes are < 0.5 g/m³/sample; anything larger is likely EMI noise
constexpr float MAX_AH_DELTA_PER_SAMPLE = 2.0f;

// ==================== TIMING ====================
constexpr uint32_t DONE_TIMEOUT_MS = 10u * 1000u;
constexpr uint32_t WET_TIMEOUT_MS = 5u * 1000u;

// Adaptive WET duration tiers based on initial moisture level
// This balances drying effectiveness with energy efficiency
constexpr uint32_t WET_BARELY_WET_MS = 180u * 1000u;         // Barely wet (0.7-1.5): 3 minutes
constexpr uint32_t WET_MODERATE_MS = 360u * 1000u;           // Moderate (1.5-3.5): 6 minutes
constexpr uint32_t WET_VERY_WET_MS = 480u * 1000u;           // Very wet (3.5-5.0): 8 minutes
constexpr uint32_t WET_SOAKED_MS = 600u * 1000u;             // Soaked (5.0+): 10 minutes

// Corresponding post-peak buffer durations
constexpr uint32_t WET_BUFFER_BARELY_WET_MS = 40u * 1000u;   // 40 seconds for barely wet
constexpr uint32_t WET_BUFFER_MODERATE_MS = 75u * 1000u;     // 75 seconds for moderate
constexpr uint32_t WET_BUFFER_VERY_WET_MS = 100u * 1000u;    // 100 seconds for very wet
constexpr uint32_t WET_BUFFER_SOAKED_MS = 120u * 1000u;      // 120 seconds for soaked

// Thresholds for moisture classification
constexpr float AH_DIFF_BARELY_WET = 1.5f;                   // g/m³
constexpr float AH_DIFF_MODERATE_WET = 3.5f;                 // g/m³
constexpr float AH_DIFF_VERY_WET = 5.0f;                     // g/m³

constexpr uint32_t DRY_COOL_MS_BASE = 90u * 1000u;  // Base COOLING motor duration (reduced - most work done in WET)
constexpr uint32_t DRY_COOL_MS_WET = 150u * 1000u;  // Extended COOLING motor duration (reduced)
constexpr uint32_t DRY_COOL_MS_SOAKED = 180u * 1000u;  // Extended COOLING motor duration (reduced)
constexpr uint32_t DRY_STABILIZE_MS = 90u * 1000u;  // Stabilization phase after motor stops
constexpr uint32_t MOTOR_SAFETY_MS = 600u * 1000u;
constexpr uint32_t HW_UV_DEFAULT_MS = 57u * 1000u;
constexpr uint32_t HEATER_WARMUP_MS = 12u * 1000u;  // Extended for cold shoes (was 10s)
constexpr uint32_t HEATER_WARMUP_MOTOR_DUTY = 65u;  // Motor duty during heater-only phase (before PID takes over)
constexpr float COOLING_TEMP_FAN_BOOST_ON = 38.5f;      // If shoe temp >= this, boost fan during COOLING
constexpr float COOLING_TEMP_RELEASE_C = 37.0f;         // Do not end COOLING motor phase until temp <= this
constexpr uint32_t COOLING_TEMP_EXTEND_MAX_MS = 120u * 1000u; // Cap extra motor run for temperature hold
// Ambient-coupled cooling target
constexpr float COOLING_AMBIENT_DELTA_C = 0.5f;              // Aim to cool to ambient + 0.5C
constexpr int COOLING_FAN_MIN_DUTY = 40;                     // Minimum fan duty during COOLING (low speed for passive cooling, avoid friction heating)
constexpr int COOLING_FAN_MAX_DUTY = 55;                     // Maximum fan duty during COOLING (cap at 55% to prevent friction heating reheating shoe)
// Re-evap short cycle (Option A)
constexpr uint32_t RE_EVAP_MAX_MS = 60u * 1000u;             // Max re-evap duration (heater+motor)
constexpr int RE_EVAP_MOTOR_DUTY = 85;                       // Motor duty during re-evap
constexpr float RE_EVAP_RISE_BARE_MOD = 0.6f;                // Rise-from-min threshold (barely/moderate)
constexpr float RE_EVAP_RISE_VERY = 0.8f;                    // Rise threshold (very wet)
constexpr float RE_EVAP_RISE_SOAKED = 1.0f;                  // Rise threshold (soaked)
constexpr uint32_t RE_EVAP_MIN_TIME_BARE_MOD = 45u * 1000u;  // Min time before rise valid
constexpr uint32_t RE_EVAP_MIN_TIME_VERY = 90u * 1000u;
constexpr uint32_t RE_EVAP_MIN_TIME_SOAKED = 120u * 1000u;
// WET post-peak buffer safeguards
constexpr float WET_BUFFER_TEMP_HOT_C = 38.5f;          // If temp >= this at buffer end, extend buffer
constexpr uint32_t WET_BUFFER_TEMP_EXTEND_MS = 30u * 1000u; // Add 30s buffer when hot
// Adaptive heater warmup thresholds (override base when shoe already warm)
constexpr float HEATER_WARMUP_FAST_35C = 35.0f;   // very warm shoe
constexpr float HEATER_WARMUP_FAST_30C = 30.0f;   // warm shoe
constexpr float HEATER_WARMUP_FAST_25C = 25.0f;   // mildly warm
constexpr uint32_t HEATER_WARMUP_35C_MS = 3u * 1000u;
constexpr uint32_t HEATER_WARMUP_30C_MS = 5u * 1000u;
constexpr uint32_t HEATER_WARMUP_25C_MS = 7u * 1000u;
// Heater efficiency cutoffs
// Bang-bang heater control thresholds for WET phase
constexpr float HEATER_MAX_TEMP_C = 39.0f;        // OFF at >= 39°C (aligned with smart controller)
constexpr float HEATER_REENABLE_TEMP_C = 39.0f;   // ON at <= 39°C (single-threshold controller uses trend)
constexpr bool HEATER_EARLY_OFF_ON_PEAK = true;   // disable heater when evaporation peak detected
constexpr uint32_t AH_ACCEL_WARMUP_MS = 180u * 1000u;  // Wait 3mins before checking AH acceleration for WET exit

// Robustness thresholds for WET phase exit
constexpr float AH_RATE_NORMAL_PEAK_THRESHOLD = 0.35f;    // Peak must have rate < this for normal loads (g/m³/min)
constexpr float AH_RATE_WET_PEAK_THRESHOLD = 0.60f;       // Peak must have rate < this for very wet (5+ g/m³) shoes
constexpr uint32_t AH_PEAK_NORMAL_MIN_TIME_MS = 120u * 1000u;  // Normal shoes: require 120s before peak valid
constexpr uint32_t AH_PEAK_WET_MIN_TIME_MS = 240u * 1000u;     // Very wet shoes: require 240s before peak valid (4 min)
constexpr float AH_DIFF_SAFETY_MARGIN = 0.5f;             // Safety check: AH diff must still be > this to exit WET

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

// Saturation recovery parameters
constexpr unsigned long PID_SAT_DETECT_MS = 15000;   // 15s continuously at ~max duty triggers recovery
constexpr double PID_SAT_DUTY_THRESH = 0.98;         // consider saturated when output >= 98% (and duty ~100%)
constexpr double PID_SAT_ERR_THRESH = 0.12;          // if setpoint - measured rate > 0.12, treat as not achievable
constexpr double PID_SAT_MARGIN = 0.05;              // reduce setpoint to measured + margin during recovery

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
