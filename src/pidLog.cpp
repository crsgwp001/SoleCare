#include "pidLog.h"

#if PID_LOGGING_ENABLED

#include <Arduino.h>
#include <cstdio>
#include <cmath>
#include "config.h"

// State name lookup
static const char* getSubStateName(SubState s) {
  switch (s) {
    case SubState::S_IDLE: return "IDLE";
    case SubState::S_WAITING: return "WAIT";
    case SubState::S_WET: return "WET";
    case SubState::S_COOLING: return "COOL";
    case SubState::S_DRY: return "DRY";
    case SubState::S_DONE: return "DONE";
    default: return "UNKN";
  }
}

// Wet/Dry status based on AH diff threshold
static const char* getWetDryStatus(float ahDiff) {
  if (std::isnan(ahDiff)) return "UNK";  // Unknown if sensor not ready
  return (ahDiff >= AH_WET_THRESHOLD) ? "WET" : "DRY";
}

void pidLogInit() {
  // Print header for CSV logging (every 2s)
  Serial.println("time_ms,ah0,ah1,ah2,s0_temp,s1_temp,s0_diff,s0_wet,s0_state,s0_rate,s0_pid,s1_diff,s1_wet,s1_state,s1_rate,s1_pid");
}

void pidLogData(float ah0, float ah1, float ah2,
                float shoe0Temp, float shoe1Temp,
                float shoe0AHDiff, SubState shoe0State, float shoe0AHRate, double shoe0PIDOut,
                float shoe1AHDiff, SubState shoe1State, float shoe1AHRate, double shoe1PIDOut) {
  // Handle NaN values (sensor not ready) - show 0.000 instead of nan
  float s0Diff = std::isnan(shoe0AHDiff) ? 0.0f : shoe0AHDiff;
  float s1Diff = std::isnan(shoe1AHDiff) ? 0.0f : shoe1AHDiff;
  float s0Temp = std::isnan(shoe0Temp) ? 0.0f : shoe0Temp;
  float s1Temp = std::isnan(shoe1Temp) ? 0.0f : shoe1Temp;
  
  // CSV format: timestamp, 3 AH sensors, shoe0 (diff, wet/dry, state, rate, pid), shoe1 (diff, wet/dry, state, rate, pid)
  Serial.printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f,%.3f,%s,%s,%.4f,%.3f,%.3f,%s,%s,%.4f,%.3f\n",
                millis(),
                ah0, ah1, ah2,
                s0Temp, s1Temp,
                s0Diff, getWetDryStatus(shoe0AHDiff), getSubStateName(shoe0State), shoe0AHRate, shoe0PIDOut,
                s1Diff, getWetDryStatus(shoe1AHDiff), getSubStateName(shoe1State), shoe1AHRate, shoe1PIDOut);
}

#endif
