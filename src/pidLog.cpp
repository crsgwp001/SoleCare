#include "pidLog.h"

#if PID_LOGGING_ENABLED

#include <Arduino.h>
#include <cstdio>

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

void pidLogInit() {
  // Print header for CSV logging (every 2s)
  Serial.println("time_ms,ah0,ah1,ah2,s0_diff,s0_state,s0_rate,s0_pid,s1_diff,s1_state,s1_rate,s1_pid");
}

void pidLogData(float ah0, float ah1, float ah2,
                float shoe0AHDiff, SubState shoe0State, float shoe0AHRate, double shoe0PIDOut,
                float shoe1AHDiff, SubState shoe1State, float shoe1AHRate, double shoe1PIDOut) {
  // CSV format: timestamp, 3 AH sensors, shoe0 (diff, state, rate, pid), shoe1 (diff, state, rate, pid)
  Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%s,%.4f,%.3f,%.3f,%s,%.4f,%.3f\n",
                millis(),
                ah0, ah1, ah2,
                shoe0AHDiff, getSubStateName(shoe0State), shoe0AHRate, shoe0PIDOut,
                shoe1AHDiff, getSubStateName(shoe1State), shoe1AHRate, shoe1PIDOut);
}

#endif
