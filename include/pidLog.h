#pragma once
#include "config.h"
#include <events.h>

#if PID_LOGGING_ENABLED

void pidLogInit();
void pidLogData(float ah0, float ah1, float ah2,
                float shoe0Temp, float shoe1Temp,
                float shoe0AHDiff, SubState shoe0State, float shoe0AHRate, double shoe0PIDOut, double shoe0Setpoint,
                float shoe1AHDiff, SubState shoe1State, float shoe1AHRate, double shoe1PIDOut, double shoe1Setpoint);

#else

// No-op stubs when logging disabled
inline void pidLogInit() {}
inline void pidLogData(float ah0, float ah1, float ah2,
                       float shoe0Temp, float shoe1Temp,
                       float shoe0AHDiff, SubState shoe0State, float shoe0AHRate, double shoe0PIDOut, double shoe0Setpoint,
                       float shoe1AHDiff, SubState shoe1State, float shoe1AHRate, double shoe1PIDOut, double shoe1Setpoint) {}

#endif
