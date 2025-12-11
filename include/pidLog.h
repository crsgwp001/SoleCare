#pragma once
#include "config.h"

#if PID_LOGGING_ENABLED

void pidLogInit();
void pidLogData(uint8_t shoeIdx, float ahRate, double pidSetpoint, 
                double pidOutput, int motorDutyPercent);

#else

// No-op stubs when logging disabled
inline void pidLogInit() {}
inline void pidLogData(uint8_t shoeIdx, float ahRate, double pidSetpoint, 
                       double pidOutput, int motorDutyPercent) {}

#endif
