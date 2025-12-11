#include "pidLog.h"

#if PID_LOGGING_ENABLED

#include <Arduino.h>
#include <cstdio>

void pidLogInit() {
  // Print header for CSV logging
  Serial.println("timestamp_ms,shoe_idx,ah_rate,pid_setpoint,pid_output,motor_duty_pct");
}

void pidLogData(uint8_t shoeIdx, float ahRate, double pidSetpoint, 
                double pidOutput, int motorDutyPercent) {
  // CSV format for easy plotting in Excel/Python
  Serial.printf("%lu,%d,%.4f,%.4f,%.4f,%d\n",
                millis(), shoeIdx, ahRate, pidSetpoint, pidOutput, motorDutyPercent);
}

#endif
