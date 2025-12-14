// Motor/heater control task API
#pragma once
#include <stdbool.h>
#include <stdint.h>

bool motorInit();
// Start control for shoe index (0 or 1). The task will monitor sensors and
// automatically advance the FSM by posting Event::SubStart when threshold met.
bool motorStart(uint8_t idx);
// Stop control for shoe index. Notifies the task of exit so it can cleanup.
bool motorStop(uint8_t idx);

// Low-level actuator control (thread-safe commands)
bool heaterRun(uint8_t idx, bool on);
// Adjust motor target duty cycle (percent 0-100)
bool motorSetDutyPercent(uint8_t idx, int percent);

// Timing helpers: return milliseconds since the shoe was started (0 if not active)
uint32_t motorActiveMs(uint8_t idx);

// Query whether motor/heater outputs are currently on
bool motorIsOn(uint8_t idx);
bool heaterIsOn(uint8_t idx);

// Query if the motor task is active for index
bool motorIsActive(uint8_t idx);
// Query current motor duty cycle percentage (0-100)
int getMotorDutyCycle(uint8_t idx);