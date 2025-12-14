// UV relay control API
#pragma once
#include <stdbool.h>
#include <stdint.h>

bool uvInit();
bool uvStart(uint8_t idx, uint32_t durationMs);
bool uvStop(uint8_t idx);
bool uvTimerFinished(uint8_t idx);
bool uvIsStarted(uint8_t idx);
// Return remaining UV time in milliseconds, or 0 if not active
uint32_t uvRemainingMs(uint8_t idx);
