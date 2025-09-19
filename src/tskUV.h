// UV relay control API
#pragma once
#include <stdint.h>
#include <stdbool.h>

bool uvInit();
bool uvStart(uint8_t idx, uint32_t durationMs);
bool uvStop(uint8_t idx);
bool uvTimerFinished(uint8_t idx);
bool uvIsStarted(uint8_t idx);
