// UV relay control API
#pragma once
#include <stdint.h>
#include <stdbool.h>

bool uvInit();                    // Create UV task/queue
bool uvStart(uint8_t idx, uint32_t durationMs); // Start UV for idx
bool uvStop(uint8_t idx);        // Stop UV for idx
bool uvTimerFinished(uint8_t idx);// True if UV for idx is not running
bool uvIsStarted(uint8_t idx);   // True if UV is currently active
