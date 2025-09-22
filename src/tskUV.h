// UV relay control API
#pragma once
#include <stdint.h>
#include <stdbool.h>

bool uvInit();
bool uvStart(uint8_t idx, uint32_t durationMs);
bool uvStop(uint8_t idx);
bool uvTimerFinished(uint8_t idx);
bool uvIsStarted(uint8_t idx);
// Pause UV timer (turn relay off but keep remaining time). Returns true if paused.
bool uvPause(uint8_t idx);
// Resume a previously paused UV timer. Returns true if resumed.
bool uvResume(uint8_t idx);
// Query whether a UV timer is paused
bool uvIsPaused(uint8_t idx);
// Return remaining UV time in milliseconds, or 0 if not active
uint32_t uvRemainingMs(uint8_t idx);
