#pragma once

#include <cstdint>

// Global cycle timing
extern uint32_t g_cycleStartMs;

void createOledTasks();
void triggerSplashAnimation();  // Called when reset button is pressed
void triggerSplashEntryOnly();  // Entry-only splash for reset
