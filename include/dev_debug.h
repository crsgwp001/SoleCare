// Lightweight developer debug helper. Define DEV_DEBUG to enable prints.
#pragma once

#ifdef DEV_DEBUG
#include <Arduino.h>
// Serial may not be ready in early init; check Serial.availableForWrite as a
// lightweight readiness indicator before printing. Use do/while(0) to make
// these macros statement-safe.
#define DEV_DBG_PRINT(...) \
	do { if (Serial && Serial.availableForWrite()) Serial.print(__VA_ARGS__); } while (0)
#define DEV_DBG_PRINTLN(...) \
	do { if (Serial && Serial.availableForWrite()) Serial.println(__VA_ARGS__); } while (0)
#else
#define DEV_DBG_PRINT(...) ((void)0)
#define DEV_DBG_PRINTLN(...) ((void)0)
#endif
