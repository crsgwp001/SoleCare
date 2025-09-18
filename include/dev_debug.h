// Lightweight developer debug helper. Define DEV_DEBUG to enable prints.
#pragma once
#ifdef DEV_DEBUG
#include <Arduino.h>
#define DEV_DBG_PRINT(...) do { if (Serial) Serial.print(__VA_ARGS__); } while(0)
#define DEV_DBG_PRINTLN(...) do { if (Serial) Serial.println(__VA_ARGS__); } while(0)
#else
#define DEV_DBG_PRINT(...) ((void)0)
#define DEV_DBG_PRINTLN(...) ((void)0)
#endif
