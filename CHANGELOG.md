# Changelog

All notable changes to this project are documented in this file.

## v0.1.9 - (2025-09-23)

Highlights
- Sensor: Added absolute-humidity (AH) computation, NaN-safe EMA seeding, and exposure of AH and AH difference to the UI.
- UV: Moved UV control into a dedicated FreeRTOS task with start/stop/pause/resume and remaining-time semantics; UV timers post events to the FSM to drive deterministic progression.
- Motor/Heater: Implemented motor/heater task with safety timeout (enforced by MOTOR_SAFETY_MS) and added status getters for UI introspection.
- FSM: Reworked per-sub deterministic flow to WET -> COOLING -> DRY -> DONE, added per-sub timestamps, and handled UV expiry during COOLING robustly.
- UI & I2C: Hardened SSD1306 usage with a FreeRTOS mutex to avoid I2C/display concurrency panics; UI shows AH/ΔAH and compact actuator/UV indicators.
- Repo & tooling: Added `docs/CODE_STYLE.md`, `.clang-format`, and performed a repository-wide tidy (includes order, formatting). Added and then removed an optional clang-format workflow per user request; CI currently not enforcing formatting.

Notes
- Build: PlatformIO (ESP32, Arduino framework)
- Release: tag `v0.1.9` published on GitHub

Testing checklist
1. Build and flash firmware to the ESP32 via PlatformIO.
2. Verify sensors report AH (EMA) values and diffs; OLED2 displays AH and ΔAH for quick inspection.
3. Start a run: subs should progress WET -> COOLING -> DRY -> DONE. UV timers will be used to advance DRY -> DONE when appropriate.
4. Confirm motor safety timeout behavior and that motors/heaters are stopped when their runtime exceeds MOTOR_SAFETY_MS.
5. If you see I2C or SSD1306 panics, ensure no other tasks access the display without taking the display mutex.

