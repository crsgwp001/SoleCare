# Changelog

All notable changes to this project are documented in this file.

## v0.1.7 - (2025-09-20)

Highlights
- Moved hardware configuration (pins, relay polarity, UV defaults, DHT pins, buttons) into `include/config.h` for easy customization.
- Made the Global Done auto-reset timeout configurable via `DONE_TIMEOUT_MS` in `include/config.h` (default 60s).
- Fixed Global->Done auto-reset bug so the timer is set on Done entry and cleared on exit.
- Implemented/cleaned UV task integration: UVs run in their own task and post timer events to the FSM.
- Added absolute-humidity calculation and NaN-safe EMA seeding for DHT sensors; exposed sensor results to UI.
- Performed a wide code tidy across headers and source files; small refactors for clarity and robustness.

Notes
- Build: PlatformIO (ESP32, Arduino framework)
- Flash: `platformio run --target upload`
- Serial monitor: `platformio device monitor -b 115200`

Testing checklist
1. Flash firmware to ESP32 using PlatformIO.
2. Monitor serial at 115200 for FSM and sensor logs (enable debug macros if you need more verbose output).
3. Press the Start button (configured in `include/config.h`) and observe Detecting → Checking → Running.
4. Confirm UV relay toggles when subs enter COOLING/DRY; UV expiry posts `UVTimer0/UVTimer1` and advances subs from `S_DRY`.
5. When both subs reach DONE, Global should enter `Done` and automatically reset to `Idle` after `DONE_TIMEOUT_MS`.


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

