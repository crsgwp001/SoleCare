# Changelog

All notable changes to this project are documented in this file.

## v0.1.12-rc1 - (2025-12-10)

Highlights
- UV: Single-channel UV now runs on GPIO14 with PWM MOSFET drive; starts only when both shoes reach DRY and the UV timer advances both subs to DONE. UVTimer1 events are ignored in single-UV mode.
- FSM & battery: LowBattery keeps the error LED solid and prevents other LED logic from overriding it; DRY entry no longer pre-advances and waits for the UV timer to complete.
- UI: OLED1 message shortened to avoid wrap artifacts and now shows a single UV indicator; OLED2 unchanged for AH/ΔAH status.
- Hardware control: Heaters now use relay control (active-high by default), motors remain PWM-driven; hardware config centralized and tidied in `include/config.h`.
- Tooling: Added VS Code `tasks.json` for running PlatformIO builds locally.

Testing checklist
1. PlatformIO build succeeds (`platformio run`).
2. Run a full drying cycle: UV should start only after both shoes are DRY and run the full default duration (10s), advancing both subs to DONE.
3. Trigger LowBattery: error LED should remain solid until recovery; status LED stays off.
4. Verify OLED1 displays compact lines (no wrapping) and shows a single UV timer/paused state; OLED2 continues to show AH/ΔAH and shoe wet/dry state.

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

