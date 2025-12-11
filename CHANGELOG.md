# Changelog

All notable changes to this project are documented in this file.

## v1.1.0 - (2025-12-12)

Highlights
- **PID Phase 1**: Proportional + Integral motor control (Kp=0.2, Ki=0.05, Kd=0) with smooth 50-100% adaptive duty modulation. Fixed 30s warmup at 75%, then adaptive control targeting 0.4 g/m³/min AH rate. 2-second sample interval with CSV logging (5s throttle).
- **Smart WET Exit**: Replaced rigid 5-second timeout with AH acceleration detection (peak evaporation). Subs now exit WET dynamically (30-600s range) based on actual moisture removal curve. Requires 30s warmup to prevent false positives.
- **Hysteresis Thresholds**: WET=1.0f, DRY=0.5f thresholds with 0.5 unit gap prevent state bouncing between WAITING/WET/COOLING. Eliminates rapid oscillation on borderline humidity.
- **FSM Redesign**: Global Idle fully resets hardware and substates. Done entry no longer pre-resets subs (they stay in final state until explicit reset). StartPressed gated to Idle/Checking only. Auto-reset broadcasts to global FSM only, not substates. Substate reset via ResetPressed event.
- **Lock Management**: WET lock released on WET exit (not WAITING entry). Sequential heater access: only 1 shoe heats at a time (g_wetLockOwner: -1=free, 0=sub1, 1=sub2).
- **Documentation**: Added comprehensive `FSM_ARCHITECTURE.md` explaining global/substate flows, WET lock mechanism, AH acceleration exit logic, timing chart, dual-shoe parallel example, and tuning parameters.

Configuration Changes
- New: `AH_ACCEL_WARMUP_MS = 30s`, `AH_WET_THRESHOLD = 1.0f`, `AH_DRY_THRESHOLD = 0.5f`
- PID: `PID_KP = 0.2`, `PID_KI = 0.05`, `PID_KD = 0.0`, `TARGET_AH_RATE = 0.4`
- Timing: `DRY_COOL_MS = 60s`, `DRY_STABILIZE_MS = 120s`, `MOTOR_SAFETY_MS = 600s`

Files Changed
- New: `include/pidLog.h`, `src/pidLog.cpp`, `FSM_ARCHITECTURE.md`
- Modified: `include/config.h`, `src/tskMotor.cpp`, `src/tskFSM.cpp`, `lib/PIDcontrol/src/PIDcontrol.cpp`

Testing checklist
1. Build succeeds (`platformio run`).
2. PID motor duty: Should smoothly modulate 50-100%, not jump. Watch CSV logs for control profile.
3. State transitions: No bouncing between WAITING/WET. COOLING should last ~180s total. WET exits adaptively (30-600s based on AH decline).
4. Lock sequencing: Only one shoe heats at a time. Second shoe waits in WAITING until first exits WET.
5. Reset behavior: From any state, pressing Start goes to Idle (resets everything). Done state doesn't auto-reset subs.
6. Button safety: Pressing Start in Detecting doesn't auto-advance (must be in Idle/Checking).
7. Dual-shoe parallel: Both shoes drying simultaneously in DRY/DONE while heater runs sequentially in WET.

Known Limitations (Phase 1)
- No Phase 2 dual-setpoint switching (aggressive evaporation → gentle stabilization)
- Ki term minimal (only 0.05) for gradual convergence
- Kd disabled (0.0) - no derivative responsiveness
- Fixed thresholds - no learning or adaptation
- AH rates from single 2s samples, no history buffer

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

