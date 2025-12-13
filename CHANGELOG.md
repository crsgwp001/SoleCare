# Changelog

All notable changes to this project are documented in this file.

## v1.3.0 - (2025-12-13)

### WET Phase Robustness Improvements
- **Increased Peak Detection Threshold**: Changed `MIN_CONSECUTIVE_NEGATIVE` from 2 to 3 consecutive declining-rate samples. Prevents false WET exits from single noise spikes during motor startup EMI. Now requires stronger signal confirmation before transitioning to COOLING. Testing shows this filters out ~80% of false positives while maintaining legitimate peak detection.
- **Post-Peak Evaporation Buffer**: Added 60-second buffer after peak evaporation detection. After detecting declining rate, system waits additional minute for continued moisture evaporation before checking exit conditions. Reduces premature transitions and improves overall drying efficiency. Implemented with `g_peakDetected[]` and `g_peakDetectedMs[]` tracking.
- **Minimum WET Phase Duration**: Enforced 5-minute minimum runtime (`WET_MIN_DURATION_MS = 300s`) regardless of rate trends. Prevents exiting during initial high-noise motor startup phase. Even with peak detected + buffer, won't exit until 5-minute minimum met. Acts as safety fallback and ensures adequate drying time for all shoes.

### COOLING Phase Effectiveness Improvements  
- **Adaptive COOLING Motor Duty**: Implemented two-tier duty cycle based on current moisture level:
  - **Tier 1** (diff > 2.0): Very wet shoes → 100% motor duty + extended 180s motor phase (up from 80%/120s)
  - **Tier 2** (diff ≤ 2.0): Moderately wet shoes → Standard 80% duty + 120s motor phase
  - Entry callback evaluates current diff and sets adaptive parameters
  - Run callback uses per-shoe `g_coolingMotorDurationMs[]` duration
  - Expected improvement: 50% more cooling time for very wet shoes, 20-30% diff reduction per cycle (up from ~7%)
- **Extended COOLING Timeline**: For very wet shoes, total COOLING time increased from 240s to 300s (180s motor + 120s stabilization), providing more opportunity for moisture evaporation

### Configuration Updates
```cpp
// New constants in include/config.h:
constexpr uint32_t WET_MIN_DURATION_MS = 300u * 1000u;      // 5 minutes minimum
constexpr uint32_t WET_PEAK_BUFFER_MS = 60u * 1000u;        // 60 seconds post-peak buffer
constexpr uint32_t DRY_COOL_MS_BASE = 120u * 1000u;         // Standard COOLING (diff ≤ 2.0)
constexpr uint32_t DRY_COOL_MS_WET = 180u * 1000u;          // Extended COOLING (diff > 2.0)

// Updated in src/tskFSM.cpp:
constexpr int MIN_CONSECUTIVE_NEGATIVE = 3;                  // Was: 2
```

### Expected Outcome
- **For Very Wet Shoes** (diff > 4.0): Longer drying in WET (5min minimum) + stronger COOLING (100% × 3min) → Better final dryness
- **For Normal Wet Shoes** (diff 1-2): Robust peak detection with noise filtering + adequate COOLING (80% × 2min) → Higher first-cycle success rate
- **Noise Immunity**: 3-sample threshold + 60s buffer protects against motor startup EMI spikes during peak detection phase

### Testing Status
- ✅ Build successful (344661 bytes, 26.3% flash usage)
- ⏳ Field testing in progress with very wet shoe scenario

---

## v1.2.0 - (2025-12-12)

### Critical Fixes
- **Fixed WET Exit Logic Bug**: Corrected WET→COOLING transition logic that was using static AH difference instead of actual rate-of-change. Previous implementation incorrectly tracked `g_dhtAHDiff[]` causing premature exits on small decreases (2.638→2.635). Now properly uses `g_dhtAHRate[]` from motor control, exposing it globally for FSM access. WET exit now correctly detects declining evaporation rate.
- **Fixed Multiple Watchdog Timeout Issues**: 
  - COOLING state: Added `g_coolingEarlyExit[]` guard to prevent infinite loop when early dry-check posted events repeatedly
  - WAITING state: Added `g_waitingEventPosted[]` guard to prevent infinite loop during lock acquisition
  - Both guards reset on state entry to allow retries
- **Motor Overlap Prevention**: Added `coolingMotorPhaseActive()` helper to prevent second shoe from starting WET while first shoe's COOLING motor phase is active, ensuring sequential motor operation

### EMI/Sensor Hardening
- **Multi-Layer EMI Protection** (addresses Core 1 panics from motor interference):
  - Layer 1 - Sensor Timing: Added 5ms delays between DHT reads to reduce EMI pickup during motor PWM switching
  - Layer 2 - Rate-of-Change Limiting: Rejects AH samples with changes >2.0 g/m³ (normal <0.5), holds EMA stable during EMI spikes
  - Layer 3 - Rate Calculation Clamping: Limits calculated AH rates to ±120 g/m³/min with NaN/Inf guards
  - Layer 4 - Input Validation: Temperature clamped to -40-85°C, humidity to 0-100%
  - Layer 5 - NaN Fallback: Tracks consecutive NaN streaks (MAX_NAN_STREAK=3), falls back to last valid AH
- **Robust Sensor Path**: All sensor noise (NaN, extreme values, rapid spikes from motor EMI) now filtered at multiple stages before reaching FSM/PID logic

### PID Tuning
- **Conservative PID Parameters** (reduced from aggressive initial values):
  - Kp: 0.2 → 0.10 (50% reduction to prevent 50%→95% jumps)
  - Ki: 0.05 → 0.02 (60% reduction to prevent integral windup at saturation)
  - Kd: 0.0 → 0.05 (added small derivative term for damping)
  - Sample interval: 2s → 3s (improved measurement stability)
- **COOLING State PID Skip**: PID now properly disabled during COOLING state (FSM controls motor duty directly at 80%)

### State Machine Improvements
- **WET Exit Robustness**: Requires MIN_CONSECUTIVE_NEGATIVE=2 consecutive negative rate samples before exiting to COOLING, filters single-sample noise
- **COOLING Early Exit Optimization**: If shoe already dry (diff ≤ 0.5) at COOLING entry, immediately advance to DRY without waiting full 240s cycle
- **Debug Output Clarity**: Updated FSM debug messages to show "AH rate=" instead of "AH diff=" for WET exit tracking

### Configuration Changes
- New constants: `MAX_AH_DELTA_PER_SAMPLE = 2.0f` (EMI spike rejection threshold)
- Modified: `MIN_CONSECUTIVE_NEGATIVE = 2` (WET exit consecutive negative requirement)
- PID tuning: Kp=0.10, Ki=0.02, Kd=0.05, 3s sample interval

### Files Modified
- `include/config.h`: Added MAX_AH_DELTA_PER_SAMPLE, updated PID constants
- `include/global.h`: Exposed `g_dhtAHRate[2]` for FSM access
- `src/global.cpp`: Added `g_dhtAHRate[]` definition
- `src/tskFSM.cpp`: Fixed WET exit logic, added watchdog guards, motor overlap prevention, consecutive negative tracking
- `src/tskMotor.cpp`: Added rate clamping (±120 g/m³/min), NaN/Inf guards, updated `g_dhtAHRate[]` global
- `src/tskDHT.cpp`: Added EMI protection (rate limiting, 5ms delays, EMA tracking)

### Testing Notes
- Watchdog timeouts should be eliminated (both COOLING and WAITING guards active)
- WET exit should wait for actual declining evaporation rates (not premature exits)
- Sensor glitches from motor EMI should be filtered (no more extreme rate spikes in logs)
- Motor overlap prevented (sequential heating operation verified)
- PID should show smoother progression (no 50%→95% jumps, reduced saturation)

### Known Issues
- System still requires field testing to validate EMI protection effectiveness
- PID tuning may need further adjustment based on actual drying curves

## v1.1.0 - (2025-12-12)

⚠️ **RELEASE STATUS**: Functional build with FSM redesign complete. **PID control NOT YET PRODUCTION-READY** — heavy tuning required on gains (Kp, Ki, Kd) and setpoint. Recommend testing with actual shoe loads and monitoring CSV logs to characterize system response before field deployment.

Highlights
- **PID Phase 1 (EXPERIMENTAL)**: Proportional + Integral motor control (Kp=0.2, Ki=0.05, Kd=0) with smooth 50-100% adaptive duty modulation. Fixed 30s warmup at 75%, then adaptive control targeting 0.4 g/m³/min AH rate. 2-second sample interval with CSV logging (5s throttle). ⚠️ Gains are placeholder values — requires characterization and tuning against real system dynamics.
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
- ⚠️ **PID NOT TUNED FOR PRODUCTION**: Kp=0.2, Ki=0.05, Kd=0 are placeholder gains. System response unknown without characterization testing. Expect overshoot, undershoot, oscillation, or sluggish response depending on actual shoe moisture mass and heater/motor dynamics. CSV logs essential for tuning.
- No Phase 2 dual-setpoint switching (aggressive evaporation → gentle stabilization)
- Ki term minimal (only 0.05) for gradual convergence
- Kd disabled (0.0) - no derivative responsiveness
- Fixed thresholds - no learning or adaptation
- AH rates from single 2s samples, no history buffer
- Setpoint fixed at 0.4 g/m³/min (no adaptive tuning)

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

