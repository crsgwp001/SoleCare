# PID Motor Control Development Roadmap

## Phase 1: Complete ✅
**P-Only Adaptive AH Rate Control with Logging**

### Implementation Details
- **Tuning**: Kp=5.0, Ki=0, Kd=0 (conservative, P-only)
- **Sample Rate**: 2 seconds
- **Control Strategy**: Single setpoint (TARGET_AH_RATE = 0.4 g/m³/min)
- **Timeline**:
  - 0-30s: Fixed 75% motor duty (heater warmup + initial stabilization)
  - 30s+: PID takes control, adapts 30-100% based on AH rate
- **Per-Shoe**: Independent PID instances, auto-reset on S_WET exit
- **Logging**: CSV format (timestamp, shoe_idx, ah_rate, pid_setpoint, pid_output, motor_duty_pct)

### Testing Checklist
- [ ] Run full drying cycle with logging enabled
- [ ] Plot AH rate vs motor duty over time
- [ ] Observe control behavior (does motor adjust smoothly?)
- [ ] Check for oscillation (should be smooth curve, not hunting)
- [ ] Test both shoes independently and simultaneously
- [ ] Record temperature, RH, AH profiles for analysis

### Known Limitations
- Single setpoint can't handle evaporation phase (AH rising) vs stabilization phase (AH flattening)
- May over-correct in early drying when AH naturally spikes
- Fixed 75% warmup may be suboptimal for different shoe materials

---

## Phase 2: Dual-Phase Setpoint Switching (Ready for Implementation)

### Objective
Dynamically switch PID target based on detected drying phase to optimize both speed and control quality.

### Architecture
Two distinct control modes based on AH rate-of-change detection:

#### **Phase 2A: Aggressive Evaporation** (AH Rising)
```
Detection: ahRate > AH_RATE_PHASE_THRESHOLD (+0.1 g/m³/min)
Setpoint: TARGET_AH_RATE_EVAP = +0.4 g/m³/min
Goal: Maintain fast moisture extraction
PID Response: Reduce motor if evaporation too fast, increase if too slow
```

#### **Phase 2B: Gentle Stabilization** (AH Flattening/Falling)
```
Detection: ahRate < AH_RATE_PHASE_THRESHOLD (crossing downward)
Setpoint: TARGET_AH_RATE_STABLE = -0.1 g/m³/min
Goal: Gentle controlled descent, avoid overshooting dry
PID Response: Reduce motor to maintain slow, controlled drying
```

### Implementation Steps

1. **Uncomment Phase 2 constants in `include/config.h`**:
   ```cpp
   // Uncomment and modify when ready to implement Phase 2
   constexpr double TARGET_AH_RATE_EVAP = 0.4;    // Phase 2A: Aggressive evaporation
   constexpr double TARGET_AH_RATE_STABLE = -0.1; // Phase 2B: Gentle stabilization
   constexpr double AH_RATE_PHASE_THRESHOLD = 0.1; // Threshold to switch phases
   ```

2. **Add phase tracking globals in `src/tskMotor.cpp`**:
   ```cpp
   enum class DryingPhase { EVAPORATION = 0, STABILIZATION = 1 };
   static DryingPhase g_dryingPhase[2] = {DryingPhase::EVAPORATION, DryingPhase::EVAPORATION};
   ```

3. **Create phase detection function in `src/tskMotor.cpp`**:
   ```cpp
   static void updateDryingPhase(uint8_t idx, float ahRate) {
     DryingPhase &phase = g_dryingPhase[idx];
     
     if (phase == DryingPhase::EVAPORATION && ahRate < AH_RATE_PHASE_THRESHOLD) {
       // Transition from evaporation to stabilization
       phase = DryingPhase::STABILIZATION;
       g_motorPID[idx].setSetpoint(TARGET_AH_RATE_STABLE);
       DEV_DBG_PRINT("PID: switched to STABILIZATION for shoe ");
       DEV_DBG_PRINTLN(idx);
     }
     else if (phase == DryingPhase::STABILIZATION && ahRate > AH_RATE_PHASE_THRESHOLD) {
       // Transitioned back to evaporation (unexpected, but possible)
       phase = DryingPhase::EVAPORATION;
       g_motorPID[idx].setSetpoint(TARGET_AH_RATE_EVAP);
       DEV_DBG_PRINT("PID: switched back to EVAPORATION for shoe ");
       DEV_DBG_PRINTLN(idx);
     }
   }
   ```

4. **Integrate phase detection in motor PID loop** (in `src/tskMotor.cpp`, PID control section):
   ```cpp
   // Calculate current AH rate
   float ahRate = calculateAHRate(i);
   
   // Update drying phase and switch setpoint if needed
   updateDryingPhase(i, ahRate);
   
   // Compute PID with current (possibly switched) setpoint
   double pidOutput = g_motorPID[i].compute(ahRate);
   ```

5. **Reset phase on S_WET entry** (in `src/tskFSM.cpp`, S_WET entry callbacks):
   ```cpp
   // Add to both SUB1 and SUB2 S_WET entry lambdas:
   g_dryingPhase[0] = DryingPhase::EVAPORATION;  // Reset to evaporation phase
   ```

### Expected Behavior Timeline
```
S_WET Entry:
  Time 0-5s:     Heater warmup (motor 0%)
  Time 5-30s:    Fixed 75% duty (initial phase)
  Time 30-60s:   Phase 2A (Evaporation) - PID with +0.4 target
                 AH rising, motor adjusting up/down to match rate
  Time 60-180s:  Phase 2B (Stabilization) - PID with -0.1 target
                 AH flattening/falling, motor backs off to gentle control
  Time 180s+:    Safety timeout or dry detection → S_COOLING
```

### Tuning Considerations

#### Conservative Start (Safest)
```cpp
// Phase 2A: Match evaporation rate
TARGET_AH_RATE_EVAP = 0.4;      // Same as Phase 1
AH_RATE_PHASE_THRESHOLD = 0.1;  // Gentle transition

// Phase 2B: Very gentle decline
TARGET_AH_RATE_STABLE = -0.05;  // Slow descent, less aggressive
```

#### Aggressive (Faster Drying)
```cpp
// Phase 2A: Faster evaporation
TARGET_AH_RATE_EVAP = 0.6;      // Push evaporation harder
AH_RATE_PHASE_THRESHOLD = 0.15; // Faster transition

// Phase 2B: Moderate stabilization
TARGET_AH_RATE_STABLE = -0.15;  // Faster controlled descent
```

### Testing Phase 2

1. **Enable Phase 2 in config.h** (uncomment constants)
2. **Uncomment Phase 2 implementation** (helper functions, phase tracking)
3. **Build and test** with logging enabled
4. **Observe behavior**:
   - Does phase switch happen at expected rate?
   - Is motor behavior smoother in each phase?
   - Any oscillation or control instability?
5. **Tune setpoints** based on observed behavior:
   - If Evaporation phase too aggressive: decrease TARGET_AH_RATE_EVAP
   - If Stabilization phase too slow: increase TARGET_AH_RATE_STABLE magnitude
   - If phase switches prematurely: adjust AH_RATE_PHASE_THRESHOLD

---

## Phase 3: Add Integral & Derivative Terms (Future)

### When to Implement
After Phase 2 is stable and well-tuned over many test cycles.

### Rationale
- **Integral (Ki)**: Eliminates steady-state error if PID never quite reaches setpoint
- **Derivative (Kd)**: Reduces overshoot if system exhibits ringing after setpoint changes

### Implementation
```cpp
// In config.h, gradually enable:
constexpr double PID_KI = 0.1;  // Start very small (anti-windup active)
constexpr double PID_KD = 0.5;  // Add sparingly (can amplify noise)
```

### Tuning Sequence
1. Increase Ki first (by 0.05-0.1 increments)
2. Observe steady-state settling
3. Add Kd only if overshooting (in 0.2-0.5 increments)
4. Watch for high-frequency noise amplification

---

## Phase 4: Full Rate History Buffer (Future)

### Current Implementation
- Tracks only last 2 AH samples (simple, low memory)
- AH rate calculated as: (current_AH - last_AH) / time_delta

### Future Enhancement
Ring buffer with 10+ samples for smoother rate calculation:
```cpp
#define AH_HISTORY_SIZE 10
static float g_ahHistory[2][AH_HISTORY_SIZE];
static unsigned long g_ahHistoryTime[2][AH_HISTORY_SIZE];
static uint8_t g_ahHistoryIdx[2];
```

**Benefits:**
- Filters out sensor noise spikes
- More stable setpoint switching
- Better derivative approximation if Kd added

**Trade-offs:**
- More RAM usage
- ~20-50 extra bytes per shoe
- Slightly more complex rate calculation (linear regression instead of delta)

---

## Long-Term Considerations

### Material-Specific Tuning
Different shoe materials dry at different rates:
- **Fast-drying** (thin, porous): Use higher TARGET_AH_RATE values
- **Slow-drying** (dense, waterproof): Use lower values or longer warmup
- **Solution**: Create material presets in config or EEPROM

### Adaptive Warmup Duration
Currently fixed at 30s. Could be:
- Tied to initial AH spike detection
- Based on material type
- Extended if AH doesn't rise initially

### Saturation Recovery
If motor hits 100% duty and still can't meet setpoint:
- Log warning
- Extend drying cycle time
- Or accept sub-optimal drying

### Environmental Compensation
Absolute humidity varies with ambient conditions:
- Dry day (20% RH): AH=~4 g/m³
- Humid day (70% RH): AH=~16 g/m³
- Could scale setpoints based on ambient AH

### UV Timer Interaction
Currently UV fires based on DRY state threshold. With PID:
- Motor may still be adjusting when DRY threshold hit
- Consider adding "final stabilization" phase after DRY detection
- Or delay UV until motor has settled for N seconds

---

## Config File Reference

### Phase 1 Active
```cpp
#define PID_LOGGING_ENABLED 1
constexpr double PID_KP = 5.0;
constexpr double PID_KI = 0.0;
constexpr double PID_KD = 0.0;
constexpr double TARGET_AH_RATE = 0.4;
constexpr unsigned long PID_CONTROL_START_MS = 30000;
constexpr int PID_FIXED_DUTY_PERCENT = 75;
```

### Phase 2 Ready (Commented)
```cpp
// constexpr double TARGET_AH_RATE_EVAP = 0.4;
// constexpr double TARGET_AH_RATE_STABLE = -0.1;
// constexpr double AH_RATE_PHASE_THRESHOLD = 0.1;
```

---

## Files Involved

| Phase | File | Changes |
|-------|------|---------|
| 1 | `include/config.h` | PID constants, fixed setpoint |
| 1 | `include/pidLog.h` | Logging interface |
| 1 | `src/pidLog.cpp` | CSV logging implementation |
| 1 | `src/tskMotor.cpp` | PID init, AH rate calc, control loop |
| 1 | `src/tskFSM.cpp` | PID reset on S_WET exit |
| 2 | `include/config.h` | Uncomment Phase 2 constants |
| 2 | `src/tskMotor.cpp` | Add phase tracking, phase detection, switch setpoint |
| 2 | `src/tskFSM.cpp` | Reset phase on S_WET entry |
| 3 | `include/config.h` | Uncomment Ki, Kd tuning |
| 4 | `src/tskMotor.cpp` | Ring buffer for AH history |

---

## Testing Protocol

### Pre-Phase 2
1. ✅ Phase 1 builds successfully
2. ✅ Logging outputs valid CSV data
3. ✅ Motor adjusts duty in response to AH rate
4. Test: Single wet shoe, single dry shoe, both wet shoes
5. Document: AH profiles, motor behavior, timing

### Pre-Phase 3
6. Phase 2 switches phases smoothly
7. No oscillation or instability
8. Drying times are acceptable (target: <5 min per shoe at 30-35°C ambient)

### Pre-Phase 4
9. Phase 1+2+3 stable over 10+ consecutive test cycles
10. Ki/Kd tuned and validated
11. Ready for rate history smoothing

---

## Next Immediate Steps

1. **Run Phase 1 tests** with current hardware
   - Capture serial logs (→ CSV file)
   - Plot results in Excel or Python
   - Verify motor responds to AH rate changes

2. **Tune PID_KP if needed**
   - If motor changes too slowly: increase Kp (try 7, 10)
   - If motor oscillates: decrease Kp (try 3, 2)

3. **Plan Phase 2 integration**
   - Decide on threshold values
   - Code phase detection logic
   - Test with logging to validate switching

4. **Document observed behavior**
   - Record which Kp value works best
   - Note any anomalies or edge cases
   - Plan material-specific tuning for future

