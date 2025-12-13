# Testing & Validation Plan for v1.2.0

## Overview
This plan validates the critical fixes in v1.2.0: WET exit logic correction, watchdog timeout prevention, EMI protection, and conservative PID tuning.

---

## Priority 1: Stability Verification
**Goal**: Confirm all watchdog fixes and EMI protection work

### 1. Extended Runtime Test (2+ hours)
- [ ] Monitor for Core 1 panics/resets
- [ ] Check for watchdog timeouts
- [ ] Verify system stays stable during full dry cycles
- [ ] **Expected**: No crashes, smooth operation throughout

### 2. EMI Protection Validation
- [ ] Monitor `s0_rate` and `s1_rate` columns in logs
- [ ] **Expected**: No extreme values beyond ±120 g/m³/min
- [ ] **Expected**: Smooth rate curves, no spikes during motor operation
- [ ] **Compare**: Previous logs showed 83, 66, 53 g/m³/min spikes - should be gone
- [ ] **Look for**: Rate limiting messages if spikes are rejected

### 3. State Transition Verification
- [ ] WET should NOT exit prematurely (verify it runs >30s minimum)
- [ ] COOLING should complete without infinite loops
- [ ] WAITING should acquire lock without hanging
- [ ] Motor overlap should not occur (verify sequential operation)
- [ ] **Check**: `g_coolingEarlyExit` and `g_waitingEventPosted` guards working

---

## Priority 2: WET Exit Behavior
**Goal**: Verify correct evaporation peak detection

### 4. WET Exit Timing Analysis
- [ ] Track when WET→COOLING transitions occur
- [ ] **Expected**: "SUB1: WET AH rate=..." debug messages showing declining rates
- [ ] **Expected**: 2 consecutive negative rate changes before exit
- [ ] **Expected**: WET phase lasts 30-180s (not 5-10s like before)
- [ ] Note actual WET duration from logs

### 5. Rate-of-Change Validation
- [ ] Check if rate values make physical sense (should be 0-10 g/m³/min during active drying)
- [ ] Verify consecutive negative tracking works correctly
- [ ] Look for false positives (shouldn't exit on single noise sample)
- [ ] **Debug output**: Should show "Negative rate change detected, consecutive count: 1" then "2"

---

## Priority 3: PID Performance
**Goal**: Evaluate if conservative tuning improved control

### 6. PID Response Analysis
- [ ] Monitor `s0_pid` column for duty cycle progression
- [ ] **Expected**: Smooth ramp 50%→70%→85% (NOT 50%→95% jumps)
- [ ] **Expected**: Less time at 100% saturation
- [ ] Check if rate oscillates around 0.4 ±0.2 g/m³/min
- [ ] Compare to baseline log (monitor_20251212_203711.log)

### 7. Saturation Check
- [ ] Count how long PID stays at 95-100% (should be reduced vs baseline)
- [ ] Calculate average duty cycle during control phase
- [ ] **If still saturating heavily**: Consider increasing TARGET_AH_RATE to 0.6-0.8
- [ ] **If oscillating**: May need to reduce Kp further or increase Kd

---

## Data Collection Requirements

### What to Capture:
1. **Full CSV log** of at least one complete dual-shoe cycle
2. **Timestamps** of key events:
   - WET entry (both shoes)
   - Motor start (heater warmup complete)
   - WET exit → COOLING transition
   - COOLING phases (motor + stabilization)
   - DRY entry
   - UV start/completion
3. **Screenshot/note** any error messages or unexpected behavior
4. **CPU usage/temperature** if monitoring available

### Log Analysis Checklist:
- [ ] No "Guru Meditation Error" messages
- [ ] No repeated state entries (infinite loops)
- [ ] Rate values stay bounded (±120 max)
- [ ] PID output progression is smooth
- [ ] Both shoes complete full cycle

---

## Success Criteria

### Must Pass:
✅ No crashes/resets during 2+ hour run  
✅ Rate values stay within ±120 g/m³/min (no EMI spikes)  
✅ WET exits based on declining rates (not premature)  
✅ PID shows smoother progression (reduced saturation)  
✅ Sequential motor operation (no overlap)  

### Nice to Have:
✅ PID averages 70-85% duty (not constantly at 100%)  
✅ WET phase duration matches actual drying curve (varies by load)  
✅ COOLING early-exit optimization triggers when applicable  

---

## Troubleshooting Guide

### If System Still Crashes:
1. Check which protection layer failed (rate clamping, spike rejection, etc.)
2. May need tighter limits:
   - Reduce MAX_AH_DELTA_PER_SAMPLE from 2.0 to 1.5
   - Reduce rate clamp from ±120 to ±100
3. Check if crash happens during specific state (add more guards)

### If WET Exits Too Early:
1. Increase MIN_CONSECUTIVE_NEGATIVE from 2 to 3
2. Adjust AH_RATE_DECLINE_THRESHOLD (currently -0.01)
3. Increase AH_ACCEL_WARMUP_MS if exiting before 30s

### If WET Never Exits:
1. Check rate calculations in debug output
2. Verify g_dhtAHRate[] is being populated correctly
3. May need to relax consecutive negative requirement
4. Check if rate is actually declining (might need longer runtime)

### If PID Still Saturating:
1. Increase TARGET_AH_RATE from 0.4 to 0.6-0.8
2. Reduce Kp further (try 0.08 or 0.05)
3. Check if heater/motor combo is underpowered for load
4. May need Phase 2 dual-setpoint implementation

### If EMI Spikes Still Present:
1. Check if rate limiting is active (should reject spikes)
2. May need to increase 5ms delays to 10ms
3. Check hardware: sensor wiring shielding, ground loops
4. Consider hardware filters (capacitors on sensor lines)

---

## Notes

**Baseline Reference**: `logs/monitor_20251212_203711.log`  
- Shows PID saturation at 95-100% for 130 seconds
- Rate never reached setpoint (0.4)
- Use this to compare improvement

**Known Fixed Issues**:
- Line 151 premature WET exit (using diff instead of rate)
- Lines 191-226 extreme rate spikes from EMI (-83, +60 g/m³/min)
- Multiple watchdog timeouts in COOLING and WAITING states

**Version**: v1.2.0  
**Date**: 2025-12-12  
**Next Review**: After initial test run completion
