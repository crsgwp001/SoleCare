# Additional Enhancements & Future Roadmap

This document outlines potential improvements beyond v1.3.0 to further optimize the drying system.

---

## Phase 2: Advanced Rate-of-Change Detection

### Problem Being Solved
Current implementation detects peak by rate-of-change, but doesn't account for:
- Natural oscillations at very low moisture levels
- Equilibrium effects when shoes reach natural drying limit
- Different drying curves for different shoe materials/sizes

### Option A: Moving Average Rate Detection
**Concept**: Instead of checking 3 consecutive samples, verify that rate is declining *on average*

```cpp
// Track last 5 rate samples in circular buffer
static float g_rateHistory[2][5];
static int g_rateHistoryIdx[2] = {0, 0};

// Calculate average rate over window
float avgRate = (g_rateHistory[0] + g_rateHistory[1] + g_rateHistory[2] + 
                 g_rateHistory[3] + g_rateHistory[4]) / 5.0f;
float avgPrevRate = (g_rateHistory_prev[0] + g_rateHistory_prev[1] + ... ) / 5.0f;

// Exit only if average rate declining AND samples stable (low variance)
if (avgRate < (avgPrevRate - 0.1) && variance < 0.05) {
  // Peak confirmed
}
```

**Pros**:
- Filters noise naturally
- Handles oscillations at low moisture levels
- More physically meaningful

**Cons**:
- Requires additional buffer (20 bytes per shoe)
- Slightly more complex logic

**Expected Improvement**: 
- Further reduce false positives
- Better for high-RH environments where evaporation slows significantly

---

### Option B: Equilibrium Detection
**Concept**: Exit WET when rate approaches environmental equilibrium (near-zero moisture removal)

```cpp
// If rate has been very low (<0.1 g/m³/min) for 3+ consecutive samples
// AND we've been running for minimum time
// = shoe likely at equilibrium with current RH, won't dry further
```

**Pros**:
- Scientifically accurate (stops when physical equilibrium reached)
- Adapts to ambient humidity
- No false positives from noise

**Cons**:
- Requires understanding of ambient RH
- Might exit too early if target RH very low

**Expected Improvement**:
- Perfect exit timing for different humidity conditions
- Applicable regardless of shoe initial wetness

---

## Phase 3: Dual-Setpoint PID with Adaptive Switching

### Problem Being Solved
Current PID runs one mode during WET. Better approach: switch setpoints mid-drying

### Implementation

**Phase 1 (Aggressive Evaporation)**: Target 0.5 g/m³/min moisture removal
- Higher PID gains
- Maximize air movement early while lots of water available
- Run for ~120 seconds or until rate drops below 0.3

**Phase 2 (Gentle Stabilization)**: Target 0.05 g/m³/min 
- Lower PID gains (avoid overshooting)
- Allow system to stabilize as shoe dries
- Run for remaining WET time

```cpp
if (wetElapsed < 120000 || currentRate > 0.3) {
  // Phase 1: Aggressive
  g_motorPID[idx].setSetpoint(0.5);
  g_motorPID[idx].setTuning(0.15, 0.03, 0.08);  // Higher gains
} else {
  // Phase 2: Gentle
  g_motorPID[idx].setSetpoint(0.05);
  g_motorPID[idx].setTuning(0.08, 0.01, 0.03);  // Lower gains
}
```

**Benefits**:
- Maximizes moisture removal when water abundant
- Stabilizes gracefully toward end
- Smoother power delivery
- Better energy efficiency

**Estimated Improvement**: 15-20% faster drying time

---

## Phase 4: Thermal Feedback Integration

### Problem Being Solved
Currently doesn't use temperature data. Temperature changes indicate:
- Evaporation cooling effect (endothermic process)
- Heater contribution
- Shoe proximity to drying completion

### Implementation

**Track Temperature Delta**:
```cpp
static float g_tempAtWETStart[2];
static float g_tempBaseline[2];  // Ambient after stabilization

// During WET, monitor if temperature is dropping
// Large drop = active evaporation (lots of water)
// Small drop = near completion (little water, equilibrium approached)

float tempDelta = g_dhtTemp[idx] - g_tempBaseline[idx];
if (tempDelta < 0.5°C && wetElapsed > 180000) {
  // Temperature barely dropping = evaporation slowing = near dry
  // Consider advancing sooner
}
```

**Multi-Sensor Confidence**:
- AH rate declining: suggests peak reached
- Temperature stable: suggests little evaporation happening
- Both together: strong confidence in peak

**Expected Improvement**:
- Better peak detection
- 10% faster for some shoe types
- Fewer mis-detections

---

## Phase 5: Machine Learning Profile Learning

### Problem Being Solved
Different shoes dry at different rates. System could learn and adapt.

### Simple Learning Approach

**Track Drying Profiles**:
```cpp
struct DryingProfile {
  float initialDiff;  // Wetness at WET start
  uint32_t timeToWETExit;  // How long WET phase ran
  float coolingDiffChange;  // How much COOLING reduced diff
  uint32_t timeToDRY;  // Total time to reach dry
};

// Store last 10 drying cycles
static DryingProfile g_history[10];
static int g_historyIdx = 0;

// On next WET entry, adjust expectations:
float avgTimePerWetDiff = average(g_history[].timeToDRY - g_history[].timeToWETExit) 
                          / average(g_history[].initialDiff);

// For current shoe with diff=3.5:
uint32_t estimatedCoolingTime = 3.5 * avgTimePerWetDiff;
```

**Per-Shoe Memory**:
- Detect if shoe inserted (diff > threshold)
- Check if shoe has history
- Adjust COOLING duration based on previous cycles
- Learn whether shoe dries fast or slow

**Benefits**:
- Personalized timing per shoe
- Faster convergence (no wasted time on first cycle)
- Optimal energy use

**Memory Cost**: ~400 bytes for history

---

## Phase 6: Dual-Motor Asynchronous Mode

### Problem Being Solved
Currently motors run sequentially (one at a time). Could be more efficient running both simultaneously with duty cycle management.

### Concept
- Allow both SUB1 and SUB2 WET phases to run at same time
- Control total power with reduced duty cycles per shoe
- Reduces total drying time by ~40%

```cpp
// Instead of sequential lock:
// If both shoes in WET, allocate power:
// Shoe 1: 50-60% duty
// Shoe 2: 50-60% duty
// Total system load stays manageable
```

**Considerations**:
- Requires dual-supply design (current is single motor supply)
- Power management more complex
- Watchdog timeout risks increase
- Likely v2.0+ feature

---

## Phase 7: Smart Heater Control

### Problem Being Solved
Currently heater runs full power during entire WET phase. Could be more selective.

### Optimization Options

**Option 1: Heater Warmup Only**
- Run heater only first 30-60 seconds
- Then switch to motor-only for remaining WET
- Reduces power consumption 20%

**Option 2: Adaptive Heater**
- If rate declining (peak reached), heater unnecessary
- Turn off heater 30 seconds after peak detected
- Saves final 60 seconds of heating

**Option 3: Temperature-Triggered**
- If shoe already warm (>35°C), skip heater
- Only use for cold, wet shoes
- Variable power based on initial temperature

---

## Phase 8: Sensor Fusion & Redundancy

### Problem Being Solved
Currently relies on single DHT per shoe. Vulnerable to:
- Single sensor failure
- Single EMI spike giving bad data
- No cross-validation

### Enhancement: Add Secondary Sensor

**Option A: Second DHT (low cost)**
- Add DHT at different physical location on shoe
- Average readings or use voting logic
- Cost: ~$5-10 per shoe

**Option B: Capacitive Humidity Sensor**
- Different technology (not resistive)
- Different failure modes
- Immune to DHT-specific EMI
- Cost: ~$15-20 per sensor

**Option C: Simple Temperature Gradient**
- Use multiple temp sensors
- Surface vs. internal temperature
- Indicates moisture still present (evaporation cooling)

---

## Phase 9: Web/Mobile Dashboard

### Problem Being Solved
Currently no visibility into drying process except serial logs. Could provide:

**Real-Time Monitoring**:
- Live progress bar for each shoe
- Current diff, AH, rate, temperature
- Motor duty, phase name, elapsed time

**Historical Analytics**:
- Total cycles per shoe
- Average time to dry
- Efficiency trends
- Failure/retry rates

**Predictive UI**:
- "Shoe 1: 8 minutes to DRY"
- "Shoe 2: 2 minutes to COOL"
- "Battery: ~2 hours remaining"

**Implementation**:
- ESP32 WiFi → cloud (Firebase/AWS)
- Mobile app (iOS/Android)
- Or simple local HTML dashboard

---

## Quick Wins (High ROI, Low Effort)

### 1. Adaptive COOLING Motor Phase II ✅ **DONE in v1.3.0**
Already implemented! Duty scales with wetness.

### 2. Config File Storage
Allow tuning constants to be read from SPIFFS (Flash filesystem)
- No recompile needed for tuning changes
- Save configurations per shoe type
- 2-3 hours of work, huge productivity gain

**Implementation**:
```cpp
// At startup:
loadConfigFromFS("/config.json");

// JSON file:
{
  "WET_MIN_DURATION_MS": 300000,
  "WET_PEAK_BUFFER_MS": 60000,
  "PID_KP": 0.10,
  // ... etc
}
```

### 3. Enhanced Serial Logging
Add structured JSON output for easier data analysis
```
{"event":"WET_PEAK","shoe":0,"time":185000,"rate":-0.15}
{"event":"COOLING_START","shoe":0,"diff":3.9,"duty":100}
```

### 4. Status LED Patterns
Use existing LED for richer feedback:
- Solid: Processing
- Blink 1Hz: WET
- Blink 2Hz: COOLING
- Blink 3Hz: DRY
- Off: Done/Error

### 5. Battery-Aware Shutdown
If battery drops below threshold during COOLING:
- Finish COOLING (don't interrupt)
- Skip UV
- Shut down gracefully
- Show "low battery" indicator

---

## Recommended Next Steps (Priority Order)

1. **SHORT TERM** (This Week):
   - [ ] Field test v1.3.0 with 5+ different very wet shoes
   - [ ] Collect data on success rate, cycle count, total time
   - [ ] Monitor for false peak detections
   - [ ] Document any issues

2. **MEDIUM TERM** (Next 2 Weeks):
   - [ ] Implement config file storage (Quick Win #2)
   - [ ] Add JSON logging for analysis
   - [ ] Test with different shoe materials
   - [ ] Optimize COOLING duration based on real data

3. **LONG TERM** (Next Month+):
   - [ ] Evaluate thermal feedback integration (Phase 4)
   - [ ] Consider machine learning profile (Phase 5)
   - [ ] Plan v2.0 hardware for dual-motor async (Phase 6)
   - [ ] Dashboard prototype (Phase 9)

---

## Success Metrics for v1.3.0

Track these to validate improvements:

| Metric | v1.2.0 | Target v1.3.0 | Method |
|--------|--------|-----------|--------|
| First-cycle success rate (diff > 3.0) | ~40% | >80% | Count cycles until DRY |
| Avg. time to DRY (very wet) | ~20 min | <15 min | Monitor log timestamps |
| False WET exits | ~30% | <5% | Count peaks followed by WAITING |
| COOLING diff reduction | 7% | 20-30% | Compare entry/exit diffs |
| Noise-induced false peaks | 40% | <10% | Examine rate patterns during peaks |
| Total system uptime | 98% | >99.5% | Monitor for watchdog resets |

---

## Known Limitations & Future Constraints

### Current Limitations
1. **Single motor** - Can only dry one shoe at a time
2. **Single DHT per shoe** - No redundancy
3. **Fixed heater** - Runs full power or off
4. **Simple rate detection** - 3-sample threshold
5. **No learning** - Same tuning for all shoes

### Hardware Bottlenecks (v2.0 needed for)
1. **Dual motor with independent control** - Complex power distribution
2. **Thermal imaging** - Would require additional sensors ($50+)
3. **Weight sensors** - Detect final water drops (not in scope)
4. **WiFi connectivity** - Requires different MCU (current limited)

### Firmware Bottlenecks (Current ESP32 can handle)
1. ✅ Moving average filtering - negligible overhead
2. ✅ Thermal feedback - one extra ADC read per second
3. ✅ Config file loading - SPIFFS available
4. ✅ Profile learning - storage available
5. ❌ ML inference - would need TensorFlow Lite (possible but complex)

---

## Long-Term Vision (v2.0 Design)

**Ideal Future System**:
- Dual independent motors (one per shoe)
- WiFi-enabled dashboard
- Thermal sensors in heating chamber
- Redundant humidity sensors per shoe
- Machine learning profiles
- Mobile app notifications
- 50% faster drying overall
- 99.9% first-cycle success

**Estimated Development**: 3-4 months, $200-300 additional hardware

