
# SoleCare Roadmap v1.3 → v1.5

## v1.3.0 (Current - Complete)
✅ Dual-phase PID setpoints  
✅ Moving-average peak detection  
✅ Adaptive cooling tiers (90/150/180s)  
✅ Trend-aware dry-check thresholds  
✅ Single cooling retry before WET re-entry  

---

## v1.4.0 (Stabilization & Quality)

### Core Improvements
- [ ] **UV delay after DRY** - 3s wait before UV starts (sensor settling)
- [ ] **PID saturation recovery** - Detect motor stuck at 100%, adjust setpoint
- [ ] **Adaptive heater warmup** - Reduce from 10s if shoe already warm

### Data & Logging
- [ ] **JSON PID logging** - Structured format (timestamp, setpoint, rate, duty, error)
- [ ] **State transition logs** - Clean state change records with timing
- [ ] **Readable debug output** - Cleaner serial format for analysis

### Testing & Validation
- [ ] **Single-pass drying tests** - 10+ cycles, target 90%+ success
- [ ] **Retry pattern analysis** - Verify one-retry strategy effective
- [ ] **Edge case testing** - Very wet vs barely wet shoes

---

## v1.5.0 (Smart Control & Integration)

### Advanced PID Control
- [ ] **Saturation detection & anti-windup** - Better integral term management
- [ ] **Kd filter** - Reduce noise in derivative term
- [ ] **Gain scheduling** - Adjust Kp/Ki/Kd based on drying phase

### UI & Monitoring
- [ ] **LED state indicators** - Dynamic blink rates per global state
- [ ] **OLED progress display** - Show current phase + % completion estimate
- [ ] **Temperature integration** - Collect & log shoe internal temp

### Data Analysis
- [ ] **Local HTML dashboard** - Real-time graphs (moisture trend, motor duty, temp)
- [ ] **CSV/JSON export** - Download logs for MATLAB/Python analysis
- [ ] **Cycle statistics** - Track success rate, average drying time, power usage

### Performance Tuning
- [ ] **Threshold learning** - Auto-adjust dry-check thresholds per shoe type
- [ ] **Power optimization** - Reduce motor duty during low-battery
- [ ] **Heater efficiency** - Disable heater earlier if peak detected sooner

---

## Future (v1.6+)

- **Machine Learning**: Predictive drying time, auto-tune PID gains
- **Multi-shoe priority**: Smart queuing for 3+ shoes
- **WiFi integration**: Remote monitoring & control
- **Safety limits**: Temperature cutoff, max drying time per cycle

