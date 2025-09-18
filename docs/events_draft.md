# Events draft

Edit these descriptions and intended behavior. Use this file as a quick reference for what each `Event` means and when it should be raised.

- None: placeholder/no-event
- Error: A fatal or recoverable error occurred; go to Error state
- Debug: Manual debug event (unused in release), can be entered when reset button is held for 5 seconds force reset all system or error
- ResetPressed: User pressed reset, return to idle
- StartPressed: User pressed start, begin detection/run sequence
- BatteryLow / BatteryReady / ChargeDetected: battery power events
- SensorTimeout: when the given time for sensor equalization is achieved 30 secs waiting
- SubFSMDone: A sub-FSM has reached its Done state
- Shoe0InitWet / Shoe1InitWet: Initial classification that a shoe sensor reports wet
- Shoe0InitDry / Shoe1InitDry: Initial classification that a shoe sensor reports dry

Suggested usage:
- `tskDHT` should translate AH-diff logic into `ShoeXInitWet` or `ShoeXInitDry` events when asked by the Global FSM during start/detection.
- `tskFSM` will broadcast `StartPressed` and then feed init events to each sub-FSM as shown in the code.

You can edit the above to add timing, hysteresis, or alternate semantics.
