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

