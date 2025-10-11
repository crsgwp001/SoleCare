# SoleCare

Small project for controlling UV, motor, and heaters with an ESP32. This repository includes a host-side test harness for the `PIDcontrol` library and a PlatformIO-native test configuration.

## Run tests on CI

The repository includes a GitHub Actions workflow `.github/workflows/ci.yml` which runs PlatformIO native tests in `test_harness`.

## Run tests locally (recommended)

Option A — PlatformIO (recommended):

1. Install PlatformIO (via VS Code PlatformIO extension or pip):

   pip install -U platformio

2. From the project root, run the native test environment in `test_harness`:

```powershell
cd test_harness
platformio test -e native -v
```

Option B — Host compiler (fallback):

If you prefer to run tests without PlatformIO, use the included PowerShell helper which will try `g++`, `clang++`, or MSVC `cl.exe`:

```powershell
.\test_harness\build_and_run_tests.ps1
```

Notes
- The host harness provides a `millis()` shim so `PIDcontrol` builds on the host without requiring Arduino headers.
- Tests live in `test_harness/test` and a duplication is available under `test/test_pid` for PlatformIO-based test flows.
# SoleCare
Software for the Smart Shoe Dryer using an ESP32 Dev Module.
