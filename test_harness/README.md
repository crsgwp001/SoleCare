# Test harness (host)

This folder contains a lightweight host test harness for the `PIDcontrol` library.
It is meant to run on the CI runner or locally on your development machine.

Prerequisites
- A C++ compiler available in your PATH (g++ / clang++ on Windows via MSYS2/MinGW, or Visual Studio's `cl.exe`).
- PowerShell (Windows) or a POSIX shell on other platforms.

Quick run (recommended)
Use the included PowerShell helper which tries to detect a suitable compiler and run the tests:

```powershell
# from the repo root
cd test_harness
.\build_and_run_tests.ps1
```

Manual run with g++ (PowerShell)

```powershell
cd test_harness
# compile
g++ -std=c++17 -Isrc run_tests.cpp src/PIDcontrol.cpp -o run_tests
# run
.\run_tests
```

Notes
- The harness is intentionally small and only exercises the PID math/logic (not hardware). It prints simple PASS/FAIL lines.
- If you use Visual Studio's `cl.exe`, either run the Developer Command Prompt or follow MSVC documentation to compile C++ sources.

Contact
If something fails on your machine, run the helper script and paste the output when reporting issues — it contains basic environment detection that helps debugging.
