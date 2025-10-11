<#
Build and run the standalone test harness on Windows.

This script checks for common compilers in PATH (g++, clang++, cl.exe)
and attempts to compile the harness located in this folder:
 - run_tests.cpp
 - src/PIDcontrol.cpp

Usage (PowerShell):
  .\build_and_run_tests.ps1

If no supported compiler is found, install one of:
 - MinGW-w64 (g++) or MSYS2
 - Visual Studio Build Tools (cl.exe)
 - LLVM (clang++)

#>
Set-StrictMode -Version Latest

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
Push-Location $scriptDir

Write-Host "Building test harness in: $scriptDir" -ForegroundColor Cyan

function Try-Gpp {
    $g = Get-Command g++ -ErrorAction SilentlyContinue
    if ($null -ne $g) {
        Write-Host "Found g++ at: $($g.Path)" -ForegroundColor Green
        & g++ -std=c++17 -I src run_tests.cpp src/PIDcontrol.cpp -O2 -o run_tests.exe
        if ($LASTEXITCODE -eq 0) { return "./run_tests.exe" }
    }
    return $null
}

function Try-Clang {
    $c = Get-Command clang++ -ErrorAction SilentlyContinue
    if ($null -ne $c) {
        Write-Host "Found clang++ at: $($c.Path)" -ForegroundColor Green
        & clang++ -std=c++17 -I src run_tests.cpp src/PIDcontrol.cpp -O2 -o run_tests.exe
        if ($LASTEXITCODE -eq 0) { return "./run_tests.exe" }
    }
    return $null
}

function Try-Cl {
    $cl = Get-Command cl.exe -ErrorAction SilentlyContinue
    if ($null -ne $cl) {
        Write-Host "Found MSVC cl.exe at: $($cl.Path)" -ForegroundColor Green
        # Compile with cl.exe (Visual Studio). Use /Fe to set output exe name.
        & cl.exe /EHsc /I src run_tests.cpp src\PIDcontrol.cpp /Fe:run_tests.exe
        if ($LASTEXITCODE -eq 0) { return ".\run_tests.exe" }
    }
    return $null
}

$exe = Try-Gpp
if (-not $exe) { $exe = Try-Clang }
if (-not $exe) { $exe = Try-Cl }

if (-not $exe) {
    Write-Host "No supported compiler found in PATH. Please install MinGW-w64 (g++), LLVM (clang++), or Visual Studio Build Tools (cl.exe)." -ForegroundColor Red
    Pop-Location
    exit 2
}

Write-Host "Running tests: $exe" -ForegroundColor Cyan
& $exe
$exitCode = $LASTEXITCODE

Pop-Location
exit $exitCode
