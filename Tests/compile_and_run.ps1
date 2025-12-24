# Compile and run a basic test for animation.cpp
# Usage: run this script from PowerShell. It will build animation.exe and run it with --create

Set-StrictMode -Version Latest
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Push-Location (Join-Path $root '..')

Write-Host "Building animation.cpp..."
g++ -std=c++17 -O2 animation.cpp -o animation.exe
if ($LASTEXITCODE -ne 0) {
    Write-Error "Compile failed. Ensure g++ is installed and on PATH."
    Pop-Location
    exit 1
}

Write-Host "Running animation.exe (--create)..."
.\animation.exe --create --target main_enhanced_test.py
$rc = $LASTEXITCODE

if ($rc -eq 0) { Write-Host "Run completed successfully." } else { Write-Error "Run exited with code $rc" }

Pop-Location
exit $rc
