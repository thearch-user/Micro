This folder contains simple test helpers for `animation.cpp`.

Files:
- `compile_and_run.ps1` — PowerShell script that compiles `animation.cpp` and runs it with `--create --target main_enhanced_test.py`.

How to run (PowerShell):

```powershell
cd path\to\Micro
Tests\compile_and_run.ps1
```

Notes:
- Requires `g++` on PATH (e.g., from MinGW or MSYS2) to compile.
- The script will create `animation.exe` in the repository root and `main_enhanced_test.py` if `main.py` exists.
