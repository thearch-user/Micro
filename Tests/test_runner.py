#!/usr/bin/env python3
"""
Simple test runner: run `main.unit_tests()` sequentially N times, report pass/fail and show stdout/stderr on failure.

This avoids running the full simulation in `main.py` by importing the module and calling the `unit_tests()` function directly.
"""
import subprocess
import sys
import time

def run_once(timeout=10):
    # Run a short import call that executes only unit_tests (module import won't run __main__ block).
    # Provide compatibility shims for some MicroPython-only modules used by main.py
    shim = (
        "import sys, binascii as _binascii, struct as _struct;"
        "sys.modules['ubinascii'] = _binascii;"
        "sys.modules['ustruct'] = _struct;"
        "import main; main.unit_tests()"
    )
    cmd = [sys.executable, "-c", shim]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        passed = proc.returncode == 0
        return passed, proc.stdout, proc.stderr, proc.returncode
    except subprocess.TimeoutExpired as e:
        out = e.stdout or ""
        err = e.stderr or "Timeout expired"
        return False, out, err, None

def main(times=5, timeout=10, pause_between=0.2):
    results = []
    any_fail = False
    for i in range(1, times + 1):
        print(f"Running iteration {i}/{times}...")
        passed, out, err, rc = run_once(timeout=timeout)
        if passed:
            print(f"Run {i}: PASSED")
        else:
            any_fail = True
            print(f"Run {i}: FAILED (rc={rc})")
            if out:
                print("--- STDOUT ---")
                print(out)
            if err:
                print("--- STDERR ---")
                print(err)
        print("" + ("-" * 40))
        results.append((passed, out, err, rc))
        time.sleep(pause_between)

    print("Summary:")
    for idx, (passed, _, _, _) in enumerate(results, start=1):
        print(f" Run {idx}: {'PASSED' if passed else 'FAILED'}")

    if any_fail:
        print("One or more runs failed.")
        sys.exit(2)
    print("All runs passed.")
    sys.exit(0)

if __name__ == '__main__':
    # default: run 5 times
    main(times=5)
