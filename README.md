# Micro

Unified firmware platform for an ESP32 controller and Raspberry Pi Pico motor node.  
This repository demonstrates a fully integrated communication stack, motor control subsystem, and system-level tooling in a single firmware package written in MicroPython.

---

## Features

- **Two-Node Architecture**
  - **ESP32 Node**: issues commands, schedules control sequences, manages telemetry ingestion, and exposes a simulated dashboard.
  - **Pico Node**: drives a motor with PWM + direction control, reads encoder feedback, executes PID loops, and enforces safety via watchdog.

- **Subsystems**
  - Config manager with simulated persistence
  - Authenticator using HMAC token validation
  - Frame protocol supporting JSON and TLV with CRC16
  - Scheduler and command sequencer for deterministic control
  - Telemetry builder and recorder with in-memory database
  - Diagnostics and watchdog logic
  - Web dashboard simulator (text-mode snapshot view)
  - CLI utility for injecting commands during runtime

- **Motor Control**
  - PWM-based duty cycle modulation
  - Direction management
  - Quadrature encoder reader
  - PID loop for closed-loop control

- **Robustness**
  - Watchdog timer with safe-stop behavior
  - Diagnostics checks for abnormal conditions
  - Structured logging with multiple levels

---

## Getting Started

### Requirements
- MicroPython environment (ESP32 and Raspberry Pi Pico boards supported)
- Python 3.x on host system for simulation

### Running Simulation on Host
```bash
python main.py
