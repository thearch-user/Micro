m Micro-Analytics

Micro-Analytics is a full-stack, enterprise-grade firmware and tooling platform for a dual-node embedded system, featuring an ESP32 controller and a Raspberry Pi Pico motor node. This repository showcases a complete, integrated solution, from low-level motor control to a sophisticated web-based management suite.

The project is designed for inspection and demonstration, with a focus on robust, production-quality code and a comprehensive feature set.

---

## Key Features

- **Dual-Node Architecture**
  - **ESP32 Node**: Manages high-level command and control, telemetry ingestion, scheduling, and exposes a web-based dashboard.
  - **Pico Node**: Handles real-time motor control, including PWM, direction, encoder feedback, PID loops, and safety watchdog.

- **Firmware Subsystems (MicroPython)**
  - **Configuration Manager**: Simulated persistence for system settings.
  - **HMAC Authentication**: Secure command and control.
  - **Communication Protocol**: JSON and TLV framing with CRC16 validation.
  - **Scheduler & Sequencer**: Deterministic execution of command sequences.
  - **Telemetry Engine**: In-memory database for telemetry data.
  - **Diagnostics & Watchdog**: Ensures system stability and safety.

- **Motor Control**
  - PWM-based duty cycle modulation.
  - Quadrature encoder for position feedback.
  - PID controller for closed-loop control.

- **Web Dashboard (`index.html`)**
  - A comprehensive, interactive UI for system management.
  - Real-time charts for telemetry and motor performance (using Chart.js).
  - Multi-level user authentication (Viewer, Engineer, Admin, Founder).
  - A project board for managing tasks and database records.
  - Dark/light theme support.

- **Tooling & Utilities**
  - **C++ Enhancements**: A utility to enhance the `main.py` firmware with compatibility shims and line ending normalization (`enhancements.cpp`), and a simple animation to demonstrate the process (`animation.cpp`).
  - **C# Secure Database Examples**: A collection of best practices for secure database interactions, including prevention of SQL injection and secure password handling (`injection.cs`).
  - **Data Analysis (`activations.py`)**: A Python script for advanced data analysis and machine learning on the collected data, using libraries like pandas, scikit-learn, and statsmodels.
  - **Automated Testing**: A suite of tests in Python (`Tests/test_runner.py`) and Ruby (`Tests/test.rb`) to ensure the reliability of the firmware.

---

## Getting Started

### Requirements
- **MicroPython Environment**: For deploying the firmware to an ESP32 or Raspberry Pi Pico.
- **Python 3.x**: For running the simulation on a host machine.
- **C++ Compiler**: For building the enhancement utility.
- **.NET SDK**: For the C# examples.
- **Ruby**: For running the test suite.

### Running the Simulation
To run the full simulation on your host machine, execute the `main.py` script:
```bash
python main.py
```

### Web Dashboard
To use the web dashboard, open `index.html` in a modern web browser. For the best experience, use a live server extension in your code editor (like VS Code's "Live Server").

The dashboard provides a real-time view of the system's performance, including telemetry graphs, motor control status, and diagnostic information.

---

## Project Structure

- **`main.py`**: The core MicroPython firmware for the ESP32 and Pico nodes.
- **`index.html`**: The web-based management dashboard.
- **`enhancements.cpp`**: A C++ utility for preparing the firmware.
- **`animation.cpp`**: A C++ animation to demonstrate the enhancement process.
- **`injection.cs`**: C# code snippets for secure database operations.
- **`activations.py`**: A Python script for data analysis and machine learning.
- **`Tests/`**: A directory containing the automated test suite.
- **`LICENSE`**: The MIT license for the project.
- **`README.md`**: This file.