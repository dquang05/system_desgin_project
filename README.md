# ESP32 Autonomous Mobile Robot (AMR)

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-red.svg)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-Supported-blue.svg)
![C++17](https://img.shields.io/badge/C++-17-00599C.svg)

An advanced Autonomous Mobile Robot firmware built on the **ESP-IDF v5.x** framework. Designed with strict real-time constraints, thread-safety, and modular architecture, this project demonstrates professional-grade embedded systems engineering using **FreeRTOS** and Object-Oriented C++.

## 🚀 System Architecture

The firmware is designed with a strict **Application-Library Separation**.

- `lib/`: Contains hardware-agnostic, non-blocking drivers (Motor Control, PID, ADC DMA, Loadcell, Wi-Fi). No FreeRTOS tasks are spawned here.
- `src/`: The Application layer orchestrating high-level state machines, task creation, and global resource management.

### Key Engineering Highlights

- **Deterministic Real-Time Execution:** Motion control and PID loops run precisely at 100Hz in a pinned Core 1 task.
- **Thread Safety:** Lock-free passing where possible, combined with strict `portENTER_CRITICAL` Spinlocks for sharing high-frequency data (like Encoder ticks and ADC raw values) across multicore tasks.
- **Zero Dynamic Allocation in Critical Loops:** No `malloc` or `new` in polling loops. Memory is strictly stack-allocated or statically provisioned at startup to prevent Heap Fragmentation.
- **Direct Memory Access (DMA):** The 5-channel Infrared Line Sensor array is sampled at 20kHz continuously via ESP32's ADC DMA hardware, entirely offloading the CPU.

## 🧠 Control Algorithms

### 1. Velocity Control (Low-Level)

- **Incremental PID Controller:** Controls the TB6612 H-Bridge drivers coupled with Quadrature Encoders.
- **Slew Rate Limiting:** Prevents sudden current spikes (Back-EMF) by artificially capping the maximum acceleration `dv/dt`.
- **Integral Anti-Windup:** Prevents overshooting when wheels are mechanically jammed or lose traction.

### 2. Autonomous Navigation (High-Level)

- **Line Tracking:** Uses Weighted Average (Centroid) calculation with Affine Calibration to determine precise offset error (`e2`).
- **State Machine:** Seamlessly transitions between `MOVING_TO_PICKUP`, `WAITING_FOR_PACKAGE` (Loadcell integration), and `DELIVERING`.
- **Two-Phase Pivot Turn:** Executes precise turning maneuvers at intersections without the need for IMU/Compass, relying purely on encoder odometry and side-sensor polling.

### 3. Remote Telemetry & Tuning

- Utilizes asynchronous **UDP Sockets** to broadcast telemetry data (RPM, Duty Cycle, Sensor Arrays) to a local network at high frequency.
- Supports Live-Tuning: Send a JSON payload via UDP to dynamically update PID gains, steering RPMs, or loadcell thresholds **without recompiling or halting the robot**.

## 🛠️ Build & Flash Instructions

This project is configured using standard ESP-IDF CMake.

1. **Prerequisites:** Ensure ESP-IDF v5.x is installed.
2. **Set Target:**
   ```bash
   idf.py set-target esp32
   ```
3. **Build & Flash:**
   ```bash
   idf.py build
   idf.py -p PORT flash monitor
   ```

## 📁 Repository Structure

```text
├── MCU/
│   ├── include/           # Shared structures, Configs, and Orchestrator Headers
│   ├── src/               # FreeRTOS Tasks (Motion, Telemetry, UDP) & Main
│   ├── lib/               # Independent Component Libraries
│   │   ├── adc_dma/       # Continuous DMA ADC Driver
│   │   ├── loadcell_hx711/# HX711 Weight Sensor
│   │   ├── tb6612_encoder/# Motor & Quadrature Encoder Driver
│   │   ├── velocity_pid/  # Slew-Rate Limited PID
│   │   └── wifi_manager/  # Wi-Fi Station & AP provisioning
│   └── CMakeLists.txt     # Global IDF Component Registration
└── UI/                    # Node.js Server & Web Dashboard for UDP Telemetry
```
