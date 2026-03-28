# public
Dedicated to show some projects made by me.

# 1. Tupan Project

|!["ESP32 Inicialização"](https://github.com/EMendesS/public/blob/main/img/tupanV4/Board_preview.jpg)|
|:--:|
| *Fig. 1: Top preview* |
|!["ESP32 Inicialização"](https://github.com/EMendesS/public/blob/main/img/tupanV4/Board_preview_bottom.jpg)|
| *Fig. 2: Bottom preview* |

⚙️ Embedded Systems & Hardware Engineering Portfolio
Overview
This repository showcases a collection of high-performance embedded systems and hardware designs, covering real-time control, RF communication, precision sensing, and mixed-signal electronics. The projects emphasize end-to-end development, from schematic and PCB design to firmware architecture and system integration.

The focus is on building robust, production-grade systems with clear architectural separation, deterministic behavior, and scalable interfaces.

Core Competencies
Embedded Systems: STM32 (Cortex-M), ESP32, real-time firmware (C/C++)
System Architecture: Multi-MCU designs, task partitioning, IPC
Analog & Mixed Signal Design: Signal conditioning, PGAs, precision ADC systems
Power Electronics: Motor control, gate drivers, high-voltage design (up to 60V)
RF Communication: LoRa (SX1276), wireless telemetry systems
Sensor Integration: GNSS, IMUs, thermocouples, environmental sensors
PCB Design: High-density layout, noise mitigation, multi-layer boards
Debug & Validation: ST-Link, oscilloscopes, logic analyzers
Featured Projects
🔥 Breflow – Smart Reflow Oven Controller
Dual-MCU system combining STM32F7 real-time control with ESP32 UI/Webserver, designed for precise thermal profiling using thermocouples and PID regulation.

📡 L86T – Micro GNSS Module
Compact GNSS + sensor fusion platform integrating positioning, altitude, and heading in a minimal footprint for embedded tracking applications.

⚡ BLDC Telemetry & Control System
Wide-range (6V–60V) brushless motor controller with LoRa telemetry, dynamic PGA-based sensing, and full 3-phase current/voltage monitoring.

Design Philosophy
All projects follow a consistent engineering approach:

Separation of concerns: Control vs UI vs communication
Deterministic behavior: Real-time systems remain isolated from non-critical tasks
Scalability: Modular interfaces (I²C, AUX, expansion headers)
Precision: Careful analog design to preserve measurement integrity
Full-cycle ownership: Hardware, firmware, and system validation
Repository Structure
Each project folder typically includes:

Hardware design files (schematics, PCB)
Firmware source code
Documentation and system description
Bring-up and debugging notes
Future Work
This portfolio is continuously evolving and will include:

Advanced RF systems
High-speed data acquisition platforms
Industrial control systems
Embedded AI / edge processing applications
Contact
For collaboration, consulting, or technical discussions, feel free to reach out.