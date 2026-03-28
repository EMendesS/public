# ⚙️ Embedded Systems & Hardware Engineering Portfolio

## Overview
This repository showcases a collection of high-performance embedded systems and hardware designs, covering real-time control, RF communication, precision sensing, and mixed-signal electronics. The projects emphasize end-to-end development, from schematic and PCB design to firmware architecture and system integration.

The focus is on building robust, production-grade systems with clear architectural separation, deterministic behavior, and scalable interfaces.

## Core Competencies
- **Embedded Systems:** STM32 (Cortex-M), ESP32, real-time firmware (C/C++)
- **System Architecture:** Multi-MCU designs, task partitioning, IPC
- **Analog & Mixed Signal Design:** Signal conditioning, PGAs, precision ADC systems
- **Power Electronics:** Motor control, gate drivers, high-voltage design (up to 60V)
- **RF Communication:** LoRa (SX1276), wireless telemetry systems
- **Sensor Integration:** GNSS, IMUs, thermocouples, environmental sensors
- **PCB Design:** High-density layout, noise mitigation, multi-layer boards
- **Debug & Validation:** ST-Link, oscilloscopes, logic analyzers

## Featured Projects

### ⚡ Tupan - BLDC Telemetry & Control System
|!["--"](https://github.com/EMendesS/public/blob/main/img/tupan_top.png)|
|:--:|
| *Fig. 1: Tupan top preview* |
|!["ESP32 Inicialização"](https://github.com/EMendesS/public/blob/main/img/tupan_btm.png)|
| *Fig. 2: Tupan bottom preview* |
Wide-range (6V–60V) brushless motor controller with LoRa telemetry, dynamic PGA-based sensing, and full 3-phase current/voltage monitoring.

**Tupan** is a high-performance, integrated motor control and telemetry platform designed for wide-voltage operation. Engineered for industrial and robotic propulsion, it combines real-time brushless motor control with long-range RF communication and high-precision power monitoring.

*   **Power Stage (DRV8353):** A robust 60V three-phase smart gate driver controlling three half-bridges, capable of managing high-torque brushless motors from **6V to 60V**.
*   **Primary Controller (STM32F722):** An ARM Cortex-M7 core executing real-time motor control algorithms, high-speed data acquisition, and safety-critical logic.
*   **Connectivity & RF:** Integrated **ESP32** for local connectivity and **SX1276 LoRa** for long-range telemetry, enabling remote monitoring in challenging environments.
*   **Precision Telemetry:** Features a **Dynamic PGA (Programmable Gain Amplifier)** topology to maintain high ADC resolution across the full 60V range, providing accurate **inline 3-phase current and voltage sensing**.
*   **Acoustic & Motion Sensing:** Integrated **MP34DT05-A** digital MEMS microphone for acoustic diagnostics and an **ICM-42688-P** 6-axis IMU for vibration and orientation tracking.
*   **System Monitoring:** Dedicated **ADS1015-Q1** precision ADC for continuous monitoring of internal voltage buses and power rail integrity.
*   **Expansion & I/O:** High-speed **USB-C** interface, onboard **MUX** for port expansion, and a multi-purpose I/O connector for system integration.

---

### **Engineering References**
*   **Smart Gate Driver:** [TI DRV8353 60V Three-Phase Driver](https://www.ti.com/product/DRV8353)
*   **LoRa Transceiver:** [Semtech SX1276 Wireless RF](https://www.semtech.com/products/wireless-rf/lora-core/sx1276)
*   **Precision ADC:** [TI ADS1015-Q1 Monitoring ADC](https://www.ti.com/product/ADS1015-Q1)
*   **Motion Tracking:** [TDK ICM-42688-P 6-Axis IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)
*   **MEMS Microphone:** [ST MP34DT05-A Digital Mic](https://www.st.com/en/mems-and-sensors/mp34dt05-a.html)

### 🔥 Breflow – Smart Reflow Oven Controller
|!["--"](https://github.com/EMendesS/public/blob/main/img/breflow_top.png)|
|:--:|
| *Fig. 3: Breflow top preview* |
|!["ESP32 Inicialização"](https://github.com/EMendesS/public/blob/main/img/breflow_btm.png)|
| *Fig. 4: Breflow bottom preview* |
Dual-MCU system combining STM32F7 real-time control with ESP32 UI/Webserver, designed for precise thermal profiling using thermocouples and PID regulation.

**Breflow** is an experimental-grade embedded control platform engineered for high-accuracy thermal profiling. By utilizing a **dual-processor architecture**, the system achieves a strict separation of concerns: a high-performance MCU handles real-time deterministic control, while a dedicated co-processor manages the user interface and network connectivity.

*   **Primary Controller (STM32F722):** An ARM Cortex-M7 core managing real-time PID control loops, safety-critical thermal monitoring, and high-speed peripheral interfacing.
*   **Connectivity Co-Processor (ESP32):** A dedicated module functioning as a **TFT Controller** and **Webserver**, providing a responsive graphical interface and remote monitoring via Wi-Fi.
*   **Thermal Acquisition:** Dual-channel thermocouple interface using high-precision digital converters with integrated cold-junction compensation.
*   **Analog Front-End:** Integrated **Programmable Gain Amplifier (PGA)** for flexible signal conditioning and enhanced ADC resolution.
*   **Integrated Debugging:** On-board **ST-Link programmer** for the STM32 and a dedicated programming header for the ESP32, enabling seamless field updates.
*   **Connectivity:** Modern **USB-C** interface for high-speed data logging and a multi-purpose I2C bus for system scaling.

---

### **Engineering References**
*   **Main MCU:** [STM32F722 High-Performance Series](https://www.st.com/en/microcontrollers-microprocessors/stm32f722ze.html)
*   **Thermal Sensing:** [MCP9600 Thermocouple-to-Digital Converter](https://www.microchip.com/en-us/product/MCP9600)
*   **Motion Tracking:** [ICM-42688-P 6-Axis IMU](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42688-p/)

### 📡 L86T – Micro GNSS Module
|!["--"](https://github.com/EMendesS/public/blob/main/img/L86T_top.png)|
|:--:|
| *Fig. 5: L86T top preview* |
|!["ESP32 Inicialização"](https://github.com/EMendesS/public/blob/main/img/L86T_btm.png)|
| *Fig. 6: L86T bottom preview* |
Compact GNSS + sensor fusion platform integrating positioning, altitude, and heading in a minimal footprint for embedded tracking applications.

The **L86T** is a high-precision, ultra-compact positioning and environmental sensing platform. Engineered for embedded navigation, it integrates a multi-constellation GNSS engine with a suite of high-resolution sensors to provide a complete telemetry solution in a minimal footprint.

*   **Core Navigation:** Powered by the **L86-M33 GNSS** module, supporting multi-constellation tracking for high-sensitivity positioning and fast time-to-first-fix (TTFF).
*   **Main Controller (STM32F103):** A dedicated ARM Cortex-M3 MCU for real-time data processing, sensor fusion, and telemetry management.
*   **Environmental Sensing:** Integrated **BMP388** barometric pressure sensor for high-resolution altitude tracking and vertical velocity monitoring.
*   **Orientation & Heading:** Onboard **LIS3MDL** 3-axis magnetometer for precise magnetic heading and orientation data.
*   **Power Management:** Integrated **BQ25180** battery management system for advanced charging, safety monitoring, and low-power operation.
*   **Unified Interface:** A single multi-purpose connector provides both **I2C telemetry** and firmware programming, simplifying system integration.

---

### **Engineering References**
*   **GNSS Module:** [Quectel L86-M33 Series](https://www.quectel.com/product/gnss-l86-module)
*   **Mainstream MCU:** [STM32F103 Series](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)
*   **Barometric Sensor:** [Bosch BMP388 Pressure Sensor](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/)
*   **Magnetometer:** [ST LIS3MDL 3-Axis Magnetometer](https://www.st.com/en/mems-and-sensors/lis3mdl.html)
*   **Battery Charger:** [TI BQ25180 Management IC](https://www.ti.com/product/BQ25180)

## Design Philosophy
All projects follow a consistent engineering approach:
- **Separation of concerns:** Control vs UI vs communication
- **Deterministic behavior:** Real-time systems remain isolated from non-critical tasks
- **Scalability:** Modular interfaces (SPI, UART, I2C, AUX, expansion headers)
- **Precision:** Careful analog design to preserve measurement integrity
- **Full-cycle ownership:** Hardware, firmware, and system validation

## Contact
For collaboration, consulting, or technical discussions, feel free to reach out.