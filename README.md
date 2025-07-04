# Warehous-Autonomous-Forklift-WAFL
A fully autonomous forklift designed for indoor warehouse environments, capable of omnidirectional motion and smart pallet handling. The system integrates low-level embedded control, high-level autonomy, and robust mechanical design to enable safe and efficient warehouse automation.

## 🧠 Low-Level Control & Communication (STM32 & CAN)

This repository contains modular firmware for a three-ECU embedded system using STM32F103 microcontrollers. The ECUs are connected via a robust CAN bus architecture:

- **Driving ECU:** Controls two drive motors and rear steering and stepper steering Motor
- **Sensor ECU:** Reads encoders, Limit swiths, and all forlift sensors  
- **Lifting ECU:** Controls the fork mechanism and status lighting

All data is transmitted to/from the Jetson Nano over CAN using custom message IDs. Each ECU runs real-time control loops with interrupt-driven CAN messaging, ensuring deterministic performance and scalable communication.

🛠️ Tools: STM32CubeIDE, STM32 HAL, C/C++, CANopen concepts (bare-metal)
