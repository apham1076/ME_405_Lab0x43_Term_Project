# ME_405
Contains files for ME 405 (Mechatronics)

# Romi Line-Following Robot — ME 405 Term Project

This repository contains the firmware, documentation, and design files for a fully autonomous Romi line-following robot developed as part of **ME 405: Mechatronics** at Cal Poly San Luis Obispo.

The project integrates closed-loop motor control, reflectance-based line sensing, cooperative multitasking firmware, and odometry-based state estimation to achieve reliable and repeatable line-following performance.

## Online Documentation
Full project documentation, including system architecture, hardware design, control development, and experimental results, is available here:

**[Project Portfolio Website](https://mduperly.github.io/ME_405_Term_Project/)**

## Repository Structure
- `src/` — MicroPython source code executed on the Romi microcontroller  
- `docs/` — Doxygen-generated project documentation and images  
- `utilities/` — Shared infrastructure such as queues and shared variables  
- `inactive tasks/` — Experimental or unused tasks retained for reference  

## Hardware Platform
- Pololu Romi chassis
- ST Nucleo development board
- Quadrature wheel encoders
- Reflectance sensor array
- IMU and Bluetooth module
- Six AA battery power system

## Notes
This repository reflects the final configuration used for time-trial demonstrations. The modular task-based architecture allows the system to be extended or modified without restructuring the entire codebase.
