# Self-Balancing Scooter (STM32 Embedded System)

## Overview
This project implements a self-balancing scooter using the STM32 NUCLEO-F446RE microcontroller. The system models an inverted pendulum and achieves stability through real-time sensing, control, and actuation. The implementation integrates IMU-based tilt estimation, encoder feedback, and cascaded PID control to maintain balance and enable controlled motion.

## Code Organization 
The repository contains both user-written code and auto-generated code from STM32CubeMX.

## Repository Structure

- **User-implemented modules:**
  - `Core/Src/`
  - `Core/Inc/`

- **Auto-generated code :**
  - `Drivers/`
  - STM32 initialization and HAL configuration files
  - `.project`, `.cproject`, and related IDE files
 

## Project Configuration
The file `MIE438_project_test.ioc` can be opened in STM32CubeMX to inspect:
- Pin assignments
- Timer configuration (PWM, encoder mode)
- I2C interface setup
- System clock configuration

Code can be regenerated and modified through STM32CubeIDE.

## Software Modules

### PWM Control
- Generates motor control signals using hardware timer (TIM1)
- Provides high-resolution duty cycle control for motor speed

### Motor Driver
- Interfaces with H-bridge driver
- Combines PWM (speed) and GPIO (direction) control
- Provides abstraction for forward, reverse, and stop operations

### Encoder Interface
- Uses timer encoder mode for quadrature decoding
- Measures wheel displacement and velocity

### IMU (MPU6050)
- Communicates via I2C
- Reads accelerometer and gyroscope data
- Estimates tilt angle for feedback control

### Filtering
- Implements a complementary filter for angle estimation
- Combines gyroscope integration with accelerometer measurements

### PID Control
- Cascaded control structure:
  - Inner loop: balance (tilt stabilization)
  - Outer loop: velocity control
  - Turning control via differential motor input
- Ensures stable operation of the inverted pendulum system

### Main Control Loop
- Executes continuously after initialization
- Pipeline:
  Sensor reading → state estimation → control computation → motor actuation

## System Architecture
The system follows a closed-loop feedback structure:
- Sensor module: IMU and encoders
- Control module: STM32 microcontroller
- Actuation module: DC motors with H-bridge driver
- Power module: battery and regulated supply

