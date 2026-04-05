# Self-Balancing Scooter (STM32 Embedded System)

## Overview
This project implements a self-balancing scooter using the STM32 NUCLEO-F446RE microcontroller. The system models an inverted pendulum and achieves stability through real-time sensing, control, and actuation. The implementation integrates IMU-based tilt estimation, encoder feedback, and cascaded PID control to maintain balance and enable controlled motion.

## Demo Video

The scooter balancing and motion control demonstration is available at the following link:

**[▶ Watch Demo](https://youtu.be/dJwlU432Z2I)**

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

### PWM Control (`pwm.c`)
- Generates motor control signals using hardware timer (TIM1)
- Provides high-resolution duty cycle control for motor speed

### Motor Driver (`motor.c`)
- Interfaces with the H-bridge motor driver
- Combines PWM (speed) and GPIO (direction) control
- Provides abstraction for forward, reverse, and stop operations

### Encoder Interface (`encoder.c`)
- Uses timer encoder mode for quadrature decoding
- Measures wheel displacement and velocity

### IMU and Sensor Processing (`mpu6050.c`, `inv_mpu.c`, `inv_mpu_dmp_motion_driver.c`, `iic.c`)
- Handles communication with MPU6050 via I2C
- Reads accelerometer and gyroscope data
- Performs onboard processing (including angle estimation and sensor fusion)

### PID Control
- `pid.c`: Balance controller for maintaining upright position (standing still)
- `pid_sm.c`: Motion controller for forward and backward movement
- Implements cascaded control:
  - Inner loop: tilt stabilization (balance)
  - Outer loop: velocity control (movement)
  - Turning via differential motor commands

### Main Control Loop (`main.c`)
- Executes continuously after initialization
- Pipeline:
  Sensor reading → state estimation → control computation → motor actuation

  
## System Architecture
The system follows a closed-loop feedback structure:
- Sensor module: IMU and encoders
- Control module: STM32 microcontroller
- Actuation module: DC motors with H-bridge driver
- Power module: battery and regulated supply

