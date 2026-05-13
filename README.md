# Indoor Spatial Mapping System

An embedded spatial mapping system that uses a VL53L1X Time-of-Flight sensor, stepper motor rotation, UART serial communication, and MATLAB visualization to generate 3D indoor scan reconstructions.

## Overview

This project was developed for an embedded systems design course at McMaster University. The system performs 360-degree distance scans using a Time-of-Flight sensor mounted on a stepper motor. Distance readings are collected by a TM4C1294-based microcontroller, transmitted to a PC through UART, and processed in MATLAB to generate a 3D point-cloud style visualization of an indoor environment.

The system captures multiple 2D scan slices and offsets them in software to form a basic 3D reconstruction of a hallway or indoor space.

## Features

- 360-degree distance scanning using a VL53L1X Time-of-Flight sensor
- Stepper motor-based angular positioning
- I2C communication between the microcontroller and ToF sensor
- UART serial transmission at 115200 baud
- Push-button scan initiation using onboard GPIO
- LED status indicators for measurement and UART transmission states
- MATLAB serial data parsing and 3D visualization
- XYZ point-cloud file export for scanned coordinates
- Configurable number of scan points and scan repetitions

## Tech Stack

**Firmware**
- Embedded C
- TM4C1294 / MSP432E401Y LaunchPad
- GPIO
- I2C
- UART
- SysTick timing

**Hardware**
- VL53L1X Time-of-Flight sensor
- 28BYJ-48 stepper motor
- ULN2003 stepper motor driver
- MSP432E401Y LaunchPad / TM4C1294 microcontroller

**PC-Side Processing**
- MATLAB
- Serial communication using `serialport()`
- 3D visualization using `scatter3()` and `plot3()`
- XYZ file export

