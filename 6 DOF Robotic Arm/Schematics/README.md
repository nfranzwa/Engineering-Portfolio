# Robot Pinout Configuration

This document outlines the pin configurations for our robot project using a RAMPS 1.4 board with TMC2209 motor controllers.

## Motor Controller Setup

- We are using 6 TMC2209 motor controllers:
  - 5 connected directly to the RAMPS 1.4 board
  - 1 connected via an external board through free GPIO pins
- All motor controllers are operating in STEP/DIR mode

## Limit Switch Configuration

- 3 inductive sensors
- 3 mechanical switches (wired in NC configuration)

## Joint-Specific Pin Configurations

### J1 (Base Joint)
- STEP_PIN: 54
- DIR_PIN: 55
- ENABLE_PIN: 38
- Limit Switch: Pin 5 (Inductive sensor)
- Sensor Power: Pin 10 (D10 on RAMPS, set to ON)

### J2
- STEP_PIN: 60
- DIR_PIN: 61
- ENABLE_PIN: 56
- Limit Switch: Pin 3 (Mechanical switch)

### J3
- STEP_PIN: 46
- DIR_PIN: 48
- ENABLE_PIN: 62
- Limit Switch: Pin 14 (Mechanical switch)

### J4
- STEP_PIN: 26
- DIR_PIN: 28
- ENABLE_PIN: 24
- Limit Switch: Pin 11 (Inductive sensor)
- Sensor Power: Pin 9 (D9 on RAMPS, set to ON)

### J5
- STEP_PIN: 36
- DIR_PIN: 34
- ENABLE_PIN: 30
- Limit Switch: Pin 18 (Mechanical switch)

### J6 (End Effector)
- STEP_PIN: 42
- DIR_PIN: 44
- ENABLE_PIN: 40
- Limit Switch: Pin 4 (Inductive sensor, always ON)

## Notes
- J1 and J4 have dedicated power pins (D10 and D9 respectively) on the RAMPS board that need to be set to ON.
- J6's inductive sensor is always powered ON.
- For mechanical switches, both C (Common) and NC (Normally Closed) are wired.

![RAMPS 1.4 Pinout](https://github.com/user-attachments/assets/e9fb35c4-472a-451c-9f5b-543ad330c704)
