# RAMPS 1.4 Pinout

Wiring map for the arm: 6 stepper drivers + 6 limit / proximity sensors. Mirrored in `Code/GUI_Control/pins.h`.

## Motor controllers

- 6× TMC2209 in STEP / DIR mode
  - 5 plugged into RAMPS 1.4 driver slots
  - 1 on an external breakout wired through free GPIO pins
- Set every microstepping selector switch to **1** (not 0) → 1/16 microstepping

## Limit / proximity switches

- 3 inductive sensors → J1, J4, J6
- 3 mechanical NC switches → J2, J3, J5

## Per-joint pin map

### J1 (base)
- STEP_PIN: 54 · DIR_PIN: 55 · ENABLE_PIN: 38
- Limit switch: pin 5 (inductive)
- Sensor power: pin 10 (D10 ON)

### J2
- STEP_PIN: 60 · DIR_PIN: 61 · ENABLE_PIN: 56
- Limit switch: pin 3 (mechanical NC)

### J3
- STEP_PIN: 46 · DIR_PIN: 48 · ENABLE_PIN: 62
- Limit switch: pin 14 (mechanical NC)

### J4
- STEP_PIN: 26 · DIR_PIN: 28 · ENABLE_PIN: 24
- Limit switch: pin 11 (inductive)
- Sensor power: pin 9 (D9 ON)

### J5
- STEP_PIN: 36 · DIR_PIN: 34 · ENABLE_PIN: 30
- Limit switch: pin 18 (mechanical NC)

### J6 (end effector)
- STEP_PIN: 42 · DIR_PIN: 44 · ENABLE_PIN: 40
- Limit switch: pin 4 (inductive, always powered)

## Notes

- J1 and J4 inductive sensors need their dedicated D10 / D9 power lines set HIGH.
- J6's inductive sensor is always powered.
- For mechanical switches, both COM and NC are wired.

## Reference files

- `RAMPS 1.4 - RepRap.pdf` — full RepRap schematic
- `ramps_1.4_pinout.jpg` — quick pinout reference

![RAMPS 1.4 Pinout](https://github.com/user-attachments/assets/e9fb35c4-472a-451c-9f5b-543ad330c704)
