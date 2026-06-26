# 6 DOF Robotic Arm

Custom 6-axis arm with stepper-driven joints, BLDC gripper, inverse kinematics, computer vision, and PyQt5 GUI. RAMPS 1.4 + TMC2209 + Arduino Mega + Python host.

![Arm](https://github.com/user-attachments/assets/6418f20c-f2b9-46b2-b623-9847ae49599c)

## Repo layout

| Path | Contents |
|---|---|
| `CAD/` | STL parts grouped by sub-assembly (Base, Shoulder, Upper_Arm, Elbow, Forearm, Wrist, Gripper, Assembly) |
| `Code/GUI_Control/` | Arduino firmware (`fullcntl.ino`) + Python host GUI (`fullcntl.py`) + pin / motor constant headers |
| `Code/Tests/` | Standalone bring-up scripts for camera, vision, limit switches, BLDC gripper, joint timing |
| `Matlab URDF Sim/` | Independent MATLAB/Simulink kinematic simulator with GUI (separate from the physical-arm controller) |
| `Schematics/` | RAMPS 1.4 pinout doc + reference PDF/JPG |
| `Vids_Photos/` | Demo media |

## Hardware

- **Joints**: 6 NEMA17 stepper motors (1.8°/step) driven by 6× TMC2209 in STEP/DIR mode (5 on RAMPS 1.4, 1 on an external breakout), 1/16 microstepping
- **End effector**: BLDC gripper (11 pole-pairs, 15 Ω, 80 KV) driven via SimpleFOC + 3-PWM driver
- **Homing**: 3 inductive sensors (J1, J4, J6) + 3 mechanical NC limit switches (J2, J3, J5)
- **MCU**: Arduino Mega 2560 on RAMPS 1.4 shield
- **Host link**: USB serial @ 115200 baud

## Per-joint config (from `Code/GUI_Control/motorcst.h`)

| Joint | Gear ratio | Home angle (°) | Target after home (°) | Back-off (°) |
|---|---:|---:|---:|---:|
| J1 (base) | 6.4 | 0 | 117.0 | 5.0 |
| J2 (shoulder) | 20.0 | 35.408 | 90.0 | 5.0 |
| J3 (elbow) | 18.095 | 0 | -76 | 5.0 |
| J4 (forearm roll) | 4.0 | 0 | -107 | 5.0 |
| J5 (wrist pitch) | 4.0 | 0 | -114 | 5.0 |
| J6 (wrist roll) | 10.0 | 0 | 100 | 5.0 |

## DH parameters (from `Code/GUI_Control/fullcntl.py`)

| Link | a (mm) | α (rad) | d (mm) | θ offset (rad) |
|---|---:|---:|---:|---:|
| 1 | 23.420 | -π/2 | 110.500 | 0 |
| 2 | 180.000 | π | 0 | -π/2 |
| 3 | -43.500 | π/2 | 0 | π |
| 4 | 0 | -π/2 | -176.350 | 0 |
| 5 | 0 | π/2 | 0 | 0 |
| 6 | 0 | π | -125.075 | π |

## Joint limits (deg)

J1 ±123.05 · J2 [-55.01, +86.63] · J3 [-72.13, +107.87] · J4 ±105.47 · J5 ±90 · J6 ±360

## Software stack

- **Firmware libs**: `AccelStepper`, `SimpleFOC`
- **Host**: Python 3 + `PyQt5`, `opencv-python`, `numpy`, `scipy`, `pyserial`
- **IK**: SciPy `L-BFGS-B` minimize on position² + 10·orientation-error² + 0.1·joint-change penalty, warm-started from previous solution

## Quick start

1. Install Arduino libraries `AccelStepper` + `SimpleFOC` via Library Manager
2. Flash `Code/GUI_Control/fullcntl.ino` to Arduino Mega 2560
3. Wire per `Schematics/README.md` pin table
4. Set serial port string in `Code/GUI_Control/fullcntl.py` → `init_serial()`
5. `pip install pyqt5 opencv-python numpy scipy pyserial`
6. `python fullcntl.py`
7. Press **Start Homing** → wait → enter (X, Y, Z) mm → **Move to Position**

## Serial protocol (115200 baud, newline-terminated)

| Cmd | Action |
|---|---|
| `H` | Run full homing sequence |
| `S` | Shutdown (move all joints +10° from home) |
| `R` | Reset all joints to 0 |
| `M<a1>,<a2>,<a3>,<a4>,<a5>,<a6>` | Move joints to absolute angles (deg) |
| `OPEN` | Open BLDC gripper |
| `CLOSE` | Close BLDC gripper |

## Demos

- [Move demo](https://github.com/user-attachments/assets/76c58376-af98-44d1-89f5-d9b5ce975651): IK + AccelStepper smooth motion to commanded (X, Y, Z)
- [Object detection + pick-and-place](https://github.com/user-attachments/assets/8c85c374-1d46-4252-8022-c0bc396d2eb2): OpenCV finds object, host computes IK, firmware executes
- [Full homing sequence](https://github.com/user-attachments/assets/09466da0-1334-4c38-82d4-a613223febb5): fast approach, trigger, back off, slow re-approach for repeatable zero

![Electronics](https://github.com/user-attachments/assets/5dd10d63-feb1-4a9c-b234-279e1b881540)
![Build](https://github.com/user-attachments/assets/99c5d5a8-d399-4f37-b20c-db5fae20546b)
