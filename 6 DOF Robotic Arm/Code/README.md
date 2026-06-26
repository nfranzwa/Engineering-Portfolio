# Code

Two-part control system: Arduino firmware on the arm + Python host running IK, GUI, and vision on a PC.

## `GUI_Control/`

| File | Role |
|---|---|
| `fullcntl.ino` | Arduino Mega firmware. Owns motion, homing, limit-switch handling, BLDC gripper. AccelStepper for J1–J6, SimpleFOC for gripper. Parses serial commands. |
| `fullcntl.py` | Host-side PyQt5 GUI. Forward + inverse kinematics (DH-param L-BFGS-B). Vision thread. Sends serial commands to firmware. |
| `pins.h` | All STEP / DIR / ENABLE / limit-switch / sensor-power pins per joint. |
| `motorcst.h` | Microstepping, step angle, per-joint gear ratios, home / target / back-off angles, BLDC voltage states. |

### Arduino dependencies

- `AccelStepper`
- `SimpleFOC`

Install via Arduino Library Manager before flashing.

### Python dependencies

```
pip install pyqt5 opencv-python numpy scipy pyserial
```

### Run

1. Flash `fullcntl.ino` to Arduino Mega 2560 + RAMPS 1.4
2. Update serial port string in `fullcntl.py` → `init_serial()`
   - macOS: `/dev/cu.usbmodem*`
   - Linux: `/dev/ttyACM*`
   - Windows: `COM*`
3. `python fullcntl.py`

### Serial protocol (115200 baud, newline-terminated)

| Command | Action |
|---|---|
| `H` | Home all 6 joints, then drive to each joint's target angle |
| `S` | Shutdown: move all joints +10° from home |
| `R` | Reset: drive all joints to 0° |
| `M<a1>,<a2>,<a3>,<a4>,<a5>,<a6>` | Move to absolute joint angles in degrees (host inverts signs per joint convention before sending) |
| `OPEN` | Open BLDC gripper |
| `CLOSE` | Close BLDC gripper |

### IK details

- 6 DH rows (see top README for table)
- Cost: `‖p − p_target‖² + 10·‖z_ee − [0,0,-1]‖² + 0.1·Σ wᵢ (θᵢ − θᵢ_prev)²`
- Solver: SciPy `L-BFGS-B` with joint-limit bounds, warm-started from previous solution
- Joint-change weights: `[1.0, 1.1, 0.9, 1.4, 0.8, 1.0]`
- Z-compensation: quadratic fit through 6 calibration points compensates effector sag vs X

## `Tests/`

Bring-up + characterization scripts. See [`Tests/README.md`](Tests/README.md).
