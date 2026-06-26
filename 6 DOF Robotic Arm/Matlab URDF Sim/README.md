# MATLAB URDF Simulation

Standalone kinematic simulator for the arm, independent from the physical robot's firmware + Python control stack. Useful for visualizing joint motion and verifying end-effector poses without hardware.

## Files

| File | Role |
|---|---|
| `arm.urdf` | URDF model of the 6-DOF arm |
| `L1.STL` through `L6.STL` | Link meshes referenced by the URDF |
| `arm.slx` | Simulink model wrapping the URDF with Slider Gain blocks per joint |
| `DHpara.mat` | DH parameters used by the kinematic blocks |
| `RobotSim.m` | Core simulation logic |
| `RoboticArmGUI.m` | MATLAB-figure GUI (sliders, input fields, reset / stop, XYZ readout) |

## How to run

1. Open this folder in MATLAB
2. Open `arm.slx` once so Simulink loads the model
3. Run `RoboticArmGUI` from the MATLAB command window
4. Move sliders or type into the joint-angle fields; XYZ readout updates from the model output ports

## How it works

- GUI updates 6 Slider Gain blocks → Simulink recomputes kinematics → output ports (X, Y, Z) of the end-effector → display refresh.
- **Stop** zeros all joint angles and closes the GUI. **Reset** zeros joint angles and restarts the simulation.

## Demo

[Joint actuation demo](https://github.com/user-attachments/assets/2ef6d265-0591-4eab-a66c-312998d703f5)

![Simulink model](https://github.com/user-attachments/assets/40ded6cc-df85-4cd9-a487-6a8478478037)
