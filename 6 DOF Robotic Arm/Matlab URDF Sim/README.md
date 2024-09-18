# 6 DOF Robotic Arm Control System

This project implements a control system for a 6 Degree of Freedom (DOF) robotic arm using MATLAB/Simulink and a custom GUI. The system allows for real-time control and visualization of the robotic arm's position.

## System Overview

The robotic arm consists of six joints, each controlled independently. The system is composed of three main components:

1. Simulink Model: Represents the physical structure and kinematics of the robotic arm.
2. MATLAB GUI: Provides an interface for user input and real-time feedback.
3. MATLAB Code: Handles the logic for updating the simulation and GUI.

### Simulink Model

![Simulink Model](https://github.com/user-attachments/assets/40ded6cc-df85-4cd9-a487-6a8478478037)

The Simulink model represents the robotic arm's structure and kinematics. It includes:

- Six "Slider Gain" blocks: These correspond to the six joints of the robotic arm.
- Kinematic calculations: Implemented using MATLAB Function blocks or other Simulink components.
- Output ports: Provide X, Y, and Z coordinates of the end-effector.

### MATLAB GUI

The GUI allows users to control the robotic arm interactively. It features:

- Six sliders and input fields: For adjusting each joint angle (range: -180° to 180°).
- Stop and Reset buttons: For simulation control.
- XYZ coordinate display: Shows the current position of the end-effector.

### MATLAB Code

The MATLAB code (`createParol6GUI()` function) manages the interaction between the GUI and the Simulink model. Key functions include:

- `updateSimulation()`: Updates the Simulink model with new joint angles.
- `sliderChanged()` and `fieldChanged()`: Handle user input from sliders and input fields.
- `stopSimulation()` and `resetSimulation()`: Control the simulation state.
- `getXYZCoordinates()`: Retrieves the current end-effector position from the Simulink model.
- `updateXYZCoordinates()`: Updates the GUI with the current end-effector position.

## How It Works

1. **Initialization**: 
   - The GUI is created with sliders, input fields, and buttons.
   - The Simulink model is loaded and started in normal mode.

2. **User Input**:
   - Users can adjust joint angles using sliders or input fields.
   - Each change triggers an update to the Simulink model.

3. **Simulation Update**:
   - New joint angles are applied to the "Slider Gain" blocks in the Simulink model.
   - The model recalculates the arm's kinematics based on the new joint angles.

4. **Position Feedback**:
   - The end-effector's XYZ coordinates are continuously read from the Simulink model's output ports.
   - These coordinates are displayed and updated in real-time on the GUI.

5. **Control Options**:
   - Users can stop the simulation, which resets all joint angles to 0 and closes the GUI.
   - The reset button sets all joint angles to 0 and restarts the simulation.

## Demonstration

The following video demonstrates the robot arm's actuation across its six joints:

[Robot Arm Demonstration Video](https://github.com/user-attachments/assets/2ef6d265-0591-4eab-a66c-312998d703f5)
