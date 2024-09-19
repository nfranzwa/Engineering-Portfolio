# Custom 6 Degree of Freedom Robot Arm 

## Overview
This project implements a 6 Degree of Freedom (DOF) robotic arm with advanced control and vision capabilities. The arm combines custom software, CAD models, and easily accesible hardware, to perform complex manipulation tasks.

## Key Features
- 6 DOF movement for maximum flexibility
- Inverse kinematics for precise positioning
- Computer vision for object detection and tracking
- Gripper control for pick-and-place operations
- Custom GUI for easy control and monitoring

## Hardware
- Custom-designed 3D printed parts
- 6 stepper motors with TMC2209 drivers
- BLDC motor for the gripper
- Arduino-based control system
- Limit switches and sensors for homing and positioning

## Software
- Python-based main control software
- OpenCV for image processing and object detection
- Custom inverse kinematics solver
- Arduino firmware for low-level motor control
- PyQt5 GUI for user interaction

## Key Functionalities
1. **Precision Movement**: Utilizes inverse kinematics to accurately position the end effector.
2. **Object Detection**: Employs computer vision to identify and locate objects in the workspace.
3. **Pick and Place**: Combines vision and motion control for automated pick-and-place tasks.
4. **Homing Sequence**: Implements a robust homing routine for consistent positioning.

## Wiring and Pin Configuration
The robot uses a RAMPS 1.4 board with TMC2209 motor controllers. Here's an overview of the pin configuration:

- 6 TMC2209 motor controllers (5 on RAMPS 1.4, 1 on external board)
- 3 inductive sensors and 3 mechanical switches for limit detection
- Specific joint configurations (examples):
  - J1 (Base Joint): STEP_PIN: 54, DIR_PIN: 55, ENABLE_PIN: 38, Limit Switch: Pin 5
  - J2: STEP_PIN: 60, DIR_PIN: 61, ENABLE_PIN: 56, Limit Switch: Pin 3
  - (Other joints configured similarly)

![RAMPS 1.4 Pinout](https://github.com/user-attachments/assets/e9fb35c4-472a-451c-9f5b-543ad330c704)

For detailed pin assignments and wiring instructions, please refer to the `pins.h` file in the project repository.

## Demonstrations

### Move Demo
This video showcases the arm's precise movement capabilities:

[![Move Demo](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://github.com/user-attachments/assets/76c58376-af98-44d1-89f5-d9b5ce975651)

### Object Detection and Gripper Actuation
Here, the arm detects an object, picks it up, and places it in a designated location:

[![Object Detection and Gripper Actuation](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://github.com/user-attachments/assets/8c85c374-1d46-4252-8022-c0bc396d2eb2)

## Future Improvements
- Implement trajectory planning for smoother motion
- Enhance object recognition capabilities
- Develop more complex manipulation tasks

