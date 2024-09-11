
![IMG_3204](https://github.com/user-attachments/assets/6418f20c-f2b9-46b2-b623-9847ae49599c)
![electronics](https://github.com/user-attachments/assets/5dd10d63-feb1-4a9c-b234-279e1b881540)
![IMG_3206](https://github.com/user-attachments/assets/99c5d5a8-d399-4f37-b20c-db5fae20546b)


## Move Demo

[![Move Demo](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://github.com/user-attachments/assets/76c58376-af98-44d1-89f5-d9b5ce975651)

This video demonstrates the precision and accuracy of the 6 DOF robotic arm's end effector movement. The Python code utilizes inverse kinematics to calculate joint angles for desired end effector positions. It sends commands to the Arduino via serial communication, which then controls the stepper motors for each joint. The `AccelStepper` library manages smooth acceleration and deceleration, while custom functions ensure precise angle-to-step conversions for each joint's unique gear ratio.

## Object Detection and Gripper Actuation

[![Object Detection and Gripper Actuation](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://github.com/user-attachments/assets/8c85c374-1d46-4252-8022-c0bc396d2eb2)

This video showcases the robotic arm's ability to detect objects, pick them up, and drop them into a bowl. The Python script uses OpenCV for real-time image processing and object detection. Once an object is identified, inverse kinematics calculates the necessary joint angles to reach it. The Arduino code controls the BLDC motor-driven gripper, opening and closing it at appropriate times. The entire pick-and-place operation is coordinated through a series of timed commands, ensuring smooth transitions between detecting, gripping, moving, and releasing the object.

## Full Homing Sequence

[![Full Homing Sequence](https://img.youtube.com/vi/VIDEO_ID/0.jpg)](https://github.com/user-attachments/assets/09466da0-1334-4c38-82d4-a613223febb5)

This video presents the complete homing sequence of the robotic arm. The process, initiated by a command from the Python interface, is primarily handled by the Arduino code. Each joint moves towards its respective limit switch, using a combination of fast and slow approaches for accuracy. Once a limit switch is triggered, the joint backs off slightly and then slowly re-approaches to ensure precise homing. After all joints are homed, they move to predefined "zero" positions. This homing sequence is crucial for establishing a consistent reference point, enabling accurate absolute positioning during subsequent operations.
