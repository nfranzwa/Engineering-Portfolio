# Vids and Photos

Demo media for the arm.

![Assembled arm](https://github.com/user-attachments/assets/6418f20c-f2b9-46b2-b623-9847ae49599c)
![Electronics](https://github.com/user-attachments/assets/5dd10d63-feb1-4a9c-b234-279e1b881540)
![Build detail](https://github.com/user-attachments/assets/99c5d5a8-d399-4f37-b20c-db5fae20546b)

## Move demo

End-effector tracking a commanded (X, Y, Z). Host runs L-BFGS-B IK, sends joint angles over serial; firmware uses AccelStepper for smooth accel / decel.

https://github.com/user-attachments/assets/76c58376-af98-44d1-89f5-d9b5ce975651

## Object detection + pick-and-place

OpenCV thread on the host detects the object inside a calibrated workspace box (perspective transform from 4 reference points in mm). Host runs IK, firmware drives joints + the BLDC gripper open/close.

https://github.com/user-attachments/assets/8c85c374-1d46-4252-8022-c0bc396d2eb2

## Full homing sequence

Each joint: fast approach to limit switch → back off `backOffAngle` → slow re-approach for repeatable zero. J1–J3 home in parallel, then J4 + J6 in parallel, then J5. After homing, joints drive to per-joint `targetAngle` from `motorcst.h`.

https://github.com/user-attachments/assets/09466da0-1334-4c38-82d4-a613223febb5
