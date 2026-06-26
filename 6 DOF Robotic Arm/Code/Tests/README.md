# Tests

Standalone bring-up + characterization scripts. Each one runs independently from the main control loop.

| File | Type | Purpose |
|---|---|---|
| `switchtest.ino` | Arduino | Read mechanical limit switch on pin 3 (NC, `INPUT_PULLUP`). Prints open/closed every 100 ms. |
| `inductivetest.ino` | Arduino | Read inductive proximity sensor on pin 4. Prints "Object detected!" on LOW. |
| `posol.ino` | Arduino | SimpleFOC BLDC gripper, **angle open-loop**. Commander interface: `T<rad>` target angle, `L<V>` voltage limit, `V<rad/s>` velocity limit. 24 V supply, 20 V limit, 5 rad/s default. |
| `voltageol.ino` | Arduino | SimpleFOC BLDC gripper, **velocity open-loop**. Commander interface: `T<rad/s>` target velocity, `C<A>` current limit. 0.5 A default. |
| `cameratest.py` | Python | Webcam + Haar cascade face detector sanity check. Draws box, prints center + side length. Quit with `q`. |
| `compvision.py` | Python | Workspace calibration + object tracking. Stabilizes the 6 corners of a dark box over the first 5 s, builds a perspective transform to 4 reference points in mm, then uses MOG2 background subtraction to track a moving object and report (Y, X) in workspace mm. |
| `pathplanning.py` | Python | Estimate longest joint-move time given user-entered target angles, motor speeds, accels, gear ratios, motor torque and power budget. Used to size host-side wait durations. |
| `jointime.py` | Python | Standalone IK test harness. Same DH params + joint limits as `fullcntl.py`, simpler GUI (no vision). Includes quadratic Z-compensation vs X. |

## Running

- `.ino` files: open in Arduino IDE, set board to Mega 2560, upload, open Serial Monitor at 9600 (limit-switch tests) or 115200 (SimpleFOC).
- `.py` files: `python <name>.py`. Vision scripts require a connected webcam.
