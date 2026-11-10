# PAC1 Robot ("Roxanne")

Control code for an autonomous robot built by the PAC1 team for the Ohio State Fundamentals of Engineering Honors (FEH) robot competition, spring 2019. The robot runs on the FEH Proteus controller and drives the competition course by itself, using shaft encoders, a CdS light sensor and the course's Robot Positioning System (RPS).

## What the robot does

When **Run Course** is chosen from the touchscreen menu, the robot waits for the start light and then:

1. Drives to the DDR station, reads the light color with the CdS cell and presses the matching button
2. Climbs the ramp
3. Drives to the coin slot and drops the coin
4. Slides the foosball counters with its servo arm
5. Flips the lever
6. Returns to the top of the ramp and drives down to press the final button

## Code overview

Everything is in [`main.cpp`](main.cpp):

| Area | Functions |
|---|---|
| Movement | `goForward`, `goForwardRamp`, `turnLeft`, `turnRight` (encoder-based) |
| RPS correction | `check_heading`, `check_x`, `check_y` nudge the robot to an exact heading or coordinate |
| Course tasks | `goToDDRLight`, `pressDDRButton`, `goUpRamp`, `goToCoin`, `dropCoin`, `goToFoosball`, `moveFoos`, `goToLeverFromFoos`, `flipLever`, `goToRampTop`, `goToFinalButtonTheta` |
| Touchscreen menu | `loadMenu`, `drawMenu`, `openMenu`: run the course, recalibrate key RPS points, show live RPS/CdS readings, or test forward/turn moves |

Tuning constants such as wheel diameter, encoder counts per inch and degree, motor speed and the default RPS coordinates are `#define`s at the top of the file.

## Building

The code uses the FEH libraries (`FEHLCD`, `FEHIO`, `FEHMotor`, `FEHServo`, `FEHRPS`, `FEHUtility`) and is built with the FEH Proteus toolchain provided by the course. It won't compile as a standalone C++ program.

## Hardware

- FEH Proteus controller with touchscreen
- 2 drive motors (ports 0 and 1) with digital encoders (P2_7 right, P0_0 left)
- CdS cell on P2_0 for the start light and the DDR light
- Servo arm on servo port 7
