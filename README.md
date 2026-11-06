# Line Follower Transporter Robot

This repository contains the programs and development files for a line follower transporter robot. The project was developed through several iterations, starting from basic motor and sensor tests and progressing to line-following control, route mapping, and object handling.

The repository includes multiple sketches and versions used during development. **The latest program is `lineFollowerPD_v4FullMapping`**, located in the `lineFollowerPD_v4FullMapping` folder.

## Project Overview

The robot follows a track using an array of infrared line sensors. Its movement is controlled by a PD-based line-following algorithm, while a route-mapping system handles turns and actions at intersections.

A gripper mechanism is used to pick up and place objects along the route. The robot also uses indicator LEDs and push buttons to support operation, calibration, and status monitoring.

## Main Features

- Line following using eight analog sensors.
- PD control for steering correction.
- Sensor calibration with calibration data saved in EEPROM.
- Route mapping with multiple selectable plans.
- Automatic intersection detection and programmed turning actions.
- Gripper mechanism for picking up and releasing objects.
- LED indicators for operating states and robot actions.
- Start, stop, and calibration controls.

## Hardware Overview

The project uses the following hardware as its main system components:

- Arduino-compatible microcontroller board.
- Eight-channel analog line sensor array.
- Dual DC motors and motor driver.
- Two servo motors for the gripper mechanism.
- WS2812 addressable LED strip.
- Push buttons for calibration, start, and stop.
- Battery and suitable power regulation for the electronics and motors.
- Robot chassis, wheels, and gripper assembly.

The exact wiring and component specifications may vary between development versions. Refer to the pin definitions in the relevant program before assembling or modifying the hardware.

## Software

The programs are written for the Arduino environment. The latest V4 program uses these libraries:

- **Adafruit NeoPixel** – controls the WS2812 LED strip.
- **EEPROM** – stores sensor calibration values.
- **Adafruit TiCoServo** – controls the gripper servos.

Install the required libraries through the Arduino IDE Library Manager, where available, before compiling the program.

## Repository Contents

Several folders and ZIP archives are included in this repository. They represent different stages of development, experiments, and supporting programs, such as:

- Basic motor and sensor testing.
- Initial line follower programs.
- PD-based line following.
- Versions with servo control and route mapping.
- Later mapping versions, including V3 and V4.

These files are kept as development references. For the most complete and recent implementation, use:

```text
lineFollowerPD_v4FullMapping/
├── lineFollowerPD_v4FullMapping.ino
├── mapping.h
└── servo_config.h
```

Keep the files in the same folder when opening the sketch in the Arduino IDE, since the main program uses the accompanying header files.

## Getting Started

1. Install the Arduino IDE.
2. Install the libraries required by the selected program.
3. Download or clone this repository.
4. Open `lineFollowerPD_v4FullMapping.ino` from the `lineFollowerPD_v4FullMapping` folder.
5. Select the appropriate Arduino board and port.
6. Check the pin assignments and hardware connections in the source files.
7. Compile the sketch and upload it to the microcontroller.
8. Calibrate the sensors before running the robot on the track.

## Operating the V4 Program

The V4 program uses three push buttons:

| Button | Pin | Function |
|---|---:|---|
| CALIBRATE | D2 | Hold to calibrate the sensors; press briefly to select the next route plan while idle. |
| START | D4 | Start the robot. |
| STOP | D12 | Stop the robot; while idle, briefly press to select the previous route plan. |

The robot has eight selectable route plans. The route sequence and actions are configured in `mapping.h`, particularly in the `executeMapStep()` function.

Servo positions and movement settings can be adjusted in `servo_config.h` and `mapping.h`. Sensor and motor settings, including PD parameters and base speed, are defined near the beginning of the main `.ino` file.

Before operating the robot, check the route configuration, sensor calibration, motor direction, servo positions, and movement durations. Values may need to be tuned to match the actual robot, track, and payload.

## Notes

- V4 is the latest version in this repository; older folders and ZIP archives are retained as references from the development process.
- Do not combine files from different versions unless their compatibility has been checked.
- Test the motors, sensors, and gripper separately before running the complete route.
- Secure the robot and keep the wheels clear of the floor during initial motor-direction tests.
- Use an appropriate power supply for the motors and servos. Avoid powering high-current loads directly from the microcontroller's 5 V pin.
- Route timing and PD parameters may require adjustment depending on the track surface, turns, battery condition, and payload.

---

This repository serves as a collection of the robot's development programs, with `lineFollowerPD_v4FullMapping` as the latest integrated version.
