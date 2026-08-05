# ChipSorter stepper-controller handoff

This note describes the current serial contract between the vision controller and the future stepper-controller project.

## Purpose
The ESP32-S3 vision board is responsible for:
- capturing images
- recognizing the chip pattern
- deciding which tube to use
- sending motion commands over serial to a separate controller

The separate stepper-controller project should own:
- stepper motor motion
- tube indexing
- pusher actuation
- homing and safety behavior

## Current serial contract
The ESP32-S3 sends newline-delimited commands over Serial2.

Supported commands:
- `HOME`
- `MOVE_TUBE <n>`
- `PUSH`
- `STATUS`
- `STOP`

Example:
- `HOME`
- `MOVE_TUBE 3`
- `PUSH`
- `STATUS`

Expected response format:
- `OK <command>` for accepted commands
- `DONE <command>` when a command finishes
- `STATUS <homed> <current_tube> <busy>` for status reports
- `ERR <reason>` for invalid requests

## ESP32-side implementation notes
The current ESP32 firmware includes a minimal stub in:
- [src/motion_link.cpp](src/motion_link.cpp)
- [src/motion_link.h](src/motion_link.h)

The serial link uses:
- TX: GPIO 17
- RX: GPIO 16
- baud: 115200

The main firmware calls the link handlers from the main loop.

## Suggested next steps for the separate project
1. Create a new Arduino/PlatformIO project for the stepper controller.
2. Read newline-delimited commands from serial.
3. Implement `HOME` by moving the indexer to a known zero position.
4. Implement `MOVE_TUBE <n>` by rotating the revolver to the requested tube.
5. Implement `PUSH` by actuating the chip pusher.
6. Return `DONE` and `STATUS` messages so the ESP32 can track progress.
7. Add safety features such as:
   - limit switch checks
   - timeout handling
   - emergency stop handling

## Hardware notes
The separate controller is expected to run on an Arduino-compatible board with a CNC shield and stepper drivers.

Typical wiring assumptions:
- stepper driver enable/step/direction pins
- homing limit switch
- pusher actuator output
- common ground shared with the ESP32
- UART TX/RX connected to the ESP32 serial link

## Recommended first milestone
Build a minimal controller that can:
- receive `HOME`
- respond `OK HOME`
- receive `MOVE_TUBE 3`
- respond `DONE 3`
- receive `PUSH`
- respond `DONE PUSH`

That gives you a reliable, testable protocol before full motion integration.
