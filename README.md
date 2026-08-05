# ChipSorter Motion Controller

This project is the motion-only controller for the poker chip sorter. It runs on an Arduino Uno with a Prontoneer CNC shield and three DRV8825 stepper drivers.

The controller owns:

- table rotation on the X axis
- chip pusher motion on the Y axis
- chip puller / dispenser motion on the Z axis
- limit switch handling for homing and safety
- serial status and command handling for the main vision processor

## Current build mode

The firmware is currently configured as a continuous motion test loop.

It repeatedly runs:

1. X forward one rotation
2. Y forward one rotation
3. Y backward one rotation
4. X backward one rotation
5. repeat forever

The test loop is enabled in [platformio.ini](platformio.ini) with `CHIP_SORTER_TEST_LOOP=1`.

To return to command-driven operation, remove that build flag or set it to `0` and rebuild.

## Hardware overview

Board and drivers:

- Arduino Uno
- Prontoneer CNC shield
- 3 x DRV8825 stepper driver modules

Motor roles:

- X axis: rotary table with 10 chip tubes
- Y axis: pusher mechanism that places a chip into the selected tube
- Z axis: puller / dispenser mechanism that releases a chip on command

## Wiring

### Stepper driver connections

The code uses these CNC shield / DRV8825 signal pins:

- X step: D2
- X direction: D5
- X limit switch: D9

- Y step: D3
- Y direction: D6
- Y limit switch: D10

- Z step: D4
- Z direction: D7
- Z limit switch: D11

- Stepper enable: D8

These match the constants in [include/ChipSorterConfig.h](include/ChipSorterConfig.h).

### UART connection

The motion controller can use either the Uno hardware serial port or `SoftwareSerial`, selected at build time.

Current project default:

- link serial mode: `SoftwareSerial`
- link pins: D12 (RX) and D13 (TX)
- link baud: 19200

Wire the main processor UART to the Uno serial pins:

Hardware serial mode:

- Uno D0 / RX <- TX from the main processor
- Uno D1 / TX -> RX on the main processor

Software serial mode:

- Uno D12 / RX <- TX from the main processor
- Uno D13 / TX -> RX on the main processor

In both cases:

- GND -> GND between both boards

Important notes:

- In hardware serial mode, the Uno serial port is shared with the USB interface used for uploading and serial monitoring.
- In software serial mode, the USB serial monitor remains available on D0/D1 while the main processor link uses D12/D13.
- The main processor is a 3.3 V device, so level shifting should be reviewed carefully before direct UART connection.
- Share ground between the Uno, the DRV8825 power supply, and the main processor.

Suggested build flag:

- `CHIP_SORTER_LINK_SERIAL_MODE=0` for D0/D1 hardware serial
- `CHIP_SORTER_LINK_SERIAL_MODE=1` for D12/D13 software serial
- `CHIP_SORTER_LINK_BAUD` can be overridden if you want a different link speed

### Motor power and driver setup

- Do not power the motors from the Uno 5 V rail.
- Provide the DRV8825 motor supply from the appropriate external supply for the steppers.
- Set the DRV8825 current limit before full operation.
- Confirm microstepping jumper settings on the CNC shield match your motion calibration.

## Limit switches

Limit switches are configured as active-low inputs with internal pull-ups enabled.

Current switch pins:

- X home / limit: D9
- Y home / limit: D10
- Z home / limit: D11

If your switch wiring is normally-open to ground, the current code matches that style.

## Motion calibration

Key motion constants live in [include/ChipSorterConfig.h](include/ChipSorterConfig.h):

- `kTableStepsPerTube`: steps required to move from one tube position to the next
- `kPusherStrokeSteps`: pusher travel distance
- `kMaxMotionSpeed`: top motion speed used by the stepper library
- `kMotionAcceleration`: acceleration used for moves

The table is currently configured for 10 tube positions.

## Serial protocol

The command-driven firmware path supports newline-delimited commands:

- `HOME`
- `MOVE_TUBE <n>`
- `PUSH`
- `STATUS`
- `STOP`

Expected responses:

- `OK <command>` when a command is accepted
- `DONE <command>` when motion completes
- `STATUS <homed> <current_tube> <busy>` for state reporting
- `ERR <reason>` for invalid requests or busy conditions

Additional asynchronous event messages:

- `LIMIT X 1` when X limit becomes active
- `LIMIT X 0` when X limit becomes inactive
- `LIMIT Y 1` when Y limit becomes active
- `LIMIT Y 0` when Y limit becomes inactive
- `LIMIT Z 1` when Z limit becomes active
- `LIMIT Z 0` when Z limit becomes inactive

The protocol is described in [docs/stepper_project_handoff.md](docs/stepper_project_handoff.md).

## Build

PlatformIO environment:

- environment name: `Uno-CNC_MotionController`
- framework: Arduino
- baud rate: 115200

Typical build command:

```bash
python -m platformio run
```

If you want to monitor serial output from the Uno, use the same baud rate configured in the project.

## Suggested next step

After the motion calibration is verified, the next useful change is to switch the firmware back from the test loop to the serial command mode so the vision processor can drive it directly.