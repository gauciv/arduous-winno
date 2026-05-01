# Implementation Plan: Line Follower Button Control

## Overview

Rework `sumo_bot.ino` to default to line follower mode with a push button on D10 controlling calibration and start/stop via a state machine. Update motor driver and QTR sensor pin mappings, split QTR into two separate sensor objects with individual emitter pins, and add serial feedback at state transitions. All changes are in the single file `sumo_bot.ino`.

## Tasks

- [x] 1. Update pin definitions and motor driver wiring
  - [x] 1.1 Replace motor driver pin constants with new TB6612FNG mapping (PWMA→D9, AIN2→D8, AIN1→D7, STBY→D6, BIN1→D5, BIN2→D4, PWMB→D3)
    - Update `AIN1`, `AIN2`, `PWMA`, `BIN1`, `BIN2`, `PWMB`, `STBY_PIN` constants
    - Remove or comment out ultrasonic pin definitions (`TRIG1`, `ECHO1`, `TRIG2`, `ECHO2`) since those pins are now reassigned
    - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5, 5.6, 5.7_
  - [x] 1.2 Add line follower button pin definition and configure INPUT_PULLUP
    - Add `LF_BUTTON_PIN = 10` constant
    - Add `pinMode(LF_BUTTON_PIN, INPUT_PULLUP)` in `setup()`
    - _Requirements: 2.1, 2.2_

- [x] 2. Replace single QTR sensor object with dual QTR sensor boards
  - [x] 2.1 Remove the single `QTRSensors qtr` object and its 6-pin array; create two `QTRSensors` objects (`qtrLeft`, `qtrRight`) with separate emitter pins
    - `qtrLeft`: pins A3, A4, A5 with emitter on D12
    - `qtrRight`: pins A0, A1, A2 with emitter on D11
    - Add `SENSORS_PER_BOARD = 3`, `TOTAL_SENSORS = 6`, `LEFT_EMITTER_PIN = 12`, `RIGHT_EMITTER_PIN = 11`
    - Initialize both objects in `setup()` with `setTypeRC()`, `setSensorPins()`, `setEmitterPin()`
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5, 6.6_
  - [x] 2.2 Implement `readAllSensors()` and `readLinePosition()` helper functions
    - `readAllSensors()`: reads both boards into combined `sensorValues[6]` array
    - `readLinePosition()`: reads calibrated values from both boards, computes weighted-average line position (0–5000) replicating `readLineBlack()` logic
    - _Requirements: 6.7_

- [x] 3. Implement button state machine and calibration logic
  - [x] 3.1 Add `ButtonState` enum, state variables, debounce constant, and calibration timing variables
    - Define `BS_IDLE`, `BS_CALIBRATING`, `BS_CALIBRATED_IDLE`, `BS_RUNNING`, `BS_STOPPED`
    - Add `btnState`, `isCalibrated`, `DEBOUNCE_MS = 200`, `lastButtonPress`, `calibrationStart`
    - Add `CALIBRATION_DURATION_MS = 10000` as a named constant in the tuning parameters section
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 4.1, 4.2_
  - [x] 3.2 Implement `buttonPressed()` debounce function
    - Read D10, return true only if LOW and at least `DEBOUNCE_MS` since last accepted press
    - _Requirements: 2.3_
  - [x] 3.3 Implement `handleButtonStateMachine()` with all state transitions
    - BS_IDLE → BS_CALIBRATING on press (if not calibrated)
    - BS_CALIBRATING → BS_CALIBRATED_IDLE when timer expires
    - BS_CALIBRATED_IDLE → BS_RUNNING on press
    - BS_RUNNING → BS_STOPPED on press (stop motors)
    - BS_STOPPED → BS_RUNNING on press
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5, 3.6_
  - [x] 3.4 Implement `runCalibrationSweep()` non-blocking calibration function
    - First half of `CALIBRATION_DURATION_MS`: CW motor sweep; second half: CCW
    - Call `qtrLeft.calibrate()` and `qtrRight.calibrate()` each tick
    - Stop motors when calibration completes
    - _Requirements: 4.3, 4.4_

- [x] 4. Add serial feedback at all state transitions
  - Print boot message with mode = LINE_FOLLOW and calibration status
  - Print calibration start message with configured duration
  - Print calibration complete message
  - Print line following started/stopped messages
  - All serial output at 115200 baud
  - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5_

- [x] 5. Checkpoint - Verify state machine and sensor setup
  - Ensure all tests pass, ask the user if questions arise.

- [x] 6. Modify setup() and loop() to wire everything together
  - [x] 6.1 Update `setup()`: use new pin mappings, initialize dual QTR objects, configure button pin, comment out `selectMode()` call and `calibrateQTR()` call, default `currentMode = LINE_FOLLOW`, print boot serial messages
    - Preserve commented-out `selectMode()` code for future re-enablement
    - _Requirements: 1.1, 1.2, 1.3, 5.8, 7.1_
  - [x] 6.2 Simplify `loop()` to call `handleButtonStateMachine()` only; line following runs inside the state machine when `btnState == BS_RUNNING`
    - Remove edge detection and mode switch from `loop()` (line follow mode doesn't use edge detection)
    - _Requirements: 3.3, 3.4, 3.5_
  - [x] 6.3 Update `runLineFollow()` to use `readLinePosition()` instead of `qtr.readLineBlack()`
    - PID logic remains unchanged, only the sensor read call changes
    - _Requirements: 6.7_

- [x] 7. Final checkpoint - Ensure all changes compile and are consistent
  - Ensure all tests pass, ask the user if questions arise.
  - Verify all pin definitions match the requirements
  - Verify the state machine covers all transitions from the design
  - Verify serial messages are present at every state transition

## Notes

- All changes are confined to the single file `sumo_bot.ino`
- No new libraries are introduced beyond the existing `QTRSensors` dependency
- The existing `selectMode()` function body is preserved as commented-out code
- Calibration duration is a named constant (`CALIBRATION_DURATION_MS`) in the tuning parameters section for easy adjustment
- The combined sensor read uses a manual weighted-average calculation since two separate QTRSensors objects cannot use the library's built-in `readLineBlack()` across both boards
