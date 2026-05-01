# Requirements Document

## Introduction

Rework the sumo bot's line follower mode to operate as the default mode with a dedicated push button (D10) controlling calibration and start/stop behavior. The boot-time mode selection is disabled (commented out), pin mappings are updated for the TB6612FNG motor driver and two QTR-MD-03RC sensor boards with separate emitter/control pins, and calibration duration is made configurable.

## Glossary

- **Bot**: The Arduino Nano-based sumo bot running the firmware in `sumo_bot.ino`
- **Line_Follower_Button**: The push button connected to digital pin D10 that controls line follower start, stop, and calibration
- **Calibration**: The QTR sensor calibration process where sensors sweep over black and white surfaces to learn min/max reflectance values
- **Calibration_Duration**: A configurable variable (default 15 seconds) controlling how long the calibration sweep runs
- **Left_QTR**: The left QTR-MD-03RC sensor board with sensor pins on A3, A4, A5 and odd control/emitter pin on D12
- **Right_QTR**: The right QTR-MD-03RC sensor board with sensor pins on A0, A1, A2 and odd control/emitter pin on D11
- **Sensor_Array**: The combined 6-sensor array formed by Left_QTR (indices 0–2) and Right_QTR (indices 3–5), ordered left-to-right
- **Motor_Driver**: The TB6612FNG motor driver with updated pin assignments (PWMA→D9, AIN2→D8, AIN1→D7, STBY→D6, BIN1→D5, BIN2→D4, PWMB→D3)
- **Left_Motor**: Channel A of the Motor_Driver controlling the left wheel
- **Right_Motor**: Channel B of the Motor_Driver controlling the right wheel
- **Button_State_Machine**: The state logic governing the Line_Follower_Button behavior across presses: IDLE → CALIBRATING/RUNNING → STOPPED → RUNNING (toggle)
- **Mode_Selection**: The existing boot-time mode selection logic (selectMode function) that reads a button press to choose SUMO, LINE_FOLLOW, or BALLOON_POP

## Requirements

### Requirement 1: Disable Boot-Time Mode Selection

**User Story:** As a developer, I want the boot-time mode selection to be commented out and the Bot to default to line follower mode, so that the Bot always starts in line follower mode without requiring a boot-time button press.

#### Acceptance Criteria

1. THE Bot SHALL default the operating mode to LINE_FOLLOW on every boot
2. THE Bot SHALL have the Mode_Selection logic (selectMode function and its call in setup) commented out in the source code
3. THE Bot SHALL retain the commented-out Mode_Selection code for future re-enablement

### Requirement 2: Line Follower Button Pin Assignment

**User Story:** As a developer, I want the line follower start/stop button on digital pin D10, so that I have a dedicated physical control for the line follower mode.

#### Acceptance Criteria

1. THE Bot SHALL configure digital pin D10 as an input for the Line_Follower_Button
2. THE Bot SHALL use INPUT_PULLUP configuration for pin D10 so that the button reads LOW when pressed and HIGH when released
3. THE Bot SHALL debounce the Line_Follower_Button with a minimum interval of 200 milliseconds between accepted presses

### Requirement 3: Button-Controlled Calibration and Start/Stop

**User Story:** As a developer, I want the push button to handle calibration awareness and start/stop toggling, so that a single button manages the full line follower lifecycle.

#### Acceptance Criteria

1. WHEN the Line_Follower_Button is pressed and the Sensor_Array is not calibrated, THE Bot SHALL begin the Calibration process for the configured Calibration_Duration
2. WHEN the Calibration process completes, THE Bot SHALL stop the motors and wait for the next button press
3. WHEN the Line_Follower_Button is pressed after Calibration is complete and the Bot is not running, THE Bot SHALL start line following
4. WHEN the Line_Follower_Button is pressed while the Bot is running line follow, THE Bot SHALL stop the motors and enter a stopped state
5. WHEN the Line_Follower_Button is pressed while the Bot is in a stopped state, THE Bot SHALL resume line following
6. WHEN the Line_Follower_Button is pressed and the Sensor_Array is already calibrated, THE Bot SHALL skip Calibration and immediately start or stop line following based on the current state

### Requirement 4: Configurable Calibration Duration

**User Story:** As a developer, I want the calibration duration stored in an easily changeable variable, so that I can quickly adjust calibration time without searching through the code.

#### Acceptance Criteria

1. THE Bot SHALL store the Calibration_Duration as a named constant or variable at the top of the source file in the tuning parameters section
2. THE Bot SHALL default the Calibration_Duration to 15 seconds
3. WHEN Calibration runs, THE Bot SHALL sweep the motors (alternating clockwise and counter-clockwise) for the full Calibration_Duration while calling the QTR calibrate function repeatedly
4. WHEN Calibration completes, THE Bot SHALL stop the motors

### Requirement 5: Updated Motor Driver Pin Mapping

**User Story:** As a developer, I want the motor driver pin definitions updated to match the new physical wiring, so that the firmware correctly drives the motors through the TB6612FNG.

#### Acceptance Criteria

1. THE Motor_Driver SHALL use pin D9 for PWMA (Left_Motor speed)
2. THE Motor_Driver SHALL use pin D8 for AIN2 (Left_Motor direction)
3. THE Motor_Driver SHALL use pin D7 for AIN1 (Left_Motor direction)
4. THE Motor_Driver SHALL use pin D6 for STBY (standby control)
5. THE Motor_Driver SHALL use pin D5 for BIN1 (Right_Motor direction)
6. THE Motor_Driver SHALL use pin D4 for BIN2 (Right_Motor direction)
7. THE Motor_Driver SHALL use pin D3 for PWMB (Right_Motor speed)
8. THE Bot SHALL set the STBY pin HIGH during setup to enable the Motor_Driver

### Requirement 6: Updated QTR Sensor Pin Mapping with Separate Emitter Pins

**User Story:** As a developer, I want the QTR sensor pins updated to match the two QTR-MD-03RC boards with their respective emitter/control pins, so that the firmware reads all six sensors correctly.

#### Acceptance Criteria

1. THE Left_QTR SHALL use analog pins A3, A4, A5 for its three sensor inputs (corresponding to physical sensor pins 5, 3, 1)
2. THE Left_QTR SHALL use digital pin D12 as its odd emitter/control pin
3. THE Right_QTR SHALL use analog pins A0, A1, A2 for its three sensor inputs (corresponding to physical sensor pins 5, 3, 1)
4. THE Right_QTR SHALL use digital pin D11 as its odd emitter/control pin
5. THE Sensor_Array SHALL be ordered left-to-right as: A3, A4, A5, A0, A1, A2 (Left_QTR sensors followed by Right_QTR sensors)
6. THE Bot SHALL configure two separate QTRSensors objects, one for Left_QTR and one for Right_QTR, each with its own emitter pin
7. WHEN reading the Sensor_Array, THE Bot SHALL combine readings from both QTRSensors objects into a single 6-element array for PID line position calculation

### Requirement 7: Button State Machine Serial Feedback

**User Story:** As a developer, I want serial output at each state transition, so that I can monitor the button state machine behavior via the Serial Monitor during testing.

#### Acceptance Criteria

1. WHEN the Bot boots, THE Bot SHALL print the current mode (LINE_FOLLOW) and calibration status to Serial at 115200 baud
2. WHEN the Line_Follower_Button triggers Calibration, THE Bot SHALL print a message indicating Calibration has started and the configured Calibration_Duration
3. WHEN Calibration completes, THE Bot SHALL print a message indicating Calibration is complete and the Bot is waiting for a button press to start
4. WHEN the Bot starts line following, THE Bot SHALL print a message indicating line following has started
5. WHEN the Bot stops line following, THE Bot SHALL print a message indicating line following has stopped
