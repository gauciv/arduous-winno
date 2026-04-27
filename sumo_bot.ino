/*
 * ============================================================
 *  MULTI-PURPOSE SUMO BOT — PRODUCTION CODE
 *  Modes: SUMO | LINE_FOLLOW | BALLOON_POP
 * ============================================================
 *  Hardware:
 *    Arduino Nano
 *    TB6612FNG Motor Driver
 *    2x Pololu QTR-3RC (line/edge detection)
 *    2x Sharp IR GP2Y0A21 (10–80cm, analog)
 *    2x HC-SR04 Ultrasonic Sensors
 *    2x 6V 200RPM DC Motors
 *    LiPo 7.4V XT60
 *
 *  REQUIRED LIBRARY:
 *    QTRSensors by Pololu — install via Arduino Library Manager
 *
 * ============================================================
 *  WIRING GUIDE (MUST match your physical assembly)
 * ============================================================
 *
 *  TB6612FNG:
 *    VM   → Battery (+) through switch
 *    VCC  → Arduino 5V
 *    GND  → Arduino GND
 *    STBY → D13
 *    PWMA → D5  (PWM, Left motor speed)
 *    AIN1 → D4  (Left motor direction)
 *    AIN2 → D7  (Left motor direction)
 *    PWMB → D6  (PWM, Right motor speed)
 *    BIN1 → D8  (Right motor direction)
 *    BIN2 → D12 (Right motor direction)
 *    AO1/AO2 → Left Motor terminals
 *    BO1/BO2 → Right Motor terminals
 *
 *  Ultrasonic 1 (front-left):
 *    TRIG → D2,  ECHO → D3
 *  Ultrasonic 2 (front-right):
 *    TRIG → D9,  ECHO → D10
 *
 *  QTR-3RC Left sensor array:  D11, A0, A1
 *  QTR-3RC Right sensor array: A2,  A3, A4
 *    (sensor order: leftmost=index 0, rightmost=index 5)
 *
 *  Sharp IR Left  → A5 (analog)
 *  Sharp IR Right → A6 (analog)
 *    NOTE: A6 on Nano is ANALOG ONLY — do not use as digital
 *
 *  Mode Select Button → A7 (analog-only pin, read via analogRead)
 *    Wire: A7 — [10kΩ] — GND, and A7 — Button — 5V
 *    No press = SUMO (default)
 *    Short press (<1s) on boot = LINE_FOLLOW
 *    Long press  (>1s) on boot = BALLOON_POP
 *
 * ============================================================
 *  !! FIRST THING TO VERIFY AFTER UPLOAD !!
 *  Run with motors disconnected first.
 *  Open Serial Monitor at 115200 baud.
 *  Check sensor readings before a live run.
 * ============================================================
 */

#include <QTRSensors.h>

// ─────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────────────────────────

// Motor driver
const uint8_t AIN1 = 4,  AIN2 = 7,  PWMA = 5;   // Left motor
const uint8_t BIN1 = 8,  BIN2 = 12, PWMB = 6;   // Right motor
const uint8_t STBY_PIN = 13;

// Ultrasonic
const uint8_t TRIG1 = 2, ECHO1 = 3;              // Front-left
const uint8_t TRIG2 = 9, ECHO2 = 10;             // Front-right

// Sharp IR (analog)
const uint8_t IR_LEFT  = A5;
const uint8_t IR_RIGHT = A6;

// Mode select button
const uint8_t MODE_BTN = A7;

// ─────────────────────────────────────────────────────────────
//  QTR SENSOR SETUP
// ─────────────────────────────────────────────────────────────
QTRSensors qtr;
const uint8_t SENSOR_COUNT = 6;
// Order: [0]=far-left ... [5]=far-right
uint8_t sensorPins[SENSOR_COUNT] = {11, A0, A1, A2, A3, A4};
uint16_t sensorValues[SENSOR_COUNT];

// ─────────────────────────────────────────────────────────────
//  OPERATING MODES
// ─────────────────────────────────────────────────────────────
enum Mode : uint8_t { SUMO, LINE_FOLLOW, BALLOON_POP };
Mode currentMode = SUMO;

// ─────────────────────────────────────────────────────────────
//  TUNING PARAMETERS — adjust these after bench testing
// ─────────────────────────────────────────────────────────────

// --- Line Follow PID ---
// Start with KI = 0, tune KP until it tracks, then add KD to damp oscillation
float KP        = 0.28f;
float KI        = 0.00008f;
float KD        = 2.8f;
int   BASE_SPEED  = 150;    // Normal forward speed (0–255)
int   MAX_SPEED   = 210;    // Speed cap to keep line tracking stable
float integralMax = 5000.0f; // Anti-windup clamp

// --- Sumo ---
int SUMO_FULL_SPEED  = 255;  // Charge speed
int SUMO_TURN_SPEED  = 160;  // Rotation speed during search
int CHARGE_DIST_CM   = 50;   // Ultrasonic: charge if enemy within this
int SEARCH_INTERVAL  = 1400; // ms before reversing search spin direction

// --- Balloon Pop ---
int BALLOON_APPROACH_SPEED = 180;
int BALLOON_RAM_SPEED      = 255;
int BALLOON_RAM_DURATION   = 700;  // ms of full-speed ram
int BALLOON_SCAN_SPEED     = 120;
int BALLOON_DETECT_DIST    = 55;   // cm

// --- Sharp IR ---
// GP2Y0A21: higher ADC = closer object.
// ~490 at 10cm, ~150 at 40cm, ~80 at 80cm (values vary per unit)
// Tune by holding hand at ~30cm and reading Serial Monitor
int IR_THRESHOLD      = 180;  // ADC value above this = obstacle detected
int IR_VERY_CLOSE     = 420;  // ADC value = object at ~12cm (ram trigger)

// --- Edge Detection ---
// QTR reads ~800–1000 over white/reflective sumo ring border
// QTR reads ~100–300 over black ring surface
uint16_t EDGE_THRESHOLD   = 700;  // above this = white edge detected
int      BACKUP_DURATION  = 280;  // ms to reverse when edge found

// ─────────────────────────────────────────────────────────────
//  STATE VARIABLES
// ─────────────────────────────────────────────────────────────

// PID
float pidIntegral  = 0.0f;
float pidLastError = 0.0f;

// Edge recovery
bool          isRecovering    = false;
unsigned long recoverStart    = 0;
int           recoverLeftSpd  = 0;
int           recoverRightSpd = 0;

// Sumo state machine
enum SumoState : uint8_t { S_SEARCH, S_CHARGE };
SumoState sumoState     = S_SEARCH;
unsigned long sumoTimer = 0;
int8_t searchDir        = 1;     // +1 = clockwise, -1 = counter

// Balloon state machine
enum BalloonState : uint8_t { B_SCAN, B_APPROACH, B_RAM };
BalloonState balloonState = B_SCAN;
unsigned long balloonTimer = 0;
int8_t balloonScanDir = 1;

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Sumo Bot Booting ==="));

  // Motor driver
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
  stopMotors();

  // Ultrasonic
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);

  // QTR sensor configuration (RC = digital timing)
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SENSOR_COUNT);
  // If your QTR boards have a shared emitter pin, set it here:
  // qtr.setEmitterPin(YOUR_EMITTER_PIN);

  // ── Mode selection (3-second window on boot) ──
  Serial.println(F("Mode select window: 3s"));
  Serial.println(F("  No press  → SUMO"));
  Serial.println(F("  Short press → LINE FOLLOW"));
  Serial.println(F("  Long press  → BALLOON POP"));
  selectMode();

  Serial.print(F("Mode selected: "));
  switch (currentMode) {
    case SUMO:        Serial.println(F("SUMO"));        break;
    case LINE_FOLLOW: Serial.println(F("LINE FOLLOW")); break;
    case BALLOON_POP: Serial.println(F("BALLOON POP")); break;
  }

  // ── QTR calibration (line follow only) ──
  if (currentMode == LINE_FOLLOW) {
    calibrateQTR();
  }

  // ── Quick sensor self-check ──
  printSensorDiag();

  Serial.println(F("=== Starting in 2s ==="));
  delay(2000);
}

// ─────────────────────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  // ── Edge detection: highest priority, runs every tick ──
  if (!isRecovering) {
    if (detectEdge()) {
      startEdgeRecovery();
      return;
    }
  }
  if (isRecovering) {
    if (millis() - recoverStart >= (unsigned long)BACKUP_DURATION) {
      isRecovering = false;
      stopMotors();
      delay(50);
    }
    return; // Don't run mode logic while recovering
  }

  // ── Mode logic ──
  switch (currentMode) {
    case LINE_FOLLOW: runLineFollow();  break;
    case SUMO:        runSumo();        break;
    case BALLOON_POP: runBalloonPop();  break;
  }
}

// ─────────────────────────────────────────────────────────────
//  MODE SELECTION
// ─────────────────────────────────────────────────────────────
void selectMode() {
  unsigned long windowStart = millis();
  unsigned long pressStart  = 0;
  bool pressing = false;
  currentMode = SUMO; // default

  while (millis() - windowStart < 3000) {
    bool btnHeld = (analogRead(MODE_BTN) > 512);

    if (btnHeld && !pressing) {
      pressing   = true;
      pressStart = millis();
    }

    if (!btnHeld && pressing) {
      unsigned long held = millis() - pressStart;
      currentMode = (held < 1000) ? LINE_FOLLOW : BALLOON_POP;
      break;
    }
    delay(10);
  }
}

// ─────────────────────────────────────────────────────────────
//  QTR CALIBRATION
// ─────────────────────────────────────────────────────────────
void calibrateQTR() {
  Serial.println(F("Calibrating QTR — sweeping over line for 3s..."));
  // Spin CW then CCW so all sensors cross both black and white surfaces
  for (int i = 0; i < 80; i++) {
    setMotors(110, -110);
    qtr.calibrate();
    delay(15);
  }
  for (int i = 0; i < 80; i++) {
    setMotors(-110, 110);
    qtr.calibrate();
    delay(15);
  }
  stopMotors();
  delay(200);
  Serial.println(F("Calibration complete."));
}

// ─────────────────────────────────────────────────────────────
//  EDGE DETECTION
// ─────────────────────────────────────────────────────────────
// Sumo rings have a white border. High QTR reading = white = edge.
// For line follow: white = track surface, different handling applies.
bool edgeTriggeredLeft  = false;
bool edgeTriggeredRight = false;

bool detectEdge() {
  // Raw RC read is faster than calibrated read for edge detection
  qtr.read(sensorValues);

  edgeTriggeredLeft  = (sensorValues[0] > EDGE_THRESHOLD || sensorValues[1] > EDGE_THRESHOLD);
  edgeTriggeredRight = (sensorValues[4] > EDGE_THRESHOLD || sensorValues[5] > EDGE_THRESHOLD);

  // In LINE FOLLOW mode, edge detection is disabled (white IS the track background)
  if (currentMode == LINE_FOLLOW) return false;

  return edgeTriggeredLeft || edgeTriggeredRight;
}

void startEdgeRecovery() {
  isRecovering  = true;
  recoverStart  = millis();

  // Reverse hard, bias away from the detected edge
  if (edgeTriggeredLeft && !edgeTriggeredRight) {
    recoverLeftSpd  = -200;
    recoverRightSpd = -240;  // Swing left rear away from left edge
  } else if (edgeTriggeredRight && !edgeTriggeredLeft) {
    recoverLeftSpd  = -240;
    recoverRightSpd = -200;
  } else {
    // Both sides — straight back
    recoverLeftSpd  = -230;
    recoverRightSpd = -230;
  }
  setMotors(recoverLeftSpd, recoverRightSpd);
}

// ─────────────────────────────────────────────────────────────
//  LINE FOLLOWING — PID CONTROL
// ─────────────────────────────────────────────────────────────
void runLineFollow() {
  // readLineBlack: returns 0 (far left) to 5000 (far right)
  // 2500 = line centered under sensor array
  uint16_t pos   = qtr.readLineBlack(sensorValues);
  float    error = (float)pos - 2500.0f;

  // Accumulate integral with anti-windup
  pidIntegral += error;
  if      (pidIntegral >  integralMax) pidIntegral =  integralMax;
  else if (pidIntegral < -integralMax) pidIntegral = -integralMax;

  float derivative = error - pidLastError;
  float correction = KP * error + KI * pidIntegral + KD * derivative;
  pidLastError = error;

  int leftSpeed  = constrain(BASE_SPEED + (int)correction, -MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - (int)correction, -MAX_SPEED, MAX_SPEED);

  setMotors(leftSpeed, rightSpeed);
}

// ─────────────────────────────────────────────────────────────
//  SUMO MODE — STATE MACHINE
// ─────────────────────────────────────────────────────────────
void runSumo() {
  // Read all forward-facing sensors
  float distL = getUltrasonicCm(TRIG1, ECHO1);
  float distR = getUltrasonicCm(TRIG2, ECHO2);
  bool  irL   = (analogRead(IR_LEFT)  > IR_THRESHOLD);
  bool  irR   = (analogRead(IR_RIGHT) > IR_THRESHOLD);

  bool enemyL = (distL < CHARGE_DIST_CM) || irL;
  bool enemyR = (distR < CHARGE_DIST_CM) || irR;
  bool enemyDetected = enemyL || enemyR;

  // Transition to CHARGE if enemy found
  if (enemyDetected) {
    sumoState = S_CHARGE;
    sumoTimer = millis();
  }

  switch (sumoState) {

    case S_SEARCH:
      // Alternate spin direction every SEARCH_INTERVAL ms
      if (millis() - sumoTimer > (unsigned long)SEARCH_INTERVAL) {
        searchDir = -searchDir;
        sumoTimer = millis();
      }
      setMotors(SUMO_TURN_SPEED * searchDir, -SUMO_TURN_SPEED * searchDir);
      break;

    case S_CHARGE:
      if (!enemyDetected) {
        // Target lost — return to search
        sumoState = S_SEARCH;
        sumoTimer = millis();
        stopMotors();
        delay(80);
        break;
      }
      // Steer toward enemy while charging
      if (enemyL && enemyR) {
        // Enemy directly ahead — full charge
        setMotors(SUMO_FULL_SPEED, SUMO_FULL_SPEED);
      } else if (enemyL) {
        // Enemy on left — turn left while advancing
        setMotors(SUMO_FULL_SPEED / 2, SUMO_FULL_SPEED);
      } else {
        // Enemy on right — turn right while advancing
        setMotors(SUMO_FULL_SPEED, SUMO_FULL_SPEED / 2);
      }
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  BALLOON POP MODE — STATE MACHINE
// ─────────────────────────────────────────────────────────────
void runBalloonPop() {
  float distL = getUltrasonicCm(TRIG1, ECHO1);
  float distR = getUltrasonicCm(TRIG2, ECHO2);
  bool  irL   = (analogRead(IR_LEFT)  > IR_THRESHOLD);
  bool  irR   = (analogRead(IR_RIGHT) > IR_THRESHOLD);

  bool targetL   = (distL < BALLOON_DETECT_DIST) || irL;
  bool targetR   = (distR < BALLOON_DETECT_DIST) || irR;
  bool targetAny = targetL || targetR;

  // Very close = trigger ram (Sharp IR at close range)
  bool veryClose = (analogRead(IR_LEFT)  > IR_VERY_CLOSE ||
                    analogRead(IR_RIGHT) > IR_VERY_CLOSE ||
                    distL < 14 || distR < 14);

  switch (balloonState) {

    case B_SCAN:
      // Slow rotation scan to locate balloon
      if (millis() - balloonTimer > 2200) {
        balloonScanDir = -balloonScanDir;
        balloonTimer   = millis();
      }
      setMotors(BALLOON_SCAN_SPEED * balloonScanDir,
               -BALLOON_SCAN_SPEED * balloonScanDir);
      if (targetAny) {
        balloonState = B_APPROACH;
        balloonTimer = millis();
      }
      break;

    case B_APPROACH:
      if (veryClose) {
        balloonState = B_RAM;
        balloonTimer = millis();
        break;
      }
      if (!targetAny) {
        // Lost it — back to scan
        balloonState = B_SCAN;
        balloonTimer = millis();
        break;
      }
      // Steer toward balloon
      if (targetL && targetR) {
        setMotors(BALLOON_APPROACH_SPEED, BALLOON_APPROACH_SPEED);
      } else if (targetL) {
        setMotors(BALLOON_APPROACH_SPEED / 2, BALLOON_APPROACH_SPEED);
      } else {
        setMotors(BALLOON_APPROACH_SPEED, BALLOON_APPROACH_SPEED / 2);
      }
      break;

    case B_RAM:
      setMotors(BALLOON_RAM_SPEED, BALLOON_RAM_SPEED);
      if (millis() - balloonTimer >= (unsigned long)BALLOON_RAM_DURATION) {
        // Done — reverse slightly, then scan again
        stopMotors();
        delay(100);
        setMotors(-180, -180);
        delay(200);
        stopMotors();
        balloonState = B_SCAN;
        balloonTimer = millis();
      }
      break;
  }
}

// ─────────────────────────────────────────────────────────────
//  MOTOR CONTROL
// ─────────────────────────────────────────────────────────────
// speed range: -255 (full reverse) to +255 (full forward)
// IF ONE MOTOR RUNS BACKWARDS: swap its AIN1/AIN2 wires physically,
// or swap its AIN1/AIN2 pin definitions above.
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor (Channel A)
  if (leftSpeed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, (uint8_t)leftSpeed);

  // Right motor (Channel B)
  if (rightSpeed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, (uint8_t)rightSpeed);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
}

// ─────────────────────────────────────────────────────────────
//  SENSOR UTILITIES
// ─────────────────────────────────────────────────────────────
// Returns distance in cm, or 999.0 if no echo / out of range.
// Timeout = 23ms (~395cm) to prevent blocking on open air reads.
float getUltrasonicCm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 23000UL);
  if (duration == 0) return 999.0f;
  return duration * 0.01723f;
}

// ─────────────────────────────────────────────────────────────
//  DIAGNOSTIC PRINT (runs once on boot via Serial Monitor)
// ─────────────────────────────────────────────────────────────
void printSensorDiag() {
  Serial.println(F("\n--- Sensor Diagnostic ---"));

  // Ultrasonic
  float d1 = getUltrasonicCm(TRIG1, ECHO1);
  float d2 = getUltrasonicCm(TRIG2, ECHO2);
  Serial.print(F("Ultrasonic L: ")); Serial.print(d1); Serial.println(F(" cm"));
  Serial.print(F("Ultrasonic R: ")); Serial.print(d2); Serial.println(F(" cm"));

  // Sharp IR (raw ADC)
  int irL = analogRead(IR_LEFT);
  int irR = analogRead(IR_RIGHT);
  Serial.print(F("Sharp IR L (ADC): ")); Serial.println(irL);
  Serial.print(F("Sharp IR R (ADC): ")); Serial.println(irR);
  Serial.println(F("  (Hold hand ~30cm in front. Should read ~200-300)"));

  // QTR raw
  qtr.read(sensorValues);
  Serial.print(F("QTR raw (0=dark/1000=white): "));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(F(" "));
  }
  Serial.println();
  Serial.println(F("  (Place on ring: ~100–300. Place on white edge: ~700–1000)"));
  Serial.println(F("--- End Diagnostic ---\n"));
}

/*
 * ============================================================
 *  TUNING CHECKLIST (do this before the demo!)
 * ============================================================
 *
 *  1. MOTOR POLARITY
 *     Upload, open Serial, place bot on a surface.
 *     Temporarily call setMotors(150,150) in setup().
 *     Both motors should drive FORWARD. If one goes backward,
 *     swap that motor's AIN1/AIN2 definitions at the top.
 *
 *  2. QTR EDGE THRESHOLD
 *     Run printSensorDiag() with bot on the ring (black area)
 *     and note the values (~100–300).
 *     Then place on the white border and note values (~700–1000).
 *     Set EDGE_THRESHOLD to halfway between the two ranges.
 *
 *  3. SHARP IR THRESHOLD
 *     Hold your hand at ~30cm from each IR sensor.
 *     Read ADC from Serial Monitor.
 *     Set IR_THRESHOLD to ~80% of that reading.
 *     Set IR_VERY_CLOSE to value seen when hand is at ~12cm.
 *
 *  4. ULTRASONIC DISTANCES
 *     Verify with a ruler. Should match reasonably (~±2cm).
 *
 *  5. PID (line follow)
 *     Start KI = 0, KD = 0. Raise KP until bot oscillates.
 *     Then raise KD until oscillation damps.
 *     Only add KI if the bot consistently drifts off center.
 *
 *  6. SUMO SPEED & TIMING
 *     SUMO_FULL_SPEED = 255 is fine for the demo.
 *     Adjust SEARCH_INTERVAL if the bot overshoots the enemy too often.
 *
 * ============================================================
 */
