#include <QTRSensors.h>
#include <EEPROM.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================

// LEFT TO RIGHT: Far-Left (A2), Mid-Left (A1), Center-Left (A0)
// Center-Right (A5), Mid-Right (A4), Far-Right (A3)
const uint8_t SensorCount = 6;
const uint8_t sensorPins[SensorCount] = {A2, A1, A0, A5, A4, A3};

const int leftEmitterCtrl = 13;
const int rightEmitterCtrl = 12;

// Motor Driver (TB6612FNG)
const int pwmaPin = 9;
const int ain1Pin = 4;
const int ain2Pin = 5;
const int stbyPin = 6;
const int pwmbPin = 10;
const int bin1Pin = 7;
const int bin2Pin = 8;

const int buttonPin = 2;

// ==========================================
// EEPROM LAYOUT
// ==========================================
// Addresses  0–11: calibration minimums (6 x uint16_t = 12 bytes)
// Addresses 12–23: calibration maximums (6 x uint16_t = 12 bytes)
// Address     24:  magic byte (0xAB) — marks a valid saved calibration

const int    EEPROM_MIN_ADDR   = 0;
const int    EEPROM_MAX_ADDR   = 12;
const int    EEPROM_MAGIC_ADDR = 24;
const uint8_t EEPROM_MAGIC     = 0xAB;

// ==========================================
// VARIABLES & CONSTANTS
// ==========================================

QTRSensors qtr;

enum RobotState {
  STANDBY_UNCALIBRATED,
  CALIBRATING,
  STANDBY_READY,
  PLAYING
};
RobotState currentState = STANDBY_UNCALIBRATED;

// Button Debouncing
bool lastButtonReading = HIGH;
bool buttonState       = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ==========================================
// TUNING VARIABLES
// ==========================================

int leftMotorOffset  = 0;
int rightMotorOffset = 0;

float Kp = 0.09;
float Ki = 0.001;  // Raised from 0.0002 — old value gave max I contribution of ~9
float Kd = 0.3;

int lastError = 0;
long integral = 0;
const long integralMax = 45000;

int baseSpeed = 255;
int maxSpeed  = 255;

uint16_t sensorValues[SensorCount];

// ==========================================
// FORWARD DECLARATIONS
// ==========================================

void handleButtonPress();
void runAutoWiggleCalibration();
void followLinePID();
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void saveCalibrationToEEPROM();
bool loadCalibrationFromEEPROM();

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(115200);  // Faster than 9600 — better for live tuning

  pinMode(pwmaPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  pinMode(leftEmitterCtrl, OUTPUT);
  pinMode(rightEmitterCtrl, OUTPUT);
  digitalWrite(leftEmitterCtrl, HIGH);
  digitalWrite(rightEmitterCtrl, HIGH);

  pinMode(buttonPin, INPUT_PULLUP);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  stopMotors();
  delay(500);

  // Attempt to restore saved calibration from EEPROM
  if (loadCalibrationFromEEPROM()) {
    currentState = STANDBY_READY;
    Serial.println("Saved calibration loaded from EEPROM. Skip recalibration.");
    Serial.println("Press button to START.");
  } else {
    currentState = STANDBY_UNCALIBRATED;
    Serial.println("No saved calibration found.");
    Serial.println("Place on track and press button to Calibrate.");
  }
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
  handleButtonPress();

  switch (currentState) {
    case STANDBY_UNCALIBRATED: break;
    case CALIBRATING:
      runAutoWiggleCalibration();
      currentState = STANDBY_READY;
      break;
    case STANDBY_READY: break;
    case PLAYING:
      followLinePID();
      break;
  }
}

// ==========================================
// STATE MACHINE & BUTTON LOGIC
// ==========================================

void handleButtonPress() {
  bool reading = digitalRead(buttonPin);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {
        if (currentState == STANDBY_UNCALIBRATED) {
          currentState = CALIBRATING;
        }
        else if (currentState == STANDBY_READY) {
          Serial.println("State: PLAYING. Escaping Start Box!");

          lastError = 0;
          integral  = 0;

          setMotors(150, 150);
          delay(400);

          currentState = PLAYING;
        }
        else if (currentState == PLAYING) {
          stopMotors();
          currentState = STANDBY_READY;
          Serial.println("State: STANDBY. Motors Stopped.");
        }
      }
    }
  }
  lastButtonReading = reading;
}

// ==========================================
// AUTOMATIC CALIBRATION
// ==========================================

void runAutoWiggleCalibration() {
  Serial.println("Calibrating...");
  delay(1000);

  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }

  stopMotors(); delay(250);

  setMotors(90, -90);
  for (int i = 0; i < 80; i++) { qtr.calibrate(); delay(5); }

  stopMotors(); delay(250);

  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }

  stopMotors();

  // Persist calibration so next boot skips this step
  saveCalibrationToEEPROM();

  Serial.println("Calibration Done & Saved to EEPROM.");
  Serial.println("Place on Grey Start Box and press button to run.");
}

// ==========================================
// PID LINE FOLLOWING
// ==========================================

void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  // ---------------------------------------------------------
  // LINE-LOST DETECTION
  // If no sensor sees the line, spin slowly toward the last
  // known direction instead of flying off the track.
  // ---------------------------------------------------------
  bool lineDetected = false;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 200) {
      lineDetected = true;
      break;
    }
  }

  if (!lineDetected) {
    int searchSpeed = 130;
    if (lastError > 0) {
      setMotors(searchSpeed, -searchSpeed);  // Spin right — line was to the right
    } else {
      setMotors(-searchSpeed, searchSpeed);  // Spin left  — line was to the left
    }
    return;
  }

  int error   = (int)position - 2500;
  int absError = abs(error);

  // ---------------------------------------------------------
  // 90-DEGREE SNAP OVERRIDE
  // Added 1500ms spin timeout — prevents infinite spin if
  // the sensor loses the line mid-turn.
  // ---------------------------------------------------------
  if (error > 2000) {
    setMotors(150, 150);
    delay(70);

    unsigned long spinStart = millis();
    while (millis() - spinStart < 1500) {
      setMotors(220, -220);
      qtr.readCalibrated(sensorValues);
      if (sensorValues[2] > 400 || sensorValues[3] > 400) break;
    }

    lastError = 0;
    integral  = 0;
    return;
  }
  else if (error < -2000) {
    setMotors(150, 150);
    delay(70);

    unsigned long spinStart = millis();
    while (millis() - spinStart < 1500) {
      setMotors(-220, 220);
      qtr.readCalibrated(sensorValues);
      if (sensorValues[2] > 400 || sensorValues[3] > 400) break;
    }

    lastError = 0;
    integral  = 0;
    return;
  }

  // ---------------------------------------------------------
  // SMOOTH PROGRESSIVE SPEED SCALING
  // Replaced hard if/else tiers with map() for linear ramp —
  // eliminates the jarring speed jump between tiers.
  // ---------------------------------------------------------
  int currentBaseSpeed;

  if (absError > 1500) {
    // Extreme corner — hard brake
    currentBaseSpeed = map(absError, 1500, 2500, 160, 100);
    currentBaseSpeed = constrain(currentBaseSpeed, 100, 160);
  }
  else if (absError > 500) {
    // Sweeping curve — medium brake
    currentBaseSpeed = map(absError, 500, 1500, baseSpeed, 160);
    currentBaseSpeed = constrain(currentBaseSpeed, 160, baseSpeed);
  }
  else {
    // Straight — full speed
    currentBaseSpeed = baseSpeed;
  }

  // ---------------------------------------------------------
  // INTEGRAL (anti-windup + sign-change reset)
  // ---------------------------------------------------------
  if (absError < 800) {
    integral += error;
    integral = constrain(integral, -integralMax, integralMax);
  }

  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) {
    integral = 0;
  }

  // ---------------------------------------------------------
  // CORE PID MATH
  // ---------------------------------------------------------
  // Suppress derivative spike on the very first reading
  int D = (lastError == 0) ? 0 : (error - lastError);

  int motorSpeedAdjustment = (int)((Kp * error) + (Ki * integral) + (Kd * D));
  lastError = error;

  int leftMotorSpeed  = currentBaseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // Final constrain is inside setMotors() — no need to do it here
  setMotors(leftMotorSpeed, rightMotorSpeed);
}

// ==========================================
// MOTOR HELPER FUNCTIONS
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(stbyPin, HIGH);

  // Single constrain point — removed redundant ones from followLinePID()
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH);
    analogWrite(pwmaPin, leftSpeed);
  } else {
    digitalWrite(ain1Pin, HIGH);
    digitalWrite(ain2Pin, LOW);
    analogWrite(pwmaPin, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    digitalWrite(bin1Pin, LOW);
    digitalWrite(bin2Pin, HIGH);
    analogWrite(pwmbPin, rightSpeed);
  } else {
    digitalWrite(bin1Pin, HIGH);
    digitalWrite(bin2Pin, LOW);
    analogWrite(pwmbPin, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(stbyPin, LOW);
  analogWrite(pwmaPin, 0);
  analogWrite(pwmbPin, 0);
}

// ==========================================
// EEPROM CALIBRATION SAVE / LOAD
// ==========================================

void saveCalibrationToEEPROM() {
  for (int i = 0; i < SensorCount; i++) {
    EEPROM.put(EEPROM_MIN_ADDR + i * 2, qtr.calibrationOn.minimum[i]);
    EEPROM.put(EEPROM_MAX_ADDR + i * 2, qtr.calibrationOn.maximum[i]);
  }
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  Serial.println("Calibration saved to EEPROM.");
}

bool loadCalibrationFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC) {
    return false;  // No valid calibration stored
  }
  // Run one dummy calibrate() to allocate the library's internal arrays
  // before we overwrite them with our saved values
  qtr.calibrate();
  for (int i = 0; i < SensorCount; i++) {
    EEPROM.get(EEPROM_MIN_ADDR + i * 2, qtr.calibrationOn.minimum[i]);
    EEPROM.get(EEPROM_MAX_ADDR + i * 2, qtr.calibrationOn.maximum[i]);
  }
  Serial.println("Calibration loaded from EEPROM.");
  return true;
}
