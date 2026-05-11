#include <QTRSensors.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
const uint8_t SensorCount = 4;
const uint8_t sensorPins[SensorCount] = {A1, A0, A5, A4};
const int emitterCtrl = 12;

// Motor Driver (TB6612FNG) 
const int pwmaPin = 9;  
const int ain1Pin = 4;  
const int ain2Pin = 5;  

const int pwmbPin = 10; 
const int bin1Pin = 7;  
const int bin2Pin = 8;  

const int stbyPin = 6;
const int buttonPin = 11; 
const int ledPin = 13; 

// ==========================================
// VARIABLES & CONSTANTS
// ==========================================
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

enum RobotState {
  STANDBY_UNCALIBRATED,
  CALIBRATING,
  STANDBY_READY,
  PLAYING
};
RobotState currentState = STANDBY_UNCALIBRATED;

bool lastButtonReading = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Timing variables for strict PID loop
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 5; // 5ms = 200Hz loop

// ==========================================
// TUNING VARIABLES
// ==========================================
int leftMotorOffset = 0;  
int rightMotorOffset = 0; 

int baseSpeed = 110; 
int maxSpeed = 160;  

// Boosted Kp slightly so it grips the line tighter on the sweep
float Kp = 0.08;   
float Ki = 0.0;    // Keep this 0.0 to prevent U-turn windup
float Kd = 1.2;    

int lastError = 0;

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);
  pinMode(pwmaPin, OUTPUT); pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT); pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT); pinMode(emitterCtrl, OUTPUT); digitalWrite(emitterCtrl, HIGH);
  pinMode(buttonPin, INPUT); pinMode(ledPin, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  stopMotors();
  delay(1000); 
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  handleButtonPress();

  switch (currentState) {
    case STANDBY_UNCALIBRATED: break;
    case CALIBRATING:
      runManualCalibration();
      currentState = STANDBY_READY;
      break;
    case STANDBY_READY: break;
    case PLAYING:
      if (millis() - lastLoopTime >= loopInterval) {
        lastLoopTime = millis();
        followLinePID();
      }
      break;
  }
}

// ==========================================
// BUTTON LOGIC
// ==========================================
void handleButtonPress() {
  bool reading = digitalRead(buttonPin);
  if (reading != lastButtonReading) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) { 
        if (currentState == STANDBY_UNCALIBRATED) currentState = CALIBRATING;
        else if (currentState == STANDBY_READY) {
          lastError = 0; delay(500); currentState = PLAYING;
        } 
        else if (currentState == PLAYING) {
          stopMotors(); currentState = STANDBY_READY;
        }
      }
    }
  }
  lastButtonReading = reading;
}

// ==========================================
// MANUAL CALIBRATION 
// ==========================================
void runManualCalibration() {
  digitalWrite(ledPin, HIGH); 
  
  // Slide the front of the robot over the line left and right
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  
  digitalWrite(ledPin, LOW); 
}

// ==========================================
// PURE PID LOGIC (NO REVERSING)
// ==========================================
void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // PID Calculation (No Integral needed right now)
  int P = error;
  int D = error - lastError;

  int motorSpeedAdjustment = (int)((Kp * P) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = baseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = baseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // ==========================================
  // THE U-TURN FIX: NO REVERSING ALLOWED
  // ==========================================
  // If the robot loses the line (error is maxed at +/- 1500), 
  // we push the outer wheel forward and stop the inner wheel entirely (0).
  // By preventing the inner wheel from going negative, it CANNOT do a 180-spin.
  
  if (error <= -1500) {
    leftMotorSpeed = 0; // Lock left wheel
    rightMotorSpeed = baseSpeed + 40; // Push right wheel forward through the arc
  } else if (error >= 1500) {
    leftMotorSpeed = baseSpeed + 40; // Push left wheel forward through the arc
    rightMotorSpeed = 0; // Lock right wheel
  } else {
    // Normal turning constraints. Minimum is strictly 0.
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed); 
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
  }

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

// ==========================================
// MOTOR HELPER FUNCTIONS
// ==========================================
void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(stbyPin, HIGH);
  leftSpeed = constrain(leftSpeed, -255, 255); rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) { digitalWrite(bin1Pin, LOW); digitalWrite(bin2Pin, HIGH); analogWrite(pwmbPin, leftSpeed); } 
  else { digitalWrite(bin1Pin, HIGH); digitalWrite(bin2Pin, LOW); analogWrite(pwmbPin, -leftSpeed); }

  if (rightSpeed >= 0) { digitalWrite(ain1Pin, LOW); digitalWrite(ain2Pin, HIGH); analogWrite(pwmaPin, rightSpeed); } 
  else { digitalWrite(ain1Pin, HIGH); digitalWrite(ain2Pin, LOW); analogWrite(pwmaPin, -rightSpeed); }
}

void stopMotors() { digitalWrite(stbyPin, LOW); analogWrite(pwmaPin, 0); analogWrite(pwmbPin, 0); }