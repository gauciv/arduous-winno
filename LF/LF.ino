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

// Button Debouncing (Working Pull-up Logic)
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Timing variables for strict PID loop (FIXES SHAKING)
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 5; // 5ms = 200Hz loop

// ==========================================
// TUNING VARIABLES 
// ==========================================
int leftMotorOffset = 0;  
int rightMotorOffset = 0; 

// Slightly softened PID to work with the new Arc-Turning constraints
float Kp = 0.08;   
float Ki = 0.0;    // Disabled to prevent memory windup on U-turns
float Kd = 1.5;    

int lastError = 0;

int baseSpeed = 150; 
int maxSpeed = 220;  

uint16_t sensorValues[SensorCount];

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);

  pinMode(pwmaPin, OUTPUT); pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT); pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);
  
  pinMode(emitterCtrl, OUTPUT); digitalWrite(emitterCtrl, HIGH);

  // Teammate's working fix for the button
  pinMode(buttonPin, INPUT_PULLUP); 

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  stopMotors();

  delay(1000); 
  Serial.println("Line Follower Powered On & Ready.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  handleButtonPress();

  switch (currentState) {
    case STANDBY_UNCALIBRATED: 
      break;
    case CALIBRATING:
      runAutoWiggleCalibration();
      currentState = STANDBY_READY;
      break;
    case STANDBY_READY: 
      break;
    case PLAYING:
      // STRICT TIMING: Only run PID calculation every 5ms
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
          lastError = 0;
          delay(400); 
          currentState = PLAYING;
        } 
        else if (currentState == PLAYING) {
          stopMotors();
          currentState = STANDBY_READY;
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
}

// ==========================================
// PURE PID LOGIC (OPTIMIZED FOR 4 SENSORS)
// ==========================================
void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // 1. Teammate's Progressive Braking
  int currentBaseSpeed = baseSpeed; 
  if (abs(error) > 500) currentBaseSpeed = 80; 
  else if (abs(error) > 200) currentBaseSpeed = 120; 

  // 2. Core PID Math
  int P = error;
  int D = error - lastError;
  int motorSpeedAdjustment = (int)((Kp * P) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // 3. THE U-TURN FIX (ARC TURNING & EXTREME RECOVERY)
  if (error <= -1500) {
    // Lost line left: Lock inner wheel, surge outer wheel
    leftMotorSpeed = 0; 
    rightMotorSpeed = currentBaseSpeed + 50; 
  } 
  else if (error >= 1500) {
    // Lost line right: Lock inner wheel, surge outer wheel
    leftMotorSpeed = currentBaseSpeed + 50; 
    rightMotorSpeed = 0; 
  } 
  else {
    // Normal turning: Minimum is strictly 0 to force an arc instead of a spin
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

void stopMotors() {
  digitalWrite(stbyPin, LOW); analogWrite(pwmaPin, 0); analogWrite(pwmbPin, 0);
}