#include <QTRSensors.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================

// LEFT TO RIGHT: Mid-Left (A1), Center-Left (A0), Center-Right (A5), Mid-Right (A4)
const uint8_t SensorCount = 4;
const uint8_t sensorPins[SensorCount] = {A1, A0, A5, A4};
const int emitterCtrl = 12;

// Motor Driver (TB6612FNG) 
const int pwmaPin = 9;  // Right Motor Speed
const int ain1Pin = 4;  // Right Motor Dir 1
const int ain2Pin = 5;  // Right Motor Dir 2

const int pwmbPin = 10; // Left Motor Speed
const int bin1Pin = 7;  // Left Motor Dir 1
const int bin2Pin = 8;  // Left Motor Dir 2

const int stbyPin = 6;
const int buttonPin = 11; 

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

// Button Debouncing 
bool lastButtonReading = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ==========================================
// TUNING VARIABLES
// ==========================================

// Motor offsets in case one motor is naturally faster
int leftMotorOffset = 0;  
int rightMotorOffset = 0; 

// Base speed and max speed
int baseSpeed = 150; 
int maxSpeed = 220;  

// PID Constants (Adjusted for smooth turning)
// Mikae suggested Kp=0.6, but with a 1500 error range, that creates corrections of 900.
// We are starting with a more standard range to prevent violent shaking.
float Kp = 0.12;   
float Ki = 0.0001; 
float Kd = 2.5;    

int lastError = 0;
long integral = 0;              
const long integralMax = 20000; // Lowered to prevent massive wind-up on curves

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(9600);

  pinMode(pwmaPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  pinMode(emitterCtrl, OUTPUT);
  digitalWrite(emitterCtrl, HIGH);
  pinMode(buttonPin, INPUT); 

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
    case STANDBY_UNCALIBRATED: 
      break;
    case CALIBRATING:
      runAutoWiggleCalibration();
      currentState = STANDBY_READY;
      break;
    case STANDBY_READY: 
      break;
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

      if (buttonState == HIGH) { 
        if (currentState == STANDBY_UNCALIBRATED) {
          currentState = CALIBRATING;
        } 
        else if (currentState == STANDBY_READY) {
          lastError = 0;
          integral = 0;
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
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(5);
  }
  
  stopMotors(); delay(250); 
  
  setMotors(90, -90);
  for (int i = 0; i < 80; i++) {
    qtr.calibrate();
    delay(5);
  }
  
  stopMotors(); delay(250); 
  
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(5);
  }

  stopMotors();
}

// ==========================================
// PURE PID LOGIC (CLEANED)
// ==========================================

void followLinePID() {
  // Read line position. 0 to 3000 (since there are 4 sensors). Center is 1500.
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // Anti-windup for the integral term
  integral += error;
  integral = constrain(integral, -integralMax, integralMax);

  // Reset integral if we cross the center line to prevent overshooting
  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) {
    integral = 0;
  }

  // Core PID Math
  int P = error;
  int I = integral;
  int D = error - lastError;

  int motorSpeedAdjustment = (int)((Kp * P) + (Ki * I) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = baseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = baseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // IMPORTANT: The minimum is set to -100 instead of 0. 
  // This allows the inner wheel to reverse on sharp zig-zags, pulling the robot through the turn tightly.
  leftMotorSpeed = constrain(leftMotorSpeed, -100, maxSpeed); 
  rightMotorSpeed = constrain(rightMotorSpeed, -100, maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

// ==========================================
// MOTOR HELPER FUNCTIONS 
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(stbyPin, HIGH);

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // LEFT MOTOR
  if (leftSpeed >= 0) {
    digitalWrite(bin1Pin, LOW);  
    digitalWrite(bin2Pin, HIGH); 
    analogWrite(pwmbPin, leftSpeed);
  } else {
    digitalWrite(bin1Pin, HIGH); 
    digitalWrite(bin2Pin, LOW);  
    analogWrite(pwmbPin, -leftSpeed);
  }

  // RIGHT MOTOR
  if (rightSpeed >= 0) {
    digitalWrite(ain1Pin, LOW);  
    digitalWrite(ain2Pin, HIGH); 
    analogWrite(pwmaPin, rightSpeed);
  } else {
    digitalWrite(ain1Pin, HIGH); 
    digitalWrite(ain2Pin, LOW);  
    analogWrite(pwmaPin, -rightSpeed);
  }
}

void stopMotors() {
  digitalWrite(stbyPin, LOW);
  analogWrite(pwmaPin, 0);
  analogWrite(pwmbPin, 0);
}