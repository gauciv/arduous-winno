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

// ==========================================
// TUNING VARIABLES (HEAVILY DAMPENED)
// ==========================================
int leftMotorOffset = 0;  
int rightMotorOffset = 0; 

// 1. Slowed down base speed to guarantee it can read the line
int baseSpeed = 100; 
int maxSpeed = 160;  

// 2. Halved the PID values to stop the shaking
float Kp = 0.05;   
float Ki = 0.0001; 
float Kd = 1.0;    

int lastError = 0;
long integral = 0;              
const long integralMax = 10000; 

void setup() {
  Serial.begin(9600);
  pinMode(pwmaPin, OUTPUT); pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT); pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT); pinMode(emitterCtrl, OUTPUT); digitalWrite(emitterCtrl, HIGH);
  pinMode(buttonPin, INPUT); 

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  stopMotors();
  delay(1000); 
}

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

void handleButtonPress() {
  bool reading = digitalRead(buttonPin);
  if (reading != lastButtonReading) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) { 
        if (currentState == STANDBY_UNCALIBRATED) currentState = CALIBRATING;
        else if (currentState == STANDBY_READY) {
          lastError = 0; integral = 0; delay(400); currentState = PLAYING;
        } 
        else if (currentState == PLAYING) {
          stopMotors(); currentState = STANDBY_READY;
        }
      }
    }
  }
  lastButtonReading = reading;
}

void runAutoWiggleCalibration() {
  delay(1000); 
  setMotors(-80, 80);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }
  stopMotors(); delay(250); 
  setMotors(80, -80);
  for (int i = 0; i < 80; i++) { qtr.calibrate(); delay(5); }
  stopMotors(); delay(250); 
  setMotors(-80, 80);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }
  stopMotors();
}

void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  integral += error;
  integral = constrain(integral, -integralMax, integralMax);

  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) integral = 0;

  int P = error;
  int I = integral;
  int D = error - lastError;

  int motorSpeedAdjustment = (int)((Kp * P) + (Ki * I) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = baseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = baseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // 3. Prevent inner wheel from reversing aggressively to stop U-turn spin-outs
  // Replaced -100 with -20 so it maintains forward momentum through wide turns
  leftMotorSpeed = constrain(leftMotorSpeed, -20, maxSpeed); 
  rightMotorSpeed = constrain(rightMotorSpeed, -20, maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

void setMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(stbyPin, HIGH);
  leftSpeed = constrain(leftSpeed, -255, 255); rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) { digitalWrite(bin1Pin, LOW); digitalWrite(bin2Pin, HIGH); analogWrite(pwmbPin, leftSpeed); } 
  else { digitalWrite(bin1Pin, HIGH); digitalWrite(bin2Pin, LOW); analogWrite(pwmbPin, -leftSpeed); }

  if (rightSpeed >= 0) { digitalWrite(ain1Pin, LOW); digitalWrite(ain2Pin, HIGH); analogWrite(pwmaPin, rightSpeed); } 
  else { digitalWrite(ain1Pin, HIGH); digitalWrite(ain2Pin, LOW); analogWrite(pwmaPin, -rightSpeed); }
}

void stopMotors() { digitalWrite(stbyPin, LOW); analogWrite(pwmaPin, 0); analogWrite(pwmbPin, 0); }