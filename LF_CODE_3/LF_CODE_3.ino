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
const int ledPin = 13; // Built-in LED for calibration status

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

// PID Constants
float Kp = 0.06;   
float Ki = 0.0001; 
float Kd = 1.2;    

int lastError = 0;
long integral = 0;              
const long integralMax = 10000; 

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
      // STRICT TIMING LOOP: Only run PID every 5ms
      if (millis() - lastLoopTime >= loopInterval) {
        lastLoopTime = millis();
        followLinePID();
      }
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
          lastError = 0; integral = 0; delay(500); currentState = PLAYING;
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
// MANUAL CALIBRATION (RELIABLE)
// ==========================================
void runManualCalibration() {
  digitalWrite(ledPin, HIGH); // Turn on LED to signal calibration start
  
  // Calibrate for ~5 seconds. 
  // YOU MUST MANUALLY SLIDE THE ROBOT OVER THE LINE LEFT AND RIGHT
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  
  digitalWrite(ledPin, LOW); // Turn off LED when done
}

// ==========================================
// PURE PID LOGIC
// ==========================================
void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // Anti-windup
  integral += error;
  integral = constrain(integral, -integralMax, integralMax);
  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) integral = 0;

  // PID Calculation
  int P = error;
  int I = integral;
  int D = error - lastError;

  int motorSpeedAdjustment = (int)((Kp * P) + (Ki * I) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = baseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = baseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // ==========================================
  // EXTREME RECOVERY OVERRIDE
  // ==========================================
  // If error is perfectly 1500 or -1500, the robot has completely lost the line.
  // The QTR library "remembers" the last known direction. 
  // We temporarily allow a deeper reverse speed (-80) to aggressively pull it back.
  if (error <= -1500) {
    leftMotorSpeed = -80; // Hard reverse inner wheel
    rightMotorSpeed = baseSpeed + 50; 
  } else if (error >= 1500) {
    leftMotorSpeed = baseSpeed + 50;
    rightMotorSpeed = -80; // Hard reverse inner wheel
  } else {
    // Normal turning constraints
    leftMotorSpeed = constrain(leftMotorSpeed, -20, maxSpeed); 
    rightMotorSpeed = constrain(rightMotorSpeed, -20, maxSpeed);
  }

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