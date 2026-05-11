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
const int ledPin = 13; // Built-in Arduino LED

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

float Kp = 0.08;   
float Ki = 0.0;    // Disabled to prevent windup on U-turns
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
  
  // NOTE: If your button connects to Ground instead of 5V, change this to INPUT_PULLUP
  // and reverse the logic in handleButtonPress.
  pinMode(buttonPin, INPUT); 
  pinMode(ledPin, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);
  
  stopMotors();
  delay(1000); 
  Serial.println("Line Follower Ready. Press button to calibrate.");
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
      runManualCalibration();
      currentState = STANDBY_READY;
      break;
    case STANDBY_READY: 
      break;
    case PLAYING:
      // STRICT TIMING LOOP: Only run PID calculation every 5ms
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
      
      if (buttonState == HIGH) { 
        if (currentState == STANDBY_UNCALIBRATED) {
          currentState = CALIBRATING;
        } 
        else if (currentState == STANDBY_READY) {
          lastError = 0; 
          delay(500); // Small pause before launching
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
// MANUAL CALIBRATION 
// ==========================================
void runManualCalibration() {
  Serial.println("Calibrating...");
  digitalWrite(ledPin, HIGH); // Turn on LED to signal calibration start
  
  // You have ~5 seconds to smoothly slide the front sensors over the line left and right
  for (int i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(20);
  }
  
  digitalWrite(ledPin, LOW); // Turn off LED when done
  Serial.println("Calibration Complete. Press button to start.");
}

// ==========================================
// OPTIMIZED 4-SENSOR PID LOGIC
// ==========================================
void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // 1. Core PID Math
  int P = error;
  int D = error - lastError;
  int motorSpeedAdjustment = (int)((Kp * P) + (Kd * D));
  lastError = error;

  // 2. Pre-emptive Braking
  // Drop the base speed if the line reaches the outer sensors
  int currentBaseSpeed = baseSpeed;
  if (abs(error) > 1000) {
    currentBaseSpeed = baseSpeed - 30; 
  }

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // 3. Aggressive Arc Recovery & Constrains
  if (error <= -1500) {
    // Lost line entirely to the left
    leftMotorSpeed = 0; // Lock inner wheel
    rightMotorSpeed = currentBaseSpeed + 60; // Surge outer wheel forward
  } 
  else if (error >= 1500) {
    // Lost line entirely to the right
    leftMotorSpeed = currentBaseSpeed + 60; // Surge outer wheel forward
    rightMotorSpeed = 0; // Lock inner wheel
  } 
  else {
    // Normal line following - Strict minimum of 0 to prevent tank-spinning
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