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

// Button Debouncing
bool lastButtonReading = HIGH;
bool buttonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Strict 5ms Timing Loop
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 5; 

// ==========================================
// TUNING VARIABLES (High Speed / Hard Brakes)
// ==========================================
int leftMotorOffset = 0;  
int rightMotorOffset = 0; 

// Boosted sprint speed for straightaways
int baseSpeed = 200; 
int maxSpeed = 255;  

// Kp lowered to prevent high-speed wobbles.
// Kd raised to aggressively absorb the shaking.
float Kp = 0.04;   
float Ki = 0.0;    
float Kd = 3.0;    

int lastError = 0;

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);

  pinMode(pwmaPin, OUTPUT); pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT); pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);
  
  pinMode(emitterCtrl, OUTPUT); digitalWrite(emitterCtrl, HIGH);
  pinMode(buttonPin, INPUT_PULLUP); 

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
      runAutoWiggleCalibration();
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

      if (buttonState == LOW) { 
        if (currentState == STANDBY_UNCALIBRATED) currentState = CALIBRATING;
        else if (currentState == STANDBY_READY) {
          lastError = 0; delay(400); currentState = PLAYING;
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
// AUTOMATIC CALIBRATION (LOW DRIFT)
// ==========================================
void runAutoWiggleCalibration() {
  delay(1000); 
  
  // Dropped speed to 70 and shortened loops to prevent the robot 
  // from twisting out of its original starting angle.
  setMotors(-70, 70);
  for (int i = 0; i < 30; i++) { qtr.calibrate(); delay(5); }
  stopMotors(); delay(150); 
  
  setMotors(70, -70);
  for (int i = 0; i < 60; i++) { qtr.calibrate(); delay(5); }
  stopMotors(); delay(150); 
  
  setMotors(-70, 70);
  for (int i = 0; i < 30; i++) { qtr.calibrate(); delay(5); }
  stopMotors();
}

// ==========================================
// PURE PID LOGIC
// ==========================================
void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // --- HYPER-SENSITIVE DYNAMIC BRAKING ---
  // We need high speed on straights, but instant braking for the zig-zag.
  int currentBaseSpeed = baseSpeed;
  
  if (abs(error) > 800) {
    // Sharp Zig-Zag or U-Turn detected: SLAM the brakes
    currentBaseSpeed = 60;  
  } else if (abs(error) > 200) {
    // Robot is drifting slightly (entering a curve): Mild brake
    currentBaseSpeed = 120; 
  }
  +

  // --- CORE PID MATH ---
  int P = error;
  int D = error - lastError;
  int motorSpeedAdjustment = (int)((Kp * P) + (Ki * I) + (Kd * D););
  lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // --- SHARP PIVOT ALLOWANCE ---
  // We allow the inner wheel to reverse up to -100. 
  // Combined with the 60 base speed above, this allows the robot to aggressively
  // yank its nose around the zig-zag without flying off the track.
  leftMotorSpeed = constrain(leftMotorSpeed, -100, maxSpeed); 
  rightMotorSpeed = constrain(rightMotorSpeed, -100, maxSpeed);

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