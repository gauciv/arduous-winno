#include <QTRSensors.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================

// LEFT TO RIGHT: Mid-Left (A1), Center-Left (A0), Center-Right (A5), Mid-Right (A4)
const uint8_t SensorCount = 4;
const uint8_t sensorPins[SensorCount] = {A1, A0, A5, A4};

const int EmitterCtrl = 12;

// Motor Driver (TB6612FNG) 
const int pwmaPin = 9;  
const int ain1Pin = 4;  
const int ain2Pin = 5;  
const int stbyPin = 6;
const int pwmbPin = 10; 
const int bin1Pin = 7;  
const int bin2Pin = 8;  

const int buttonPin = 13; 

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
bool lastButtonReading = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ==========================================
// TUNING VARIABLES (LiPo Compensated)
// ==========================================

int leftMotorOffset = 0;  
int rightMotorOffset = 0; 

float Kp = 0.09;   
float Ki = 0.0001; 
float Kd = 0.3;    

int lastError = 0;
long integral = 0;              
const long integralMax = 45000; 

int baseSpeed = 255; 
int maxSpeed = 255;  

uint16_t sensorValues[SensorCount];

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

  // pinMode(EmitterCtrl, OUTPUT);
  pinMode(EmitterCtrl, OUTPUT);
  // digitalWrite(EmitterCtrl, HIGH);
  digitalWrite(EmitterCtrl, HIGH);

  pinMode(buttonPin, INPUT); 

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

      if (buttonState == HIGH) { 
        if (currentState == STANDBY_UNCALIBRATED) {
          currentState = CALIBRATING;
        } 
        else if (currentState == STANDBY_READY) {
          lastError = 0;
          integral = 0;
          setMotors(150, 150); 
          delay(400); 
          currentState = PLAYING;
        } 
        else if (currentState == PLAYING) {
          brakeMotors(); // SAFETY FIX: Active brake instead of coasting when stopping
          delay(100);
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
  
  // SAFETY FIX: Brake to safely kill momentum before reversing
  brakeMotors(); delay(150); 
  
  setMotors(90, -90);
  for (int i = 0; i < 80; i++) {
    qtr.calibrate();
    delay(5);
  }
  
  brakeMotors(); delay(150); 
  
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(5);
  }

  brakeMotors(); delay(100);
  stopMotors();
}

// ==========================================
// PURE PID LOGIC
// ==========================================

void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // ---------------------------------------------------------
  // 90-DEGREE SNAP OVERRIDE
  // ---------------------------------------------------------
  if (error > 1000) {
    setMotors(200, 200); 
    delay(60); 

    // SAFETY FIX: Active brake to kill forward momentum before snap reversing
    brakeMotors();
    delay(50); 

    while (true) {
      // SAFETY FIX: Lowered extreme reverse speed to prevent current spiking 
      setMotors(160, -160); 
      qtr.readCalibrated(sensorValues); 
      
      if (sensorValues[1] > 650 || sensorValues[2] > 650) {
        break;
      }
    }
    brakeMotors(); // SAFETY FIX: Brake after snap turn
    delay(20);

    lastError = 0; 
    integral = 0;  
    return; 
  } 
  else if (error < -1000) {
    setMotors(200, 200); 
    delay(60); 
    
    // SAFETY FIX: Active brake to kill forward momentum
    brakeMotors();
    delay(50);

    while (true) {
      setMotors(-160, 160); 
      qtr.readCalibrated(sensorValues);
      
      if (sensorValues[1] > 650 || sensorValues[2] > 650) {
        break;
      }
    }
    brakeMotors();
    delay(20);

    lastError = 0; 
    integral = 0;  
    return; 
  }

  // ---------------------------------------------------------
  // MULTI-STAGE PROGRESSIVE BRAKING 
  // ---------------------------------------------------------
  int currentBaseSpeed = baseSpeed; 

  if (abs(error) > 750) {
    currentBaseSpeed = 90; 
  } 
  else if (abs(error) > 250) {
    currentBaseSpeed = 150; 
  }

  // ---------------------------------------------------------
  // U-TURN ANTI-WINDUP
  // ---------------------------------------------------------
  if (abs(error) < 500) {
    integral += error;
    integral = constrain(integral, -integralMax, integralMax);
  }

  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) {
    integral = 0;
  }

  // ---------------------------------------------------------
  // CORE PID MATH
  // ---------------------------------------------------------
  int P = error;
  int I = integral;
  int D = error - lastError;

  if (lastError == 0 && D != 0) {
    D = 0;
  }

  int motorSpeedAdjustment = (int)((Kp * P) + (Ki * I) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + rightMotorOffset;

  // SAFETY FIX: Limited PID reverse speed from -150 to -100 to reduce rapid stutter spikes
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

// SAFETY FIX: Added proper Active Braking for TB6612FNG
void brakeMotors() {
  digitalWrite(stbyPin, HIGH);
  
  // To brake the TB6612FNG, both IN pins must be HIGH
  digitalWrite(ain1Pin, HIGH);
  digitalWrite(ain2Pin, HIGH);
  analogWrite(pwmaPin, 255); 
  
  digitalWrite(bin1Pin, HIGH);
  digitalWrite(bin2Pin, HIGH);
  analogWrite(pwmbPin, 255); 
}

// Stop function coasts the motors (High Impedance)
void stopMotors() {
  digitalWrite(stbyPin, LOW); // Entering standby safely cuts outputs
  analogWrite(pwmaPin, 0);
  analogWrite(pwmbPin, 0);
}
