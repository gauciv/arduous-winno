// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

float lf_Kp = 0.09;   
float lf_Ki = 0.0001; 
float lf_Kd = 0.3;    

int lf_lastError = 0;
long lf_integral = 0;
const long lf_integralMax = 45000; 

int lf_baseSpeed = 255; 
int lf_maxSpeed = 255;  

// ==========================================
// LINE FOLLOWER ROUTINES
// ==========================================
void loopLineFollower() {
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

void runAutoWiggleCalibration() {
  Serial.println("Calibrating Line Sensors (Wiggle)...");
  delay(1000); 
  
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(5);
  }
  
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
  Serial.println("Line Calibration Complete.");
}

void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // ---------------------------------------------------------
  // 90-DEGREE SNAP OVERRIDE
  // ---------------------------------------------------------
  if (error > 1000) {
    setMotors(200, 200); delay(60); 
    brakeMotors(); delay(50);
    while (true) {
      setMotors(160, -160);
      qtr.readCalibrated(sensorValues); 
      if (sensorValues[1] > 650 || sensorValues[2] > 650) break;
    }
    brakeMotors(); delay(20);
    lf_lastError = 0; lf_integral = 0; return; 
  } 
  else if (error < -1000) {
    setMotors(200, 200); delay(60); 
    brakeMotors(); delay(50);
    while (true) {
      setMotors(-160, 160); 
      qtr.readCalibrated(sensorValues);
      if (sensorValues[1] > 650 || sensorValues[2] > 650) break;
    }
    brakeMotors(); delay(20);
    lf_lastError = 0; lf_integral = 0; return;
  }

  // ---------------------------------------------------------
  // MULTI-STAGE PROGRESSIVE BRAKING 
  // ---------------------------------------------------------
  int currentBaseSpeed = lf_baseSpeed;
  if (abs(error) > 750) currentBaseSpeed = 90;
  else if (abs(error) > 250) currentBaseSpeed = 150;

  // ---------------------------------------------------------
  // U-TURN ANTI-WINDUP
  // ---------------------------------------------------------
  if (abs(error) < 500) {
    lf_integral += error;
    lf_integral = constrain(lf_integral, -lf_integralMax, lf_integralMax);
  }
  if ((lf_lastError < 0 && error > 0) || (lf_lastError > 0 && error < 0)) {
    lf_integral = 0;
  }

  // ---------------------------------------------------------
  // CORE PID MATH
  // ---------------------------------------------------------
  int P = error;
  int I = lf_integral;
  int D = error - lf_lastError;
  if (lf_lastError == 0 && D != 0) D = 0;

  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Ki * I) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  leftMotorSpeed = constrain(leftMotorSpeed, -100, lf_maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -100, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}