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

// Flag to ensure it only jumps once per start
bool lf_needsKickoff = true; 

// ==========================================
// LINE FOLLOWER ROUTINES
// ==========================================
void loopLineFollower() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED: 
      lf_needsKickoff = true; // Reset the kickoff flag
      break;
      
    case CALIBRATING:
      Serial.println("[LF] Starting Calibration Routine...");
      runAutoWiggleCalibration();
      currentState = STANDBY_READY;
      Serial.println("[LF] Calibration Finished. Ready to Play.");
      break;
      
    case STANDBY_READY: break;
    
    case PLAYING:
      // THE KICKOFF JUMP: Only runs on the very first frame of PLAYING
      if (lf_needsKickoff) {
        Serial.println("[LF] Executing Kickoff Jump!");
        setMotors(150, 150);
        delay(400); 
        lf_lastError = 0;
        lf_integral = 0;
        lf_needsKickoff = false; // Prevent it from jumping again
      }
      followLinePID();
      break;
  }
}

bool safeDelay(unsigned long waitTime) {
  unsigned long start = millis();
  while ((millis() - start) < waitTime) {
    handleMasterButton();
    if (currentState != PLAYING) {
      Serial.println("[LF] safeDelay INTERRUPTED by Button!");
      return false; 
    }
  }
  return true; 
}

void runAutoWiggleCalibration() {
  delay(1000); 
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); setMotors(90, -90);
  for (int i = 0; i < 80; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }

  brakeMotors(); delay(100); stopMotors();
}

void followLinePID() {
  handleMasterButton();
  if (currentState != PLAYING) return; 

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // ---------------------------------------------------------
  // 90-DEGREE SNAP OVERRIDE
  // ---------------------------------------------------------
  if (error > 1000) {
    Serial.println("[LF] RIGHT Snap Turn");
    setMotors(200, 200); 
    if (!safeDelay(60)) return; 
    
    brakeMotors(); 
    if (!safeDelay(50)) return; 

    while (true) {
      handleMasterButton(); 
      if (currentState != PLAYING) return; 

      setMotors(160, -160);
      qtr.readCalibrated(sensorValues); 
      if (sensorValues[1] > 650 || sensorValues[2] > 650) break;
    }
    
    brakeMotors(); 
    if (!safeDelay(20)) return;
    lf_lastError = 0; lf_integral = 0; 
    return; 
  } 
  else if (error < -1000) {
    Serial.println("[LF] LEFT Snap Turn");
    setMotors(200, 200); 
    if (!safeDelay(60)) return; 
    
    brakeMotors(); 
    if (!safeDelay(50)) return; 

    while (true) {
      handleMasterButton(); 
      if (currentState != PLAYING) return; 

      setMotors(-160, 160); 
      qtr.readCalibrated(sensorValues);
      if (sensorValues[1] > 650 || sensorValues[2] > 650) break;
    }
    
    brakeMotors(); 
    if (!safeDelay(20)) return;
    lf_lastError = 0; lf_integral = 0; 
    return;
  }

  // ---------------------------------------------------------
  // CORE PID MATH
  // ---------------------------------------------------------
  int currentBaseSpeed = lf_baseSpeed;
  if (abs(error) > 750) currentBaseSpeed = 90;
  else if (abs(error) > 250) currentBaseSpeed = 150;

  if (abs(error) < 500) {
    lf_integral += error;
    lf_integral = constrain(lf_integral, -lf_integralMax, lf_integralMax);
  }
  if ((lf_lastError < 0 && error > 0) || (lf_lastError > 0 && error < 0)) {
    lf_integral = 0;
  }

  int P = error;
  int I = lf_integral;
  int D = error - lf_lastError;
  if (lf_lastError == 0 && D != 0) D = 0;

  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Ki * I) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = constrain(currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset, -100, lf_maxSpeed);
  int rightMotorSpeed = constrain(currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset, -100, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}