// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

// Capped max speed to prevent "bursting" on curves
int lf_baseSpeed = 220; 
int lf_maxSpeed = 220;  

// Exact Reference Tuning
float lf_Kp = 0.08;   
float lf_Ki = 0.0004;     
float lf_Kd = 1.0;    

int lf_lastError = 0;
long lf_integral = 0;              
const long lf_integralMax = 45000; 

bool lf_needsKickoff = true; 

// Strict Timing Loop
unsigned long lf_lastLoopTime = 0;
const unsigned long lf_loopInterval = 5; 

// ==========================================
// LINE FOLLOWER MAIN ROUTINE
// ==========================================
void loopLineFollower() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED: 
    case STANDBY_READY:
      lf_needsKickoff = true; 
      break;
      
    case CALIBRATING:
      Serial.println("[LF] Starting LF Wiggle Calibration...");
      runLfCalibrationRoutine();
      isLfCalibrated = true; 
      currentState = STANDBY_READY;
      Serial.println("[LF] Calibration Finished. Ready to Play.");
      break;
    
    case PLAYING:
      if (lf_needsKickoff) {
        Serial.println("[LF] Executing Kickoff Jump!");
        setMotors(150, 150);
        if (!safeDelay(400)) return; 
        
        lf_lastError = 0;
        lf_integral = 0;
        lf_needsKickoff = false; 
      }
      
      if (millis() - lf_lastLoopTime >= lf_loopInterval) {
        lf_lastLoopTime = millis();
        followLinePID();
      }
      break;
  }
}

// ---------------------------------------------------------
// LINE FOLLOWER WIGGLE CALIBRATION
// ---------------------------------------------------------
void runLfCalibrationRoutine() {
  delay(1000); 
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(250); 
  setMotors(90, -90);
  for (int i = 0; i < 80; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(250); 
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }

  brakeMotors(); delay(100); stopMotors();
}

// ---------------------------------------------------------
// PURE PID LOGIC (Reference Architecture)
// ---------------------------------------------------------
void followLinePID() {
  if (currentState != PLAYING) return; 

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // --- 1. 90-DEGREE SNAP OVERRIDE (The Brute-Force Turn) ---
  // If it hits a sharp point, abandon PID. Punch forward to clear the corner,
  // then spin until the exact center sensors lock onto the line.
  if (error > 1000) {
    setMotors(200, 200); 
    if (!safeDelay(60)) return; 

    while (true) {
      // Emergency stop check inside the infinite loop
      handleMasterButton();
      if (currentState != PLAYING) return;

      setMotors(220, -220); 
      qtr.readLineBlack(sensorValues); 
      
      if (sensorValues[1] > 650 || sensorValues[2] > 650) break;
    }
    lf_lastError = 0; 
    lf_integral = 0;  
    return; 
  } 
  else if (error < -1000) {
    setMotors(200, 200); 
    if (!safeDelay(60)) return; 
    
    while (true) {
      handleMasterButton();
      if (currentState != PLAYING) return;

      setMotors(-220, 220); 
      qtr.readLineBlack(sensorValues);
      
      if (sensorValues[1] > 650 || sensorValues[2] > 650) break;
    }
    lf_lastError = 0; 
    lf_integral = 0;  
    return; 
  }

  // --- 2. MULTI-STAGE PROGRESSIVE BRAKING ---
  int currentBaseSpeed = lf_baseSpeed; 

  if (abs(error) > 750) {
    currentBaseSpeed = 90;   // Extreme corners - Hard Brake
  } 
  else if (abs(error) > 250) {
    currentBaseSpeed = 150;  // Sweeping curves - Medium Brake 
  }

  // --- 3. U-TURN ANTI-WINDUP (For the I-Term) ---
  if (abs(error) < 500) {
    lf_integral += error;
    lf_integral = constrain(lf_integral, -lf_integralMax, lf_integralMax);
  }

  // Reset integral if we cross the center line
  if ((lf_lastError < 0 && error > 0) || (lf_lastError > 0 && error < 0)) {
    lf_integral = 0;
  }

  // --- 4. CORE PID MATH & THE D-MUTE HACK ---
  int P = error;
  int I = lf_integral;
  int D = error - lf_lastError;

  // The secret hack that kills straight-line shaking:
  if (lf_lastError == 0 && D != 0) {
    D = 0;
  }

  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Ki * I) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  // --- 5. MOTOR CONSTRAINTS ---
  // Allow reversing up to -150 to yank the nose tightly around curves
  leftMotorSpeed = constrain(leftMotorSpeed, -150, lf_maxSpeed); 
  rightMotorSpeed = constrain(rightMotorSpeed, -150, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}