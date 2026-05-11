// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

int lf_baseSpeed = 200; // Sprint speed for the straights
int lf_maxSpeed = 255;  

// Restored Baseline Tuning
float lf_Kp = 0.04;   
float lf_Ki = 0.0;     
float lf_Kd = 3.0;    // Aggressive shock absorber

int lf_lastError = 0;
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
        lf_needsKickoff = false; 
      }
      
      // Strict 5ms Timing Loop
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
  setMotors(-70, 70);
  for (int i = 0; i < 30; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); 
  setMotors(70, -70);
  for (int i = 0; i < 60; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); 
  setMotors(-70, 70);
  for (int i = 0; i < 30; i++) { qtr.calibrate(); delay(5); }

  brakeMotors(); delay(100); stopMotors();
}

// ---------------------------------------------------------
// PURE PID LOGIC & EDGE RECOVERY
// ---------------------------------------------------------
void followLinePID() {
  if (currentState != PLAYING) return; 

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // --- TIGHT PIVOT EDGE RECOVERY ---
  // We use -60 to pull the nose around fast enough for the zig-zags, 
  // without going into a full -100 death spin or a 0 overshoot.
  if (position == 0) {
    setMotors(-60, 160); 
    return; 
  } 
  else if (position == 3000) {
    setMotors(160, -60); 
    return;
  }

  // --- DYNAMIC BRAKING ---
  int currentBaseSpeed = lf_baseSpeed;
  // Brake slightly earlier (600) so it doesn't fly into the zig-zags too fast
  if (abs(error) > 600) currentBaseSpeed = 60;  
  else if (abs(error) > 200) currentBaseSpeed = 120; 

  // --- CORE PID MATH ---
  int P = error;
  int D = error - lf_lastError;
  
  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  // --- MOTOR CONSTRAINTS ---
  // Allow the inner wheel to reverse slightly (-60) for tight corners during normal PID math
  leftMotorSpeed = constrain(leftMotorSpeed, -60, lf_maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -60, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}