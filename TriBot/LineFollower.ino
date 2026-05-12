// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

int lf_baseSpeed = 200; // Sprint speed for the straights
int lf_maxSpeed = 255;  

// Smoothed Tuning
float lf_Kp = 0.04;   
float lf_Ki = 0.0;     
float lf_Kd = 2.5;    // Dropped slightly to stop the twitching

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

  // --- 1. THE "ANCHOR" PIVOT ---
  // Keeps the nose tight on the zig-zags if it slips off
  if (position == 0) {
    setMotors(-50, 90); 
    return; 
  } 
  else if (position == 3000) {
    setMotors(90, -50); 
    return;
  }

  // --- 2. SMOOTH 3-STAGE DYNAMIC BRAKING ---
  int currentBaseSpeed = lf_baseSpeed;
  
  if (abs(error) > 800) {
    currentBaseSpeed = 50;  // Tip of the zig-zag: Maximum brake
  } 
  else if (abs(error) > 500) {
    currentBaseSpeed = 100; // Entering sharp curve: Heavy brake
  } 
  else if (abs(error) > 250) {
    currentBaseSpeed = 160; // Approaching curve: Light brake
  }
  // If error is < 250, it STAYS at 200. This is the "Deadband" that stops the stuttering.

  // --- 3. CORE PID MATH ---
  int P = error;
  int D = error - lf_lastError;
  
  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  // --- 4. MOTOR CONSTRAINTS ---
  // Inner wheel allowed to reverse slightly (-50) to assist cornering
  leftMotorSpeed = constrain(leftMotorSpeed, -50, lf_maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -50, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}