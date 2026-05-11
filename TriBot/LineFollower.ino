// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

int lf_baseSpeed = 160; 
int lf_maxSpeed = 255;  

// Pure PID 
float lf_Kp = 0.05;   
float lf_Ki = 0.0;     
float lf_Kd = 1.0;    

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
      // Reset the kickoff flag so it jumps every time a new race starts
      lf_needsKickoff = true; 
      break;
      
    case CALIBRATING:
      Serial.println("[LF] Starting LF Wiggle Calibration...");
      runLfCalibrationRoutine();
      
      // Update the independent persistence flag in TriBot.ino
      isLfCalibrated = true; 
      currentState = STANDBY_READY;
      Serial.println("[LF] Calibration Finished. Ready to Play.");
      break;
    
    case PLAYING:
      // KICKOFF JUMP: Runs only on the first frame
      if (lf_needsKickoff) {
        Serial.println("[LF] Executing Kickoff Jump!");
        setMotors(150, 150);
        
        // Uses the safeDelay from TriBot.ino
        if (!safeDelay(400)) return; 
        
        lf_lastError = 0;
        lf_needsKickoff = false; 
      }
      
      // Strict 5ms Timing Loop for stable PID
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
  
  brakeMotors(); delay(150); 
  setMotors(90, -90);
  for (int i = 0; i < 80; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); 
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }

  brakeMotors(); delay(100); stopMotors();
}

// ---------------------------------------------------------
// RAW, HYPER-RESPONSIVE PID LOGIC
// ---------------------------------------------------------
void followLinePID() {
  // Final safety check in case state changed mid-loop
  if (currentState != PLAYING) return; 

  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // --- ARC-PIVOT EDGE RECOVERY (Fixes the 180-spin) ---
  // If the line is lost, we don't spin in place. We keep moving forward 
  // slightly while turning hard so we push THROUGH the U-turn.
  if (position == 0) {
    setMotors(-40, 160); 
    return; 
  } 
  else if (position == 3000) {
    setMotors(160, -40); 
    return;
  }

  // --- DYNAMIC BRAKING (Fixes the Jitter) ---
  // Thresholds widened. It will only brake on actual curves now, 
  // allowing it to glide smoothly on the straights.
  int currentBaseSpeed = lf_baseSpeed;
  if (abs(error) > 1000) currentBaseSpeed = 60;  
  else if (abs(error) > 500) currentBaseSpeed = 120; 

  // --- RAW PID MATH ---
  int P = error;
  int D = error - lf_lastError;
  
  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Kd * D));
  lf_lastError = error;

  // --- PIVOT ALLOWANCE ---
  int leftMotorSpeed = constrain(currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset, -100, lf_maxSpeed);
  int rightMotorSpeed = constrain(currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset, -100, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}