// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

int lf_baseSpeed = 200; // Sprint speed restored
int lf_maxSpeed = 255;  

// Restored Baseline Tuning
float lf_Kp = 0.04;   
float lf_Ki = 0.0;     
float lf_Kd = 3.0;    // Aggressive shock absorber for the zig-zags

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
        // The old code delayed for 400ms. We use safeDelay so the Kill Switch stays active.
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
  // Slowed to 70 (like baseline) so it doesn't twist off the line
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
// PURE PID LOGIC (Baseline Math Restored)
// ---------------------------------------------------------
void followLinePID() {
  if (currentState != PLAYING) return; 

  // The QTR library naturally outputs 0 or 3000 if it loses the line.
  // We let this feed directly into the PID math to generate a massive D-spike for tight pivots.
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = position - 1500;

  // --- HYPER-SENSITIVE DYNAMIC BRAKING ---
  int currentBaseSpeed = lf_baseSpeed;
  if (abs(error) > 800) currentBaseSpeed = 60;  // Sharp Zig-Zag: SLAM brakes
  else if (abs(error) > 200) currentBaseSpeed = 120; // Slight drift: Mild brake

  // --- CORE PID MATH ---
  int P = error;
  int D = error - lf_lastError;
  
  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  // --- SHARP PIVOT ALLOWANCE ---
  // Inner wheel allowed to reverse to -100 to yank the nose around the zig-zag
  leftMotorSpeed = constrain(leftMotorSpeed, -100, lf_maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -100, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}