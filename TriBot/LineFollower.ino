// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

int lf_baseSpeed = 180; 
int lf_maxSpeed = 255;  

// Pure PID 
float lf_Kp = 0.05;   
float lf_Ki = 0.0;     
float lf_Kd = 2.0;    

int lf_lastError = 0;

// Senior Smoothing Buffer
const int LF_SENSOR_SAMPLES = 3;
unsigned int lf_sensorBuffer[4][LF_SENSOR_SAMPLES]; 

// Debounce Edge Counters
int lf_leftLineLostCount = 0;
int lf_rightLineLostCount = 0;
const int lf_maxLostCount = 3; 

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
      // Handled globally in TriBot.ino now
      break;
    
    case PLAYING:
      // KICKOFF JUMP: Runs only on the first frame
      if (lf_needsKickoff) {
        Serial.println("[LF] Executing Kickoff Jump!");
        setMotors(150, 150);
        
        // Uses the safeDelay from TriBot.ino
        if (!safeDelay(400)) return; 
        
        lf_lastError = 0;
        lf_leftLineLostCount = 0; 
        lf_rightLineLostCount = 0;
        
        // Zero out the smoothing buffer
        for(int i=0; i<4; i++) {
          for(int j=0; j<LF_SENSOR_SAMPLES; j++) {
            lf_sensorBuffer[i][j] = 0;
          }
        }
        
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
// SENIOR SENSOR SMOOTHING
// ---------------------------------------------------------
void lf_smoothSensorValues() {
  for (int i = 0; i < 4; i++) { 
    for (int j = LF_SENSOR_SAMPLES - 1; j > 0; j--) {
      lf_sensorBuffer[i][j] = lf_sensorBuffer[i][j - 1];
    }
    lf_sensorBuffer[i][0] = sensorValues[i]; 
    
    int sum = 0;
    for (int j = 0; j < LF_SENSOR_SAMPLES; j++) {
      sum += lf_sensorBuffer[i][j];
    }
    sensorValues[i] = sum / LF_SENSOR_SAMPLES;
  }
}

// ---------------------------------------------------------
// PURE PID LOGIC
// ---------------------------------------------------------
void followLinePID() {
  // Final safety check in case state changed mid-loop
  if (currentState != PLAYING) return; 

  uint16_t position = qtr.readLineBlack(sensorValues);
  lf_smoothSensorValues(); 
  
  int error = position - 1500;

  // --- EDGE RECOVERY ---
  if (position == 0) {
    lf_leftLineLostCount++;
    if (lf_leftLineLostCount >= lf_maxLostCount) {
      setMotors(-100, 150); 
      return; 
    }
  } else {
    lf_leftLineLostCount = 0; 
  }

  if (position == 3000) {
    lf_rightLineLostCount++;
    if (lf_rightLineLostCount >= lf_maxLostCount) {
      setMotors(150, -100); 
      return;
    }
  } else {
    lf_rightLineLostCount = 0; 
  }

  // --- DYNAMIC BRAKING ---
  int currentBaseSpeed = lf_baseSpeed;
  if (abs(error) > 800) currentBaseSpeed = 60;  
  else if (abs(error) > 200) currentBaseSpeed = 120; 

  // --- CORE PID MATH ---
  int P = error;
  int D = error - lf_lastError;
  D = constrain(D, -250, 250); // Derivative Clamping
  
  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Kd * D));
  lf_lastError = error;

  // --- PIVOT ALLOWANCE ---
  int leftMotorSpeed = constrain(currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset, -100, lf_maxSpeed);
  int rightMotorSpeed = constrain(currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset, -100, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}