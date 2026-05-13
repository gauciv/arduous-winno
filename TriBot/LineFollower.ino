// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;  
int lf_rightMotorOffset = 0;

// The old follower felt slow because it spent a lot of time braking to 90
// or spinning in place. A steadier forward speed is usually faster in practice.
int lf_straightSpeed = 200;
int lf_mediumCurveSpeed = 175;
int lf_tightCurveSpeed = 145;
int lf_sharpTurnSpeed = 115;
int lf_searchSpeed = 105;
int lf_maxSpeed = 230;  

// Four front-mounted sensors benefit more from a calm PD loop than from a
// very aggressive PID copied from a six-sensor robot.
float lf_Kp = 0.11;   
float lf_Ki = 0.0;    
float lf_Kd = 0.32;   
float lf_filteredDerivative = 0.0;
const float lf_derivativeFilter = 0.7;

int lf_lastError = 0;
int lf_lastSeenError = 0;
long lf_integral = 0;              
const long lf_integralMax = 8000; 

bool lf_needsKickoff = true; 
bool lf_lineWasVisible = false;
unsigned long lf_lineLostAt = 0;

// Strict Timing Loop
unsigned long lf_lastLoopTime = 0;
const unsigned long lf_loopInterval = 4;

const int lf_centerPosition = 1500;
const int lf_centerDeadband = 70;
const int lf_lineVisibleThreshold = 220;
const int lf_lineStrengthThreshold = 260;
const int lf_integralZone = 180;

void resetLfController() {
  lf_lastError = 0;
  lf_lastSeenError = 0;
  lf_integral = 0;
  lf_filteredDerivative = 0.0;
  lf_lineWasVisible = false;
  lf_lineLostAt = 0;
  lf_lastLoopTime = 0;
}

bool readLfLine(int &error, uint16_t &lineStrength, uint16_t &peakValue) {
  qtr.readCalibrated(sensorValues);

  uint32_t weightedSum = 0;
  lineStrength = 0;
  peakValue = 0;

  for (uint8_t i = 0; i < SensorCount; i++) {
    uint16_t value = sensorValues[i];

    if (value > peakValue) {
      peakValue = value;
    }

    if (value > 50) {
      weightedSum += (uint32_t)value * (i * 1000);
      lineStrength += value;
    }
  }

  if (peakValue < lf_lineVisibleThreshold || lineStrength < lf_lineStrengthThreshold) {
    return false;
  }

  int position = weightedSum / lineStrength;
  error = position - lf_centerPosition;

  if (abs(error) < lf_centerDeadband) {
    error = 0;
  }

  if (error != 0) {
    lf_lastSeenError = error;
  }

  return true;
}

int getLfBaseSpeed(int absError, uint16_t lineStrength, uint16_t peakValue) {
  int baseSpeed = lf_straightSpeed;

  if (absError > 1050) {
    baseSpeed = lf_sharpTurnSpeed;
  } else if (absError > 650) {
    baseSpeed = lf_tightCurveSpeed;
  } else if (absError > 250) {
    baseSpeed = lf_mediumCurveSpeed;
  }

  // When the line signature is weak, slow down a bit so recovery starts sooner.
  if (peakValue < 350 || lineStrength < 550) {
    baseSpeed = min(baseSpeed, lf_tightCurveSpeed);
  }

  return baseSpeed;
}

void recoverLfLine() {
  if (lf_lineWasVisible || lf_lineLostAt == 0) {
    lf_lineLostAt = millis();
    lf_lineWasVisible = false;
  }

  unsigned long lostFor = millis() - lf_lineLostAt;
  bool lastSeenRight = (lf_lastSeenError >= 0);

  // Stage 1: keep moving with a slight bias to bridge tiny gaps.
  if (lostFor < 90) {
    if (lastSeenRight) setMotors(lf_searchSpeed + 15, lf_searchSpeed - 15);
    else setMotors(lf_searchSpeed - 15, lf_searchSpeed + 15);
    return;
  }

  // Stage 2: arc back toward the last known line side.
  if (lostFor < 260) {
    if (lastSeenRight) setMotors(150, 25);
    else setMotors(25, 150);
    return;
  }

  // Stage 3: allow a small reverse on the inner wheel for sharper recovery,
  // but avoid the old full-speed spin that caused 360s.
  if (lostFor < 600) {
    if (lastSeenRight) setMotors(150, -35);
    else setMotors(-35, 150);
    return;
  }

  // If the first guess was wrong, alternate the search direction slowly.
  bool searchRight = ((lostFor / 220UL) % 2 == 0) ? lastSeenRight : !lastSeenRight;
  if (searchRight) setMotors(130, -45);
  else setMotors(-45, 130);
}

// ==========================================
// LINE FOLLOWER MAIN ROUTINE
// ==========================================
void loopLineFollower() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED: 
    case STANDBY_READY:
      lf_needsKickoff = true; 
      resetLfController();
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
        setMotors(170, 170);
        if (!safeDelay(250)) return; 

        resetLfController();
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
  qtr.resetCalibration();
  delay(600); 

  setMotors(-85, 85);
  for (int i = 0; i < 50; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(250); 
  setMotors(85, -85);
  for (int i = 0; i < 100; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(250); 
  setMotors(-85, 85);
  for (int i = 0; i < 50; i++) { qtr.calibrate(); delay(5); }

  brakeMotors(); delay(100); stopMotors();
}

// ---------------------------------------------------------
// LINE FOLLOWER CONTROL
// ---------------------------------------------------------
void followLinePID() {
  if (currentState != PLAYING) return; 

  int error = 0;
  uint16_t lineStrength = 0;
  uint16_t peakValue = 0;
  bool lineVisible = readLfLine(error, lineStrength, peakValue);

  if (!lineVisible) {
    lf_integral = 0;
    lf_filteredDerivative = 0.0;
    recoverLfLine();
    return;
  }

  lf_lineWasVisible = true;
  lf_lineLostAt = 0;

  int absError = abs(error);
  int currentBaseSpeed = getLfBaseSpeed(absError, lineStrength, peakValue);

  // Only integrate when we are almost centered; otherwise the I-term just
  // stores turning demand that comes back as overshoot.
  if (absError < lf_integralZone) {
    lf_integral += error;
    lf_integral = constrain(lf_integral, -lf_integralMax, lf_integralMax);
  } else {
    lf_integral = 0;
  }

  if ((lf_lastError < 0 && error > 0) || (lf_lastError > 0 && error < 0)) {
    lf_integral = 0;
  }

  int P = error;
  int I = lf_integral;
  int rawDerivative = error - lf_lastError;
  lf_filteredDerivative =
    (lf_derivativeFilter * lf_filteredDerivative) +
    ((1.0 - lf_derivativeFilter) * rawDerivative);

  int motorSpeedAdjustment =
    (int)((lf_Kp * P) + (lf_Ki * I) + (lf_Kd * lf_filteredDerivative));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  // Keep both wheels forward on normal corrections. Only allow a small reverse
  // when the line is far off-center.
  int minMotorSpeed = (absError > 1000) ? -60 : 0;
  leftMotorSpeed = constrain(leftMotorSpeed, minMotorSpeed, lf_maxSpeed); 
  rightMotorSpeed = constrain(rightMotorSpeed, minMotorSpeed, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}
