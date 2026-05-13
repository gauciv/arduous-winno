// ==========================================
// LINE FOLLOWER VARIABLES
// ==========================================
int lf_leftMotorOffset = 0;
int lf_rightMotorOffset = 0;

float lf_Kp = 0.08;
float lf_Ki = 0.0004;
float lf_Kd = 1.0;

int lf_lastError = 0;
int lf_lastSeenError = 0;
long lf_integral = 0;
const long lf_integralMax = 45000;

int lf_baseSpeed = 220;
int lf_mediumCurveSpeed = 150;
int lf_hardBrakeSpeed = 90;
int lf_maxSpeed = 220;

bool lf_needsKickoff = true;
unsigned long lf_lastLoopTime = 0;
const unsigned long lf_loopInterval = 4;
unsigned long lf_lineLostAt = 0;

const int lf_centerPosition = 1500;
const int lf_mediumBrakeError = 250;
const int lf_hardBrakeError = 750;
const int lf_sharpTurnError = 1000;
const int lf_integralZone = 500;

const uint16_t lf_lineVisibleThreshold = 220;
const uint16_t lf_lineStrengthThreshold = 260;
const uint16_t lf_outerSharpThreshold = 700;
const uint16_t lf_centerReacquireThreshold = 650;
const uint16_t lf_centerWeakThreshold = 450;

void resetLfController() {
  lf_lastError = 0;
  lf_lastSeenError = 0;
  lf_integral = 0;
  lf_lastLoopTime = 0;
  lf_lineLostAt = 0;
}

bool readLfLine(int &error, uint16_t &peakValue, uint16_t &lineStrength) {
  uint16_t position = qtr.readLineBlack(sensorValues);

  peakValue = 0;
  lineStrength = 0;
  for (uint8_t i = 0; i < SensorCount; i++) {
    peakValue = max(peakValue, sensorValues[i]);
    lineStrength += sensorValues[i];
  }

  if (peakValue < lf_lineVisibleThreshold || lineStrength < lf_lineStrengthThreshold) {
    return false;
  }

  error = (int)position - lf_centerPosition;
  if (error != 0) {
    lf_lastSeenError = error;
  }
  return true;
}

bool shouldSnapLfTurn(int error) {
  bool hardRight =
    (error > lf_sharpTurnError) ||
    (sensorValues[3] > lf_outerSharpThreshold &&
     sensorValues[1] < lf_centerWeakThreshold &&
     sensorValues[2] < lf_centerWeakThreshold);

  bool hardLeft =
    (error < -lf_sharpTurnError) ||
    (sensorValues[0] > lf_outerSharpThreshold &&
     sensorValues[1] < lf_centerWeakThreshold &&
     sensorValues[2] < lf_centerWeakThreshold);

  return hardLeft || hardRight;
}

bool executeLfSharpTurn(bool turnRight) {
  setMotors(200, 200);
  if (!safeDelay(60)) return false;

  unsigned long turnStart = millis();
  while ((millis() - turnStart) < 450) {
    handleMasterButton();
    if (currentState != PLAYING) {
      stopMotors();
      return false;
    }

    if (turnRight) setMotors(220, -220);
    else setMotors(-220, 220);

    qtr.readCalibrated(sensorValues);
    if (sensorValues[1] > lf_centerReacquireThreshold || sensorValues[2] > lf_centerReacquireThreshold) {
      lf_lastError = 0;
      lf_integral = 0;
      lf_lineLostAt = 0;
      return true;
    }
  }

  if (turnRight) setMotors(175, -90);
  else setMotors(-90, 175);
  if (!safeDelay(70)) return false;

  lf_lastError = 0;
  lf_integral = 0;
  lf_lineLostAt = millis();
  return true;
}

int getLfBaseSpeed(int absError) {
  if (absError > lf_hardBrakeError) {
    return lf_hardBrakeSpeed;
  }
  if (absError > lf_mediumBrakeError) {
    return lf_mediumCurveSpeed;
  }
  return lf_baseSpeed;
}

void recoverLfLine() {
  if (lf_lineLostAt == 0) {
    lf_lineLostAt = millis();
  }

  unsigned long lostFor = millis() - lf_lineLostAt;
  bool lastSeenRight = (lf_lastSeenError >= 0);

  if (lostFor < 90) {
    if (lastSeenRight) setMotors(180, 140);
    else setMotors(140, 180);
    return;
  }

  if (lostFor < 240) {
    if (lastSeenRight) setMotors(190, 40);
    else setMotors(40, 190);
    return;
  }

  if (lostFor < 550) {
    if (lastSeenRight) setMotors(205, -105);
    else setMotors(-105, 205);
    return;
  }

  if (lastSeenRight) setMotors(180, -180);
  else setMotors(-180, 180);
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
        setMotors(150, 150);
        if (!safeDelay(400)) return;

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
  delay(1000);

  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(5);
  }

  stopMotors();
  delay(250);

  setMotors(90, -90);
  for (int i = 0; i < 80; i++) {
    qtr.calibrate();
    delay(5);
  }

  stopMotors();
  delay(250);

  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) {
    qtr.calibrate();
    delay(5);
  }

  stopMotors();
}

// ---------------------------------------------------------
// LINE FOLLOWER CONTROL
// ---------------------------------------------------------
void followLinePID() {
  if (currentState != PLAYING) return;

  int error = 0;
  uint16_t peakValue = 0;
  uint16_t lineStrength = 0;
  bool lineVisible = readLfLine(error, peakValue, lineStrength);

  if (!lineVisible) {
    lf_lastError = 0;
    lf_integral = 0;
    recoverLfLine();
    return;
  }

  lf_lineLostAt = 0;

  if (shouldSnapLfTurn(error)) {
    bool turnRight = (error >= 0 || sensorValues[3] > sensorValues[0]);
    executeLfSharpTurn(turnRight);
    return;
  }

  int currentBaseSpeed = getLfBaseSpeed(abs(error));

  if (abs(error) < lf_integralZone) {
    lf_integral += error;
    lf_integral = constrain(lf_integral, -lf_integralMax, lf_integralMax);
  }

  if ((lf_lastError < 0 && error > 0) || (lf_lastError > 0 && error < 0)) {
    lf_integral = 0;
  }

  int P = error;
  int I = lf_integral;
  int D = error - lf_lastError;

  if (lf_lastError == 0 && D != 0) {
    D = 0;
  }

  int motorSpeedAdjustment = (int)((lf_Kp * P) + (lf_Ki * I) + (lf_Kd * D));
  lf_lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + lf_leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + lf_rightMotorOffset;

  leftMotorSpeed = constrain(leftMotorSpeed, -150, lf_maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -150, lf_maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}
