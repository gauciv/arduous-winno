// ==========================================
// LINE FOLLOWER VARIABLES & CONSTANTS
// ==========================================
int leftMotorOffset = 0;
int rightMotorOffset = 0;

float Kp = 0.08;
float Ki = 0.0001;
float Kd = 1.0;

int lastError = 0;
long integral = 0;
const long integralMax = 45000;

int baseSpeed = 220;
int maxSpeed = 220;

bool lineFollowerNeedsKickoff = true;

void resetLineFollowerController() {
  lastError = 0;
  integral = 0;
}

// ==========================================
// LINE FOLLOWER MAIN ROUTINE
// ==========================================
void loopLineFollower() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED:
      lineFollowerNeedsKickoff = true;
      resetLineFollowerController();
      break;

    case CALIBRATING:
      Serial.println("[LF] Starting LF Wiggle Calibration...");
      runAutoWiggleCalibration();
      isLfCalibrated = true;
      currentState = STANDBY_READY;
      lineFollowerNeedsKickoff = true;
      resetLineFollowerController();
      Serial.println("[LF] Calibration Finished. Ready to Play.");
      break;

    case STANDBY_READY:
      lineFollowerNeedsKickoff = true;
      resetLineFollowerController();
      break;

    case PLAYING:
      if (lineFollowerNeedsKickoff) {
        resetLineFollowerController();
        setMotors(150, 150);
        if (!safeDelay(400)) {
          stopMotors();
          lineFollowerNeedsKickoff = true;
          return;
        }
        lineFollowerNeedsKickoff = false;
      }

      followLinePID();
      break;
  }
}

// ==========================================
// AUTOMATIC CALIBRATION
// ==========================================
void runAutoWiggleCalibration() {
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

// ==========================================
// PURE PID LOGIC
// ==========================================
void followLinePID() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = (int)position - 1500;

  // ---------------------------------------------------------
  // 90-DEGREE SNAP OVERRIDE
  // ---------------------------------------------------------
  if (error > 1000) {
    setMotors(200, 200);
    if (!safeDelay(60)) {
      stopMotors();
      return;
    }

    while (currentState == PLAYING) {
      handleMasterButton();
      if (currentState != PLAYING) {
        stopMotors();
        return;
      }

      setMotors(220, -220);
      qtr.readCalibrated(sensorValues);

      if (sensorValues[1] > 650 || sensorValues[2] > 650) {
        break;
      }
    }

    resetLineFollowerController();
    return;
  } else if (error < -1000) {
    setMotors(200, 200);
    if (!safeDelay(60)) {
      stopMotors();
      return;
    }

    while (currentState == PLAYING) {
      handleMasterButton();
      if (currentState != PLAYING) {
        stopMotors();
        return;
      }

      setMotors(-220, 220);
      qtr.readCalibrated(sensorValues);

      if (sensorValues[1] > 650 || sensorValues[2] > 650) {
        break;
      }
    }

    resetLineFollowerController();
    return;
  }

  // ---------------------------------------------------------
  // MULTI-STAGE PROGRESSIVE BRAKING
  // ---------------------------------------------------------
  int currentBaseSpeed = baseSpeed;

  if (abs(error) > 750) {
    currentBaseSpeed = 90;
  } else if (abs(error) > 250) {
    currentBaseSpeed = 150;
  }

  // ---------------------------------------------------------
  // U-TURN ANTI-WINDUP
  // ---------------------------------------------------------
  if (abs(error) < 500) {
    integral += error;
    integral = constrain(integral, -integralMax, integralMax);
  }

  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) {
    integral = 0;
  }

  // ---------------------------------------------------------
  // CORE PID MATH
  // ---------------------------------------------------------
  int P = error;
  long I = integral;
  int D = error - lastError;

  if (lastError == 0 && D != 0) {
    D = 0;
  }

  int motorSpeedAdjustment = (int)((Kp * P) + (Ki * I) + (Kd * D));
  lastError = error;

  int leftMotorSpeed = currentBaseSpeed + motorSpeedAdjustment + leftMotorOffset;
  int rightMotorSpeed = currentBaseSpeed - motorSpeedAdjustment + rightMotorOffset;

  leftMotorSpeed = constrain(leftMotorSpeed, -150, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -150, maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}
