// ==========================================
// SUMO VARIABLES
// ==========================================
int irAttackThresholdLeft = 300;  
int irAttackThresholdRight = 300;
const int usAttackDistanceCm = 60; 
const int sumoEdgeWhiteThreshold = 300; // Adjust based on your ring's white line

// ==========================================
// SUMO ROUTINES
// ==========================================
void loopSumo() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED: break;
    case CALIBRATING:
      runSumoCalibrationRoutine();
      currentState = STANDBY_READY;
      break;
    case STANDBY_READY: break;
    case PLAYING:
      executeCombatLogic();
      break;
  }
}

void runSumoCalibrationRoutine() {
  Serial.println("Calibrating Sumo IR Sensors... Keep front clear.");
  long totalLeft = 0;
  long totalRight = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++) {
    totalLeft += analogRead(leftIrPin);
    totalRight += analogRead(rightIrPin);
    delay(40); 
  }

  // Tweak: +40 buffer for high sensitivity
  irAttackThresholdLeft = (totalLeft / samples) + 40; 
  irAttackThresholdRight = (totalRight / samples) + 40;
  
  Serial.println("Sumo Calibration Complete.");
}

void executeCombatLogic() {
  // ---------------------------------------------------------
  // 1. RING EDGE DETECTION (QTR SENSORS)
  // ---------------------------------------------------------
  // Read raw values to detect white border of the sumo ring
  qtr.read(sensorValues); 
  
  // If outer left or outer right sensor sees the white line
  if (sensorValues[0] < sumoEdgeWhiteThreshold || sensorValues[3] < sumoEdgeWhiteThreshold) {
    brakeMotors();
    delay(50);
    // Reverse hard to avoid falling off
    setMotors(-255, -255); 
    delay(300);
    // Spin around to face back into the ring
    setMotors(200, -200); 
    delay(400);
    brakeMotors();
    return; // Exit loop early so it doesn't try to attack while saving itself
  }

  // ---------------------------------------------------------
  // 2. HUNTER-SEEKER LOGIC
  // ---------------------------------------------------------
  int leftIrRaw = analogRead(leftIrPin);
  int rightIrRaw = analogRead(rightIrPin);
  
  bool enemyFrontLeft = (leftIrRaw > irAttackThresholdLeft);
  bool enemyFrontRight = (rightIrRaw > irAttackThresholdRight);
  
  // Using the shared trigger pin for both ultrasonic readings
  int leftUsDist = getUltrasonicDistance(sharedUsTrig, leftUsEcho);
  int rightUsDist = getUltrasonicDistance(sharedUsTrig, rightUsEcho);

  bool enemySideLeft = (leftUsDist > 0 && leftUsDist < usAttackDistanceCm);
  bool enemySideRight = (rightUsDist > 0 && rightUsDist < usAttackDistanceCm);

  if (enemyFrontLeft && enemyFrontRight) {
    // Enemy dead center -> RAM FULL SPEED
    setMotors(255, 255); 
  } 
  else if (enemyFrontLeft) {
    // Enemy slightly left -> CHASE
    setMotors(100, 255); 
  } 
  else if (enemyFrontRight) {
    // Enemy slightly right -> CHASE
    setMotors(255, 100); 
  } 
  else if (enemySideLeft) {
    // Enemy on left blind spot -> Snap to them
    setMotors(-255, 255); 
  } 
  else if (enemySideRight) {
    // Enemy on right blind spot -> Snap to them
    setMotors(255, -255); 
  } 
  else {
    // SEARCH MODE -> Spin to find target
    setMotors(120, -120); 
  }
}

// ==========================================
// SENSOR HELPER FUNCTION
// ==========================================
int getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 8000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}