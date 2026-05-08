// ==========================================
// BALLOON POPPER VARIABLES
// ==========================================
// We use separate variables so calibrating Sumo doesn't mess with Balloon mode
int balloonIrThresholdLeft = 300;  
int balloonIrThresholdRight = 300;
const int balloonUsDistanceCm = 80;     // Increased range to spot balloons earlier
const int balloonEdgeWhiteThreshold = 300; 

// ==========================================
// BALLOON POPPER ROUTINES
// ==========================================
void loopBalloon() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED: break;
    
    case CALIBRATING:
      runBalloonCalibration();
      currentState = STANDBY_READY;
      break;
      
    case STANDBY_READY: break;
    
    case PLAYING:
      executeBalloonLogic();
      break;
  }
}

void runBalloonCalibration() {
  Serial.println("Calibrating Balloon Sensors... Keep front clear.");
  long totalLeft = 0;
  long totalRight = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++) {
    totalLeft += analogRead(leftIrPin);
    totalRight += analogRead(rightIrPin);
    delay(40); 
  }

  // Slightly tighter buffer than Sumo to detect lighter colored balloons
  balloonIrThresholdLeft = (totalLeft / samples) + 30; 
  balloonIrThresholdRight = (totalRight / samples) + 30;
  
  Serial.println("Balloon Calibration Complete. Ready to POP!");
}

void executeBalloonLogic() {
  // ---------------------------------------------------------
  // 1. ARENA EDGE DETECTION (QTR SENSORS)
  // ---------------------------------------------------------
  // Assuming the balloon arena still has a white border to keep the bot contained
  qtr.read(sensorValues); 
  
  if (sensorValues[0] < balloonEdgeWhiteThreshold || sensorValues[3] < balloonEdgeWhiteThreshold) {
    brakeMotors();
    delay(50);
    setMotors(-255, -255); // Reverse
    delay(300);
    setMotors(200, -200);  // Turn around
    delay(400);
    brakeMotors();
    return; 
  }

  // ---------------------------------------------------------
  // 2. TARGET SEEKER LOGIC
  // ---------------------------------------------------------
  int leftIrRaw = analogRead(leftIrPin);
  int rightIrRaw = analogRead(rightIrPin);
  
  bool balloonFrontLeft = (leftIrRaw > balloonIrThresholdLeft);
  bool balloonFrontRight = (rightIrRaw > balloonIrThresholdRight);
  
  // Using the shared trigger pin from the Main file
  int leftUsDist = getUltrasonicDistance(sharedUsTrig, leftUsEcho);
  int rightUsDist = getUltrasonicDistance(sharedUsTrig, rightUsEcho);

  bool balloonSideLeft = (leftUsDist > 0 && leftUsDist < balloonUsDistanceCm);
  bool balloonSideRight = (rightUsDist > 0 && rightUsDist < balloonUsDistanceCm);

  if (balloonFrontLeft && balloonFrontRight) {
    // Target locked dead ahead -> LUNGE AT MAXIMUM SPEED
    setMotors(255, 255); 
  } 
  else if (balloonFrontLeft) {
    // Target slightly left -> SHARP ARC TO LINE UP NEEDLE
    setMotors(60, 255); 
  } 
  else if (balloonFrontRight) {
    // Target slightly right -> SHARP ARC TO LINE UP NEEDLE
    setMotors(255, 60); 
  } 
  else if (balloonSideLeft) {
    // Target far left -> SNAP TO TARGET
    setMotors(-255, 255); 
  } 
  else if (balloonSideRight) {
    // Target far right -> SNAP TO TARGET
    setMotors(255, -255); 
  } 
  else {
    // SEARCH MODE -> Faster spin to find targets quickly
    setMotors(150, -150); 
  }
}