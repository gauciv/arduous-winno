// ==========================================
// SUMO VARIABLES
// ==========================================
const int sumo_usAttackDistanceCm = 60; 

// RC QTR Sensors: 0-2500. Lower means more reflective (White border).
const int sumo_edgeWhiteThreshold = 300; 

// Flag to ensure the 5-second countdown only happens once per match
bool sumo_needsKickoff = true; 

// Note: sumo_irAttackThresholdLeft and sumo_irAttackThresholdRight 
// are already declared in TriBot.ino because they are set during Global Calibration.

// ==========================================
// SUMO ROUTINES
// ==========================================
void loopSumo() {
  switch (currentState) {
    case STANDBY_UNCALIBRATED: 
    case STANDBY_READY: 
      // Reset the 5-second timer flag anytime we go back to standby
      sumo_needsKickoff = true; 
      break;
      
    case CALIBRATING:
      // Handled globally in TriBot.ino now
      break;
      
    case PLAYING:
      // --- THE 5-SECOND TOURNAMENT DELAY ---
      if (sumo_needsKickoff) {
        Serial.println("[SUMO] 5-Second Start Delay Initiated...");
        
        // Uses the safeDelay from TriBot.ino so the kill switch works during countdown
        if (!safeDelay(5000)) return; 
        
        Serial.println("[SUMO] FIGHT!");
        sumo_needsKickoff = false;
      }
      
      executeCombatLogic();
      break;
  }
}

// ---------------------------------------------------------
// COMBAT LOGIC
// ---------------------------------------------------------
void executeCombatLogic() {
  // FINAL SAFETY CHECK: Ensure we are still playing in case button was pressed
  handleMasterButton();
  if (currentState != PLAYING) return;

  // ---------------------------------------------------------
  // 1. RING EDGE DETECTION (SURVIVAL FIRST)
  // ---------------------------------------------------------
  qtr.read(sensorValues); 
  
  if (sensorValues[0] < sumo_edgeWhiteThreshold || sensorValues[3] < sumo_edgeWhiteThreshold) {
    Serial.println("[SUMO] EDGE DETECTED! Evading...");
    
    // HARD BRAKE to kill forward momentum
    brakeMotors();
    // safeDelay prevents the motor driver from drawing a massive stall current spike
    if (!safeDelay(150)) return; 

    // REVERSE HARD
    setMotors(-255, -255); 
    if (!safeDelay(350)) return;

    // SPIN BACK INTO RING
    setMotors(200, -200); 
    if (!safeDelay(400)) return;

    brakeMotors();
    return; // Exit loop early so it doesn't try to attack while saving itself
  }

  // ---------------------------------------------------------
  // 2. HUNTER-SEEKER LOGIC (ATTACK SECOND)
  // ---------------------------------------------------------
  int leftIrRaw = analogRead(leftIrPin);
  int rightIrRaw = analogRead(rightIrPin);
  
  // NOTE: If your robot attacks empty space instead of objects, 
  // flip these from ">" to "<" depending on your IR module's logic.
  bool enemyFrontLeft = (leftIrRaw > sumo_irAttackThresholdLeft);
  bool enemyFrontRight = (rightIrRaw > sumo_irAttackThresholdRight);
  
  // ULTRASONIC PINGS (With Anti-Crosstalk Delays)
  int leftUsDist = getUltrasonicDistance(sharedUsTrig, leftUsEcho);
  delay(15); // CRITICAL: Let the left ping echoes fade away from the room
  int rightUsDist = getUltrasonicDistance(sharedUsTrig, rightUsEcho);
  delay(15); // Let the right ping echoes fade away

  bool enemySideLeft = (leftUsDist > 0 && leftUsDist < sumo_usAttackDistanceCm);
  bool enemySideRight = (rightUsDist > 0 && rightUsDist < sumo_usAttackDistanceCm);

  // --- COMBAT DECISION TREE ---
  if (enemyFrontLeft && enemyFrontRight) {
    // Enemy dead center -> RAM FULL SPEED
    setMotors(255, 255); 
  } 
  else if (enemyFrontLeft) {
    // Enemy slightly left -> CHASE
    setMotors(120, 255); 
  } 
  else if (enemyFrontRight) {
    // Enemy slightly right -> CHASE
    setMotors(255, 120); 
  } 
  else if (enemySideLeft) {
    // Enemy on left blind spot -> Snap to them
    setMotors(-200, 200); 
  } 
  else if (enemySideRight) {
    // Enemy on right blind spot -> Snap to them
    setMotors(200, -200); 
  } 
  else {
    // SEARCH MODE -> Slow spin to find target
    setMotors(100, -100); 
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

  // 8000 microsecond timeout = roughly 130cm range. 
  // Keeps the code fast so the robot isn't waiting for sound to cross the room.
  long duration = pulseIn(echoPin, HIGH, 8000); 
  if (duration == 0) return 999; 
  return duration * 0.034 / 2;
}