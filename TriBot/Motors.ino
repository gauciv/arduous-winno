// ==========================================
// MOTOR HELPER FUNCTIONS 
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
  // Wake up the motor driver
  digitalWrite(stbyPin, HIGH);
  
  // Safety constraint to prevent invalid PWM values
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // --- Left Motor (Motor B) ---
  if (leftSpeed >= 0) {
    digitalWrite(bin1Pin, LOW);  
    digitalWrite(bin2Pin, HIGH); 
    analogWrite(pwmbPin, leftSpeed);
  } else {
    digitalWrite(bin1Pin, HIGH); 
    digitalWrite(bin2Pin, LOW);  
    analogWrite(pwmbPin, -leftSpeed);
  }

  // --- Right Motor (Motor A) ---
  if (rightSpeed >= 0) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH); 
    analogWrite(pwmaPin, rightSpeed);
  } else {
    digitalWrite(ain1Pin, HIGH); 
    digitalWrite(ain2Pin, LOW);  
    analogWrite(pwmaPin, -rightSpeed);
  }
}

// ---------------------------------------------------------
// HARD BRAKE (Instant Stop)
// ---------------------------------------------------------
void brakeMotors() {
  digitalWrite(stbyPin, HIGH);
  
  // To strictly brake the TB6612FNG, both IN pins must be HIGH
  digitalWrite(ain1Pin, HIGH);
  digitalWrite(ain2Pin, HIGH);
  analogWrite(pwmaPin, 255); 
  
  digitalWrite(bin1Pin, HIGH);
  digitalWrite(bin2Pin, HIGH);
  analogWrite(pwmbPin, 255); 
}

// ---------------------------------------------------------
// COAST TO A STOP
// ---------------------------------------------------------
void stopMotors() {
  // Pulling standby LOW safely cuts all outputs
  digitalWrite(stbyPin, LOW);
  analogWrite(pwmaPin, 0);
  analogWrite(pwmbPin, 0);
}