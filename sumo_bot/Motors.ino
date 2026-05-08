// ==========================================
// MOTOR HELPER FUNCTIONS 
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
  // Wake up the motor driver
  digitalWrite(stbyPin, HIGH);
  
  // Safety constraint to prevent invalid PWM values
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // --- Left Motor ---
  if (leftSpeed >= 0) {
    digitalWrite(ain1Pin, LOW);
    digitalWrite(ain2Pin, HIGH); 
    analogWrite(pwmaPin, leftSpeed);
  } else {
    digitalWrite(ain1Pin, HIGH); 
    digitalWrite(ain2Pin, LOW);  
    analogWrite(pwmaPin, -leftSpeed);
  }

  // --- Right Motor ---
  if (rightSpeed >= 0) {
    digitalWrite(bin1Pin, LOW);  
    digitalWrite(bin2Pin, HIGH); 
    analogWrite(pwmbPin, rightSpeed);
  } else {
    digitalWrite(bin1Pin, HIGH); 
    digitalWrite(bin2Pin, LOW);  
    analogWrite(pwmbPin, -rightSpeed);
  }
}

// Active braking kills momentum instantly (crucial for Sumo and 90-degree line turns)
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

// Stopping allows the motors to coast (useful for entering standby)
void stopMotors() {
  // Pulling standby LOW safely cuts all outputs
  digitalWrite(stbyPin, LOW);
  analogWrite(pwmaPin, 0);
  analogWrite(pwmbPin, 0);
}