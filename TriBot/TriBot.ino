#include <QTRSensors.h>

// ==========================================
// GLOBAL DEFINITIONS & STATES
// ==========================================
enum RobotMode {
  MODE_STANDBY,
  MODE_LINE_FOLLOWER,
  MODE_SUMO
};

enum RobotState {
  STANDBY_UNCALIBRATED,
  CALIBRATING,
  STANDBY_READY,
  PLAYING
};

RobotMode currentMode = MODE_STANDBY;
RobotState currentState = STANDBY_UNCALIBRATED;
bool isGloballyCalibrated = false; // Calibration Persistence Flag

// ==========================================
// GLOBAL SENSOR THRESHOLDS (Set by Calibration)
// ==========================================
int sumo_irAttackThresholdLeft = 300;  
int sumo_irAttackThresholdRight = 300;

// ==========================================
// PIN DEFINITIONS (MASTER)
// ==========================================
const int buttonPin = 11; 
const int dipPin3 = 3;    
const int dipPin2 = 2;    

const int pwmaPin = 9;  
const int ain1Pin = 4;  
const int ain2Pin = 5;
const int stbyPin = 6;
const int pwmbPin = 10; 
const int bin1Pin = 7;  
const int bin2Pin = 8;

const uint8_t SensorCount = 4;
const uint8_t sensorPins[SensorCount] = {A1, A0, A5, A4};
const int EmitterCtrl = 12;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

const int leftIrPin = A6;      
const int rightIrPin = A7;     
const int leftUsEcho = A2;     
const int rightUsEcho = A3;    
const int sharedUsTrig = 13;   

bool lastButtonReading = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);

  pinMode(pwmaPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  // ALL inputs set to PULLUP
  pinMode(buttonPin, INPUT_PULLUP); 
  pinMode(dipPin3, INPUT_PULLUP); 
  pinMode(dipPin2, INPUT_PULLUP); 

  pinMode(EmitterCtrl, OUTPUT);
  digitalWrite(EmitterCtrl, HIGH);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  pinMode(sharedUsTrig, OUTPUT);
  pinMode(leftUsEcho, INPUT);
  pinMode(rightUsEcho, INPUT);

  stopMotors();
  Serial.println("=== SYSTEM POWERED ON ===");
  Serial.println("Select mode via DIP and press D11.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  handleMasterButton();

  // If triggered, run the global calibration right here in the master file
  if (currentState == CALIBRATING) {
    runGlobalCalibration();
  }

  // Route to the active mode's logic (These will now only handle PLAYING logic)
  if (currentMode == MODE_LINE_FOLLOWER) {
    loopLineFollower();
  } 
  else if (currentMode == MODE_SUMO) {
    loopSumo();
  }
}

// ==========================================
// MASTER STATE MACHINE
// ==========================================
RobotMode getDipSwitchMode() {
  // digitalRead with INPUT_PULLUP is LOW (0) when the switch is ON.
  // We invert it so: 1 = Switch ON, 0 = Switch OFF
  int d2 = !digitalRead(dipPin2); 
  int d3 = !digitalRead(dipPin3); 
  
  if (d2 == 0 && d3 == 0) return MODE_STANDBY;       
  if (d2 == 0 && d3 == 1) return MODE_LINE_FOLLOWER; 
  if (d2 == 1 && d3 == 0) return MODE_SUMO;          
  
  return MODE_STANDBY; // Fallback for 11
}

void handleMasterButton() {
  bool reading = digitalRead(buttonPin);
  
  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) { 
        RobotMode requestedMode = getDipSwitchMode();

        // SCENARIO 1: SWITCHING TO A NEW MODE
        if (requestedMode != currentMode) {
          stopMotors();
          currentMode = requestedMode;
          
          Serial.print(">>> MODE SWITCHED TO: ");
          if (currentMode == MODE_LINE_FOLLOWER) Serial.println("LINE FOLLOWER (01)");
          else if (currentMode == MODE_SUMO) Serial.println("SUMO BOT (10)");
          else Serial.println("STANDBY (00)");

          if (currentMode == MODE_STANDBY) {
            currentState = STANDBY_UNCALIBRATED;
          } 
          else {
            // Check persistence flag
            if (!isGloballyCalibrated) {
              currentState = STANDBY_UNCALIBRATED;
              Serial.println(">>> WAITING FOR START PRESS TO BEGIN CALIBRATION.");
            } else {
              currentState = STANDBY_READY;
              Serial.println(">>> ALREADY CALIBRATED. WAITING FOR START PRESS TO PLAY.");
            }
          }
        } 
        // SCENARIO 2: PROGRESSING THE STATE IN THE CURRENT MODE
        else {
          if (currentMode == MODE_STANDBY) {
            Serial.println(">>> IGNORING (Standby Mode 00)");
          }
          else if (currentState == STANDBY_UNCALIBRATED) {
            currentState = CALIBRATING;
            Serial.println(">>> STATE TRIGGERED: CALIBRATING");
          } 
          else if (currentState == STANDBY_READY) {
            currentState = PLAYING;
            Serial.println(">>> STATE TRIGGERED: PLAYING");
          } 
          else if (currentState == PLAYING) {
            Serial.println(">>> EMERGENCY STOP TRIGGERED!");
            brakeMotors();
            delay(100);
            stopMotors();
            currentState = STANDBY_READY; // Ready to start again without recalibrating
            Serial.println(">>> STATE RETURNED TO: READY");
          }
        }
      }
    }
  }
  lastButtonReading = reading;
}

// ==========================================
// GLOBAL CALIBRATION ROUTINE
// ==========================================
void runGlobalCalibration() {
  Serial.println(">>> CALIBRATION PHASE 1: QTR WIGGLE");
  delay(1000); 
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); 
  setMotors(90, -90);
  for (int i = 0; i < 80; i++) { qtr.calibrate(); delay(5); }
  
  brakeMotors(); delay(150); 
  setMotors(-90, 90);
  for (int i = 0; i < 40; i++) { qtr.calibrate(); delay(5); }

  brakeMotors(); delay(200); stopMotors();

  Serial.println(">>> CALIBRATION PHASE 2: SUMO IR SCAN");
  long totalLeft = 0;
  long totalRight = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++) {
    totalLeft += analogRead(leftIrPin);
    totalRight += analogRead(rightIrPin);
    delay(20); 
  }
  sumo_irAttackThresholdLeft = (totalLeft / samples) + 40; 
  sumo_irAttackThresholdRight = (totalRight / samples) + 40;

  Serial.println(">>> GLOBAL CALIBRATION COMPLETE. FULL STOP.");
  
  // Set persistence flag so we never have to do this again unless reset
  isGloballyCalibrated = true;
  currentState = STANDBY_READY; 
}

// ==========================================
// GLOBAL UTILITIES
// ==========================================
// safeDelay is placed here so LineFollower and Sumo can both use it
bool safeDelay(unsigned long waitTime) {
  unsigned long start = millis();
  while ((millis() - start) < waitTime) {
    handleMasterButton();
    if (currentState != PLAYING) {
      Serial.println(">>> safeDelay INTERRUPTED by Master Button!");
      return false; 
    }
  }
  return true; 
}