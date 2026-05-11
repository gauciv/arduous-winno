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

// --- INDEPENDENT CALIBRATION PERSISTENCE ---
bool isLfCalibrated = false; 
bool isSumoCalibrated = false; 

// --- COMPETITION VARIABLES ---
// Easily change this value on tournament day
int sumoStartDelaySeconds = 5; 

// ==========================================
// GLOBAL SENSOR THRESHOLDS (Set by Sub-Modules)
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

  pinMode(pwmaPin, OUTPUT); pinMode(ain1Pin, OUTPUT); pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT); pinMode(bin1Pin, OUTPUT); pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  // ALL inputs set to PULLUP
  pinMode(buttonPin, INPUT_PULLUP); 
  pinMode(dipPin3, INPUT_PULLUP); 
  pinMode(dipPin2, INPUT_PULLUP); 

  pinMode(EmitterCtrl, OUTPUT); digitalWrite(EmitterCtrl, HIGH);
  qtr.setTypeRC(); qtr.setSensorPins(sensorPins, SensorCount);

  pinMode(sharedUsTrig, OUTPUT);
  pinMode(leftUsEcho, INPUT); pinMode(rightUsEcho, INPUT);

  stopMotors();
  Serial.println("=== SYSTEM POWERED ON ===");
  Serial.println("Select mode via DIP and press D11.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  handleMasterButton();

  // Routing directly to the modules. 
  // The modules will handle their own CALIBRATING and PLAYING logic now.
  if (currentMode == MODE_LINE_FOLLOWER) loopLineFollower();
  else if (currentMode == MODE_SUMO) loopSumo();
}

// ==========================================
// MASTER STATE MACHINE
// ==========================================
RobotMode getDipSwitchMode() {
  int rawD2 = digitalRead(dipPin2);
  int rawD3 = digitalRead(dipPin3);

  int d2 = (rawD2 == LOW) ? 1 : 0;
  int d3 = (rawD3 == LOW) ? 1 : 0;

  Serial.print(">>> [DIP READ] d2 (Pin 2): "); Serial.print(d2);
  Serial.print(" | d3 (Pin 3): "); Serial.println(d3);

  if (d2 == 0 && d3 == 0) return MODE_STANDBY;
  if (d2 == 0 && d3 == 1) return MODE_LINE_FOLLOWER;
  if (d2 == 1 && d3 == 0) return MODE_SUMO;
  
  if (d2 == 1 && d3 == 1) {
    Serial.println(">>> [WARNING] Both switches read as 1. Invalid state (11) detected. Defaulting to Standby.");
    return MODE_STANDBY; 
  }

  return MODE_STANDBY;
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
        Serial.println("\n>>> BUTTON PRESSED");
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
          else if (currentMode == MODE_LINE_FOLLOWER) {
            if (!isLfCalibrated) {
              currentState = STANDBY_UNCALIBRATED;
              Serial.println(">>> WAITING FOR START PRESS TO BEGIN LF CALIBRATION.");
            } else {
              currentState = STANDBY_READY;
              Serial.println(">>> LF ALREADY CALIBRATED. WAITING FOR START PRESS TO PLAY.");
            }
          }
          else if (currentMode == MODE_SUMO) {
            if (!isSumoCalibrated) {
              currentState = STANDBY_UNCALIBRATED;
              Serial.println(">>> WAITING FOR START PRESS TO BEGIN SUMO CALIBRATION.");
            } else {
              currentState = STANDBY_READY;
              Serial.println(">>> SUMO ALREADY CALIBRATED. WAITING FOR START PRESS TO PLAY.");
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
            currentState = STANDBY_READY; 
            Serial.println(">>> STATE RETURNED TO: READY");
          }
        }
      }
    }
  }
  lastButtonReading = reading;
}

// ==========================================
// GLOBAL UTILITIES
// ==========================================
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