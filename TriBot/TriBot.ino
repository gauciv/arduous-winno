#include <QTRSensors.h>

// ==========================================
// GLOBAL DEFINITIONS & STATES
// ==========================================
enum RobotMode {
  MODE_STANDBY,
  MODE_LINE_FOLLOWER,
  MODE_SUMO,
  MODE_BALLOON
};

enum RobotState {
  STANDBY_UNCALIBRATED,
  CALIBRATING,
  STANDBY_READY,
  PLAYING
};

RobotMode currentMode = MODE_STANDBY;
RobotState currentState = STANDBY_UNCALIBRATED;

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

  if (currentMode == MODE_LINE_FOLLOWER) loopLineFollower();
  else if (currentMode == MODE_SUMO) loopSumo();
  else if (currentMode == MODE_BALLOON) loopBalloon();
}

// ==========================================
// MASTER STATE MACHINE
// ==========================================
RobotMode getDipSwitchMode() {
  bool d3 = !digitalRead(dipPin3);
  bool d2 = !digitalRead(dipPin2);
  
  if (!d3 && !d2) return MODE_STANDBY;       
  if (d3 && !d2)  return MODE_LINE_FOLLOWER; 
  if (!d3 && d2)  return MODE_SUMO;          
  if (d3 && d2)   return MODE_BALLOON;       
  
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
      
      // DEBUG PRINT: See exactly when the button registers
      Serial.print(">>> BUTTON SENSOR: "); 
      Serial.println(buttonState == LOW ? "PRESSED (LOW)" : "RELEASED (HIGH)");
      
      // FIX: Since it's INPUT_PULLUP, pressing the button connects it to ground (LOW)
      if (buttonState == LOW) { 
        Serial.println(">>> BUTTON FIRED COMMAND!");
        
        RobotMode requestedMode = getDipSwitchMode();

        if (requestedMode != currentMode) {
          stopMotors();
          currentMode = requestedMode;
          currentState = STANDBY_UNCALIBRATED;
          Serial.print(">>> MODE LOCKED: "); Serial.println(currentMode);
        } 
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