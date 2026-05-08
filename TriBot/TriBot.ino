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
// Input Pins
const int buttonPin = 11; // Activator Button (MOVED FROM D13 TO D11)
const int dipPin3 = 3;    // DIP Switch Bit 1 (D3)
const int dipPin2 = 2;    // DIP Switch Bit 0 (D2)

// Motor Driver (TB6612FNG)
const int pwmaPin = 9;  
const int ain1Pin = 4;  
const int ain2Pin = 5;
const int stbyPin = 6;
const int pwmbPin = 10; 
const int bin1Pin = 7;  
const int bin2Pin = 8;

// QTR Sensors
const uint8_t SensorCount = 4;
const uint8_t sensorPins[SensorCount] = {A1, A0, A5, A4};
const int EmitterCtrl = 12;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

// Sumo & Balloon Sensors
const int leftIrPin = A6;      // Sharp IR Left
const int rightIrPin = A7;     // Sharp IR Right
const int leftUsEcho = A2;     // Ultrasonic Echo Left
const int rightUsEcho = A3;    // Ultrasonic Echo Right
const int sharedUsTrig = 13;   // Shared Ultrasonic Trigger (MOVED FROM D11 TO D13)

// Global Button Variables
bool lastButtonReading = LOW;
bool buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600);

  // Motor Pins
  pinMode(pwmaPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(ain2Pin, OUTPUT);
  pinMode(pwmbPin, OUTPUT);
  pinMode(bin1Pin, OUTPUT);
  pinMode(bin2Pin, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  // Input Pins
  pinMode(buttonPin, INPUT_PULLUP); 
  pinMode(dipPin3, INPUT_PULLUP); 
  pinMode(dipPin2, INPUT_PULLUP); 

  // QTR Setup
  pinMode(EmitterCtrl, OUTPUT);
  digitalWrite(EmitterCtrl, HIGH);
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, SensorCount);

  // Sumo Setup
  pinMode(sharedUsTrig, OUTPUT);
  pinMode(leftUsEcho, INPUT);
  pinMode(rightUsEcho, INPUT);

  stopMotors();
  Serial.println("System Powered On. Select mode via DIP and press D11.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  handleMasterButton();

  // Route logic based on current selected mode
  if (currentMode == MODE_LINE_FOLLOWER) {
    loopLineFollower();
  } 
  else if (currentMode == MODE_SUMO) {
    loopSumo();
  } 
  else if (currentMode == MODE_BALLOON) {
    loopBalloon();
  }
}

// ==========================================
// MASTER STATE MACHINE
// ==========================================
RobotMode getDipSwitchMode() {
  bool d3 = !digitalRead(dipPin3);
  bool d2 = !digitalRead(dipPin2);
  
  if (!d3 && !d2) return MODE_STANDBY;       // 00
  if (d3 && !d2)  return MODE_LINE_FOLLOWER; // 01 (D3=1, D2=0)
  if (!d3 && d2)  return MODE_SUMO;          // 10 (D3=0, D2=1)
  if (d3 && d2)   return MODE_BALLOON;       // 11
  
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
      
      // When button is firmly pressed
      if (buttonState == HIGH) { 
        RobotMode requestedMode = getDipSwitchMode();

        // 1. Did the mode switch change since last press?
        if (requestedMode != currentMode) {
          stopMotors();
          currentMode = requestedMode;
          currentState = STANDBY_UNCALIBRATED;
          Serial.print("Mode Locked In: "); Serial.println(currentMode);
        } 
        // 2. Mode is the same, advance the state machine
        else {
          if (currentMode == MODE_STANDBY) {
            Serial.println("Currently in Standby (00). Do nothing.");
          }
          else if (currentState == STANDBY_UNCALIBRATED) {
            currentState = CALIBRATING;
            Serial.println("State: CALIBRATING");
          } 
          else if (currentState == STANDBY_READY) {
            currentState = PLAYING;
            Serial.println("State: PLAYING");
          } 
          else if (currentState == PLAYING) {
            brakeMotors();
            delay(100);
            stopMotors();
            currentState = STANDBY_READY;
            Serial.println("State: STOPPED -> READY");
          }
        }
      }
    }
  }
  lastButtonReading = reading;
}