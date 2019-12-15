#include <AccelStepper.h>

int hbLedPin           = 3;
int motorPinSleep      = 5;
int motorPinDirection  = 6;
int motorPinStep       = 7;

int followSwitchPin    = 8;
int followLedPin       = 9;
int fastCcSwitchPin    = 10;
int fastCSwitchPin     = 11;
int fastLedPin         = 12;
int testLedPin         = 13;

int hbLedSpeed = 500;
unsigned long hbLastCheck = 0;
bool followModeEnabled = false;
//float followModeSpeed = 250.0;
float followModeSpeed = 7.427696;
int followSwitchLast = LOW;
int followSwitchCurrent = LOW;
bool followSwitchBlocked = false;
unsigned long followSwitchLastCheck = 0;
int directionCc = -1;
int directionC = 1;

bool fastModeEnabled = false;
bool fastModeEnabledOld = false;
bool fastModeEnabledLast = false;
unsigned long fastSwitchLastCheck = 0;
int fastSwitchLast = LOW;
int fastSwitchCurrent = LOW;
int fastModeDirection = directionCc;

int fastModeSpeedMax = 5000;
int fastModeSpeedCurrent = 0;
int fastModeSpeedAccelerationSteps = 1;
int fastModeSpeedAccelerationMillis = 3;
unsigned long fastModeSpeedLastChanged = 0;

String serialIncomingString = "";

AccelStepper stepper(AccelStepper::DRIVER, motorPinStep, motorPinDirection);

void setup()
{
  // initialize control pins
  pinMode(motorPinSleep, OUTPUT);
  digitalWrite(motorPinSleep, LOW);
  
  // Initialize follow pins
  pinMode(followSwitchPin, INPUT_PULLUP);
  pinMode(followLedPin, OUTPUT);
  digitalWrite(followLedPin, LOW);

  // Initialize fast pins
  pinMode(fastCcSwitchPin, INPUT_PULLUP);
  pinMode(fastCSwitchPin, INPUT_PULLUP);
  pinMode(fastLedPin, OUTPUT);
  digitalWrite(fastLedPin, LOW);

  pinMode(testLedPin, OUTPUT);
  digitalWrite(testLedPin, LOW);

  pinMode(hbLedPin, OUTPUT);
  digitalWrite(hbLedPin, LOW);

  // initialize serial listener
  Serial.begin(115200);

  log(String("Initialized"));
}

void loop()
{  
  checkFollowButton();
  handleFollowMode();
  checkFastButtons();
  handleFastMode();
  handleMotorPower();
  handleSerialInput();
  handleHb();
}

void handleMotorPower()
{
  digitalWrite(motorPinSleep, followModeEnabled || fastModeEnabled ? HIGH : LOW);
}

void checkFollowButton()
{
  if (millis() - followSwitchLastCheck < 5) {
    return;
  }

  followSwitchCurrent = digitalRead(followSwitchPin);
  followSwitchLastCheck = millis();
  
  if (followSwitchLast != followSwitchCurrent) {
    followSwitchLast = followSwitchCurrent;
    return;
  }

  bool followModeEnabledOld = followModeEnabled;
  bool followSwitchBlockedOld = followSwitchBlocked;
  
  followModeEnabled = followSwitchCurrent == LOW && !followSwitchBlocked ? !followModeEnabled : followModeEnabled;
  followSwitchBlocked = followSwitchCurrent == HIGH ? false : true;

  if (followModeEnabled != followModeEnabledOld) {
    log(String("followModeEnabled changed to: ") + String((followModeEnabled == true ? "ON" : "OFF")));
  }

  if (followSwitchBlocked != followSwitchBlockedOld) {
    log(String("followSwitchBlocked changed to: ") + String((followSwitchBlocked ? "ON" : "OFF")));
  }
}

void handleFollowMode()
{

  if (followModeEnabled && !fastModeEnabled) {
    stepper.setMaxSpeed(followModeSpeed * directionCc);
    stepper.setSpeed(followModeSpeed * directionCc);
    stepper.runSpeed();
  }
        
  digitalWrite(followLedPin, followModeEnabled ? HIGH : LOW);
}

void checkFastButtons()
{
  if (millis() - fastSwitchLastCheck < 5) {
    return;
  }

  fastSwitchCurrent = digitalRead(fastCcSwitchPin) * digitalRead(fastCSwitchPin) > 0 ? HIGH : LOW;
  fastSwitchLastCheck = millis();
  
  if (fastSwitchLast != fastSwitchCurrent) {
    fastSwitchLast = fastSwitchCurrent;
    return;
  }

  if(digitalRead(fastCcSwitchPin) == LOW) {
    fastModeDirection = directionCc;
  } else {
    fastModeDirection = directionC;
  }

  bool fastModeEnabledOld = fastModeEnabled;
  fastModeEnabled = fastSwitchCurrent == LOW ? true : false;

  if (fastModeEnabled != fastModeEnabledOld) {
    log(String("fastModeEnabled changed to: ") + String((fastModeEnabled == true ? "ON" : "OFF")) + String(", direction is: ") + String(fastModeDirection == directionC ? "Clockwise" : "CounterClockwise"));
  }
}

void handleFastMode()
{
  if (fastModeEnabledLast && !fastModeEnabled) {
    fastModeSpeedCurrent = 0;
  }

  if (fastModeEnabled) {

    if(millis() - fastModeSpeedLastChanged >= fastModeSpeedAccelerationMillis && fastModeSpeedCurrent < fastModeSpeedMax) {
      fastModeSpeedCurrent += fastModeSpeedAccelerationSteps;
      digitalWrite(testLedPin, !digitalRead(testLedPin));
      fastModeSpeedLastChanged = millis();
    }
        
    stepper.setMaxSpeed(fastModeSpeedCurrent * fastModeDirection);
    stepper.setSpeed(fastModeSpeedCurrent * fastModeDirection);
    stepper.runSpeed();
  }

  digitalWrite(fastLedPin, fastModeEnabled ? HIGH : LOW);
  fastModeEnabledLast = fastModeEnabled;
}

void handleHb()
{
  if (millis() - hbLastCheck < hbLedSpeed) {
    return;
  }

  hbLastCheck = millis();
  digitalWrite(hbLedPin, !digitalRead(hbLedPin));
}

void handleSerialInput()
{
  if (Serial.available() == 0) {
    return;
  }
  
  serialIncomingString = Serial.readString();
  serialIncomingString.trim();
  
  Serial.println(String("") + String("Received: ") + serialIncomingString);
  followModeSpeed = atof(serialIncomingString.c_str());
  Serial.println(String("followModeSpeed was changed to: ") + serialIncomingString + " steps per second");
}

void log(String msg)
{
  Serial.println(String("Log: ") + msg);
}
