#include <AccelStepper.h>

int motorPinSleep      = 5;
int motorPinDirection  = 6;
int motorPinStep       = 7;

int followSwitchPin    = 8;
int followLedPin       = 9;
int fastCcSwitchPin    = 10;
int fastCSwitchPin     = 11;
int fastLedPin         = 12;
int testLedPin         = 13;

bool followModeEnabled = false;
int followSwitchLast = LOW;
int followSwitchCurrent = LOW;
bool followSwitchBlocked = false;
unsigned long followSwitchLastCheck = 0;
int directionCc = -1;
int directionC = 1;

bool fastModeEnabled = false;
bool fastModeEnabledLast = false;
unsigned long fastSwitchLastCheck = 0;
int fastSwitchLast = LOW;
int fastSwitchCurrent = LOW;
int fastModeDirection = directionCc;

int fastModeSpeedMax = 1000;
int fastModeSpeedCurrent = 0;
int fastModeSpeedAccelerationSteps = 200;
int fastModeSpeedAccelerationMillis = 1000;
unsigned long fastModeSpeedLastChanged = 0;

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
}

void loop()
{  
  checkFollowButton();
  handleFollowMode();
  checkFastButtons();
  handleFastMode();
  handleMotorPower();
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

  followModeEnabled = followSwitchCurrent == LOW && !followSwitchBlocked ? !followModeEnabled : followModeEnabled;
  followSwitchBlocked = followSwitchCurrent == HIGH ? false : true;
}

void handleFollowMode()
{

  if (followModeEnabled && !fastModeEnabled) {
    stepper.setMaxSpeed(250 * directionC);
    stepper.setSpeed(250 * directionC);
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

  fastModeEnabled = fastSwitchCurrent == LOW ? true : false;
  
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
