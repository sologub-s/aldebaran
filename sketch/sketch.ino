#include <AccelStepper.h>

int pulseGuide_LedPin  = 3;
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
float followModeSpeed = 7.427696; // if we assume that amount of teeth of wheel is 100
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

// values from ASCOM Interface
const int pulseGuide_GUIDE_NORTH        = 0;
const int pulseGuide_GUIDE_SOUTH        = 1;
const int pulseGuide_GUIDE_EAST         = 2;
const int pulseGuide_GUIDE_WEST         = 3;

int   pulseGuide_Ra_multi     = 1;
long  pulseGuide_until        = 0;
bool  pulseGuide_active       = false;
float pulseGuide_raPlusMulti  = 2;
float pulseGuide_raMinusMulti = 0;

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

  pinMode(pulseGuide_LedPin, OUTPUT);
  digitalWrite(pulseGuide_LedPin, LOW);

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
  handleSerialOutput();
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
    float speed = followModeSpeed * directionCc;
    
    if (pulseGuide_active) {
      speed *= pulseGuide_Ra_multi;
    }
    
    stepper.setMaxSpeed(speed);
    stepper.setSpeed(speed);
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

void handleSerialInput()
{
  String command = readSerialCommand();

  if (command == "") {
    return;
  }

  // "GUIDE_*_*"
  // example: GUIDE_3_1200
  if (command.length() > 6 && command.substring(0, 6) == "GUIDE_") {

      int direction = command.substring(6, 7).toInt();
      int duration = getValue(command, '_', 2).toInt();

      driver_PulseGuide(direction, duration);

      return true;
  }

  // "SPEED_*"
  // example: SPEED_7.777
  if (command.length() > 6 && command.substring(0, 6) == "SPEED_") {
    float followModeSpeed = getValue(command, '_', 1).toFloat();
    return;    
  }

  Serial.print("ERROR_UNKNOWN_COMMAND");
  Serial.println("#");
  return false;
  
}

void handleSerialOutput()
{
  driver_PulseGuide_SerialOutput();
}

String readSerialCommand()
{
  String command = "";
  
  if (Serial.available() == 0) {
    return command;
  }

  command = Serial.readStringUntil('#');
  command.trim();

  return command;
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void log(String msg)
{
  Serial.println(String("Log: ") + msg + String("#"));
}

// ASCOM driver control

void driver_PulseGuide(int direction, int duration)
{
  pulseGuide_Ra_multi = 1;
  
  // check direction and set pulseGuide_Ra_multi
  if (direction == pulseGuide_GUIDE_NORTH) {
    Serial.print("GUIDE_FAIL");
    Serial.println("#");
    return;
  }

  if (direction == pulseGuide_GUIDE_SOUTH) {
    Serial.print("GUIDE_FAIL");
    Serial.println("#");
    return;
  }
  
  if (direction == pulseGuide_GUIDE_EAST) {
    pulseGuide_Ra_multi = pulseGuide_raMinusMulti;
  }

  if (direction == pulseGuide_GUIDE_WEST) {
    pulseGuide_Ra_multi = pulseGuide_raPlusMulti;
  }
  
  // set pulseGuide_until = millis() + duration
  pulseGuide_until = millis() + duration;

  // set pulseGuide_active = true
  pulseGuide_active = true;

  digitalWrite(pulseGuide_LedPin, HIGH);
}

void driver_PulseGuide_SerialOutput()
{
  if (!pulseGuide_active) {
    return;
  }

  if (millis() > pulseGuide_until) {
    pulseGuide_Ra_multi = 1;
    pulseGuide_active = false;
    digitalWrite(pulseGuide_LedPin, LOW);
    Serial.print("GUIDE_OK");
    Serial.println("#");
  }
}
