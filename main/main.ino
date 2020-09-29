#include "timer-api.h"

#include <math.h>
//#include <IRremote.h>

#include "Command.h"
#include "Commander.h"
#include "Info.h"
#include "Informer.h"
#include "Coordinator.h"

Commander commander;
Informer informer;

unsigned long lastButtonPressedAt = 0;
int lastButtonPressed = 0;

const int serialSpeed = 19200;

const int pinRaDir = 4;
const int pinRaStp = 5;

const int pinDecDir = 6;
const int pinDecStp = 7;

const int pinIRrecv = 8;

const int pinMotorFast = 9;

//IRrecv irrecv(pinIRrecv);
//decode_results irrecvResults;
//int resultsValue = 0;

int raForward = HIGH;
int raBackward = LOW;

int decNorth = HIGH;
int decSouth = LOW;

int slewRa = 0;
unsigned long lastRaStepUpAgo = 0;
//unsigned long raFreq = 3;
unsigned long raFreq = 1;

int slewDec = 0;
unsigned long lastDecStepUpAgo = 0;
//unsigned long decFreq = 3;
unsigned long decFreq = 1;

unsigned long lastSkyMoveAgo = 0;

bool follow = false;
unsigned long lastRaFollowAgo = 0;
//float followFreq = 15582.3206 / 5; // anyway, decimal part doesn't matter...
//float followFreq = 3116.4641199999996; // anyway, decimal part doesn't matter...
//int followFreq = 3116;
int followFreq = 311; // true speed !
//int followFreq = 1;

//float raSecondsPerMicrostep = 0.00002005477925815886 * 5;
//float raSecondsPerMicrostep = 0.0001002738962907943;
///unsigned long raMicroSecondsPerMicrostep = 100;

long currentRaCoords = 0; // microsteps, max is 2764800, 0 is 00h 00m 00s on RA axis, the incresing means moving to the east
long microstepsInRa = 2764800;

long currentDecCoords = 0; // microsteps, max is 1382400, 0 is -90d (south pole), 1382400 is +90d (north pole)
//long microstepsInDec = 1382400;
long microstepsInDec = 2764800;

byte raCurrentDir = 0;
byte decCurrentDir = 0;

// n = north, s - south
char currentHemisphere = 'n';

// e - east, w - west
char currentDecWard = 'e';

bool gotoEnabled = false;

bool gotoRaEnabled = false;
long gotoRaCoords = 0;
long gotoRaSteps = 0;
int gotoRaDir = raForward;
bool gotoRaFreq = 2;

bool gotoDecEnabled = false;
long gotoDecCoords = 0;
long gotoDecSteps = 0;
int gotoDecDir = decNorth;
bool gotoDecFreq = 2;
int DEC_RIGHT = 1;

int slowRadius = 3000;
int fastMode = false;
int coordsPerStep = 1;

Coordinator coordinator(informer, microstepsInRa, microstepsInDec, currentHemisphere, currentDecWard);

void setup() {

  Serial.begin(serialSpeed);

  informer.send("SYSTEM_STARTING#");

  if (currentDecWard == 'e') {
      int decNorth = HIGH;
      int decSouth = LOW;
  } else {
      int decNorth = LOW;
      int decSouth = HIGH;
  }

  if (currentHemisphere == 's') {
    decNorth = decNorth == HIGH ? LOW : HIGH;
    decSouth = decSouth == HIGH ? LOW : HIGH;

    raForward = LOW;
    raBackward = HIGH;
  }

  // motors pin speed
  pinMode(pinMotorFast, OUTPUT);
  digitalWrite(pinMotorFast, HIGH);

  // RA pin DIR
  pinMode(pinRaDir, OUTPUT);
  digitalWrite(pinRaDir, raForward);
  /*
  if (currentHemisphere == 's') {
    digitalWrite(pinRaDir, raBackward);
  }
  */

  // RA pin STP
  pinMode(pinRaStp, OUTPUT);
  digitalWrite(pinRaStp, LOW);

  // DEC pin DIR
  pinMode(pinDecDir, OUTPUT);
  digitalWrite(pinDecDir, decNorth);

  // DEC pin STP
  pinMode(pinDecStp, OUTPUT);
  digitalWrite(pinDecStp, LOW);

  //irrecv.enableIRIn();

  // frequency = 10Khz = 10000Hz, period = 1/10000s = 1/10ms
  //timer_init_ISR_10KHz(TIMER_DEFAULT);

  // frequency = 100Khz = 100000Hz, period = 1/100000s = 1/100ms
  // frequency = 50Khz = 50000Hz, period = 1/50000s = 1/50ms
  timer_init_ISR_10KHz(TIMER_DEFAULT);

  informer.send("SYSTEM_STARTED#");

}

// 10000 times per second
// 50000 times per second
void timer_handle_interrupts(int timer) {

  if (lastSkyMoveAgo >= followFreq) {
    currentRaCoords++;
    if (gotoRaSteps > 0) {
      gotoRaSteps += gotoRaDir == raForward ? 1 : -1; // @TODO whats wrong in here ???
    }
    //gotoRaSteps += gotoRaDir == raForward ? 1 : -1; // @TODO whats wrong in here ???
    lastSkyMoveAgo = 0;
  } else {
    lastSkyMoveAgo++;
  }

  raCurrentDir = (PIND & _BV(PD4)) > 0 ? raForward : raBackward;
  //decCurrentDir = (PIND & _BV(PD6)) > 0 ? decNorth : decSouth;

  if (currentDecWard == 'e') {
    decCurrentDir = (PIND & _BV(PD6)) > 0 ? decNorth : decSouth;
  } else {
    decCurrentDir = (PIND & _BV(PD6)) > 0 ? decSouth : decNorth;
  }

  if (currentHemisphere == 's') {
    decCurrentDir == decNorth ? decSouth : decNorth;
  }

  if (follow && !slewRa && !gotoRaEnabled) {
    if (lastRaFollowAgo >= followFreq) {
      PORTD |= _BV(PD5);
      PORTD &= ~_BV(PD5);
      if (currentHemisphere == 'n') {
        if (raCurrentDir == raForward) {
          currentRaCoords--;
        } else {
          currentRaCoords++;
        }
      } else {
        if (raCurrentDir == raForward) {
          currentRaCoords++;
        } else {
          currentRaCoords--;
        }
      }
      
      lastRaFollowAgo = 0;
    } else {
      lastRaFollowAgo++;
    }
  }
  
  if (slewRa) {
    if (lastRaStepUpAgo >= raFreq) {
      PORTD |= _BV(PD5);
      PORTD &= ~_BV(PD5);
      if (currentHemisphere == 'n') {
        if (raCurrentDir == raForward) {
          currentRaCoords--;
        } else {
          currentRaCoords++;
        }
      } else {
        if (raCurrentDir == raForward) {
          currentRaCoords++;
        } else {
          currentRaCoords--;
        }
      }
      /*
      if (raCurrentDir == raForward) {
        currentRaCoords--;
      } else {
        currentRaCoords++;
      }
      */
      lastRaStepUpAgo = 0;
    } else {
      lastRaStepUpAgo++;
    }
  }

  if (slewDec) {
    if (lastDecStepUpAgo >= decFreq) {
      PORTD |= _BV(PD7);
      PORTD &= ~_BV(PD7);

      // ?
      if (currentHemisphere == 'n') {
        if (currentDecWard == 'e') {
          if (decCurrentDir == decNorth) {
            currentDecCoords++; // @TODO RIGHT_LEFT
          } else {
            currentDecCoords--; // @TODO RIGHT_LEFT
          }
        } else {
          if (decCurrentDir == decNorth) {
            currentDecCoords--; // @TODO RIGHT_LEFT
          } else {
            currentDecCoords++; // @TODO RIGHT_LEFT
          }
        }
      } else {
        if (currentDecWard == 'e') {
          if (decCurrentDir == decNorth) {
            currentDecCoords--; // @TODO RIGHT_LEFT
          } else {
            currentDecCoords++; // @TODO RIGHT_LEFT
          }
        } else {
          if (decCurrentDir == decNorth) {
            currentDecCoords++; // @TODO RIGHT_LEFT
          } else {
            currentDecCoords--; // @TODO RIGHT_LEFT
          }
        }
      }

      lastDecStepUpAgo = 0;
    } else {
      lastDecStepUpAgo++;
    }
  }

  if (gotoEnabled) {
    
    if (lastRaStepUpAgo >= gotoRaFreq && gotoRaSteps > 0) {
      PORTD |= _BV(PD5);
      PORTD &= ~_BV(PD5);
      if (gotoRaDir == raForward) {
        currentRaCoords--;
      } else {
        currentRaCoords++;
      }
      gotoRaSteps--;
      lastRaStepUpAgo = 0;
    } else {
      lastRaStepUpAgo++;
    }

    if (lastDecStepUpAgo >= gotoDecFreq && gotoDecSteps > 0) {
      PORTD |= _BV(PD7);
      PORTD &= ~_BV(PD7);

      if (currentDecWard == 'e') {
        if (gotoDecDir == decNorth) {
          currentDecCoords++; // @TODO RIGHT_LEFT
        } else {
          currentDecCoords--; // @TODO RIGHT_LEFT
        }
      } else {
        if (gotoDecDir == decNorth) {
          currentDecCoords--; // @TODO RIGHT_LEFT
        } else {
          currentDecCoords++; // @TODO RIGHT_LEFT
        }
      }
      
      gotoDecSteps--;
      lastDecStepUpAgo = 0;
    } else {
      lastDecStepUpAgo++;
    }

    if (gotoRaSteps <= 0) {
      gotoRaEnabled = false;
      PORTD |= _BV(PD4); // setting direction for follow by default
      /*
      if (currentHemisphere == 'n') {
        PORTD |= _BV(PD4); // setting direction for follow by default
      } else {
        PORTD &= ~_BV(PD4);
      }
      */
    }

    if (gotoDecSteps <= 0) {
      gotoDecEnabled = false;
    }

    if (!gotoRaEnabled && !gotoDecEnabled) {
      gotoEnabled = false;
    }
    
  }
  
  if (currentRaCoords > microstepsInRa) {
    currentRaCoords -= microstepsInRa;
  } else if (currentRaCoords < 0) {
    currentRaCoords += microstepsInRa;
  }
  
}

void loop() {
  
  commander.readCommandLineFromSerial();

  String commands = commander.getCommandsInLine();

  int totalCommands = commander.countCommandsInLine();

  if (commands.length() > 0) {
    //informer.logLn("Total commands: " + String(totalCommands));
  }

  // for each command
  for (int i = 1; i <= totalCommands; i++) {
    // get command by its index
    Command command = commander.getCommandAtPosition(i);
    
      if (command.getName() == "SLEW_RA") {
        //informer.logLn("executing slew_ra");
        gotoEnabled = false;
        gotoRaEnabled = false;
        gotoDecEnabled = false;
        int newSlewRa = command.getValue().toInt();
        if (slewRa != newSlewRa) {
          slewRa = newSlewRa;
          if (currentHemisphere == 'n') {
            if (slewRa > 0) {
              PORTD &= ~_BV(PD4);
            } else {
              PORTD |= _BV(PD4);
            }
          } else {
            if (slewRa > 0) {
              PORTD |= _BV(PD4);
            } else {
              PORTD &= ~_BV(PD4);
            }
          }
          /*
          if (slewRa > 0) {
            PORTD &= ~_BV(PD4);
          } else {
            PORTD |= _BV(PD4);
          }
          */
          informer.logLn("slewRa: " + String(slewRa));
        }
      }

      if (command.getName() == "SLEW_RA_FREQ") {
        //informer.logLn("executing slew_ra_freq");
        raFreq = command.getValue().toInt();
        informer.logLn("raFreq: " + String(slewRa));
      }

      if (command.getName() == "SLEW_DEC") {
        //informer.logLn("executing slew_dec");
        gotoEnabled = false;
        gotoRaEnabled = false;
        gotoDecEnabled = false;
        int newSlewDec = command.getValue().toInt();
        if (slewDec != newSlewDec) {
          slewDec = newSlewDec;

          if (currentDecWard == 'e') {
            if (slewDec > 0) {
              PORTD &= ~_BV(PD6);
            } else {
              PORTD |= _BV(PD6);
            }
          } else {
            if (slewDec > 0) {
              PORTD |= _BV(PD6);
            } else {
              PORTD &= ~_BV(PD6);
            }
          }

          /*
          if (currentHemisphere == 'n') {
            if (currentDecWard == 'e') {
              if (slewDec > 0) {
                PORTD &= ~_BV(PD6);
              } else {
                PORTD |= _BV(PD6);
              }
            } else {
              if (slewDec > 0) {
                PORTD |= _BV(PD6);
              } else {
                PORTD &= ~_BV(PD6);
              }
            }
          } else {
            if (currentDecWard == 'e') {
              if (slewDec > 0) {
                PORTD |= _BV(PD6);
              } else {
                PORTD &= ~_BV(PD6);
              }
            } else {
              if (slewDec > 0) {
                PORTD &= ~_BV(PD6);
              } else {
                PORTD |= _BV(PD6);
              }
            }
          }
          */
          
          informer.logLn("slewDec: " + String(slewDec));
        }
      }

      if (command.getName() == "SLEW_DEC_FREQ") {
        //informer.logLn("executing slew_dec_freq");
        decFreq = command.getValue().toInt();
        informer.logLn("decFreq: " + String(slewDec));
      }

      if (command.getName() == "GET_DEC_WARD") {
        informer.send("DEC_WARD=" + String(currentDecWard));
      }

      if (command.getName() == "SET_DEC_WARD") {
        //informer.logLn("executing SET_DEC_WARD ...");
        char newDecWard = command.getValue() == "e" ? 'e' : 'w';
        currentDecWard = newDecWard;
        informer.logLn("currentDecWard: " + String(currentDecWard));
      }

      if (command.getName() == "FOLLOW") {
        //informer.logLn("executing follow ...");
        bool toFollow = command.getValue().toInt() == 0 ? false : true;
        if (toFollow) {
          PORTD |= _BV(PD4);
          if (currentHemisphere == 's') {
            PORTD &= ~_BV(PD4);
          }
        }
        follow = toFollow;
        informer.logLn("follow: " + String(follow));
      }

      if (command.getName() == "GET_FOLLOW") {
        informer.send("FOLLOW=" + String(follow ? 1 : 0));
      }

      if (command.getName() == "GET_SLEW") {
        informer.send("SLEW=" + String(slewRa || slewDec ? 1 : 0));
      }

      if (command.getName() == "SET_RA_COORDS_FROM_STRING") {
        //informer.logLn("executing set_ra_coords_from_string");
        gotoEnabled = false;
        gotoRaEnabled = false;
        gotoDecEnabled = false;
        currentRaCoords = coordinator.raCoordsToStepsFromString(command.getValue());
      }

      if (command.getName() == "SET_RA_COORDS") {
        //informer.logLn("executing set_ra_coords");
        gotoEnabled = false;
        gotoRaEnabled = false;
        gotoDecEnabled = false;
        currentRaCoords = command.getValue().toInt();
      }

      if (command.getName() == "GOTO_RA_COORDS") {
        //informer.logLn("executing goto_ra_coords");
        slewRa = 0;
        
        gotoRaCoords = command.getValue().toInt();

        gotoRaCoordsProc(gotoRaCoords);
      }

      if (command.getName() == "GOTO_RA_COORDS_FROM_STRING") {
        //informer.logLn("executing goto_ra_coords_from_string");
        slewRa = 0;
        
        gotoRaCoords = coordinator.raCoordsToStepsFromString(command.getValue());

        gotoRaCoordsProc(gotoRaCoords);
      }

      if (command.getName() == "GET_RA_COORDS_AS_STRING") {
        String raCoordsAsString = coordinator.raCoordsToStringFromSteps(currentRaCoords);
        informer.send("RA_COORDS_AS_STRING=" + raCoordsAsString);
      }

      if (command.getName() == "SET_DEC_COORDS_FROM_STRING") {
        //informer.logLn("executing set_dec_coords_from_string");
        gotoEnabled = false;
        gotoRaEnabled = false;
        gotoDecEnabled = false;
        currentDecCoords = coordinator.decCoordsToStepsFromString(command.getValue());
      }

      if (command.getName() == "SET_DEC_COORDS") {
        //informer.logLn("executing set_dec_coords");
        gotoEnabled = false;
        gotoRaEnabled = false;
        gotoDecEnabled = false;
        currentDecCoords = command.getValue().toInt();
      }

      if (command.getName() == "GOTO_DEC_COORDS") {
        //informer.logLn("executing goto_dec_coords");
        slewDec = 0;
        
        gotoDecCoords = command.getValue().toInt();

        gotoDecCoordsProc(gotoDecCoords);
      }

      if (command.getName() == "GOTO_DEC_COORDS_FROM_STRING") {
        //informer.logLn("executing goto_dec_coords_from_string");
        slewDec = 0;
        
        gotoDecCoords = coordinator.decCoordsToStepsFromString(command.getValue());

        gotoDecCoordsProc(gotoDecCoords);
      }

      if (command.getName() == "GET_DEC_COORDS_AS_STRING") {
        String decCoordsAsString = coordinator.decCoordsToStringFromSteps(currentDecCoords);
        informer.send("DEC_COORDS_AS_STRING=" + decCoordsAsString);
      }
    
  }

  // clear the line with commands from the previous loop
  commander.clearCommands();

  //informer.logLn("currentRaCoords: " + String(currentRaCoords) + " ; " + "currentDecCoords: " + String(currentDecCoords)); 

  /*
  if (irrecv.decode(&irrecvResults)) {
    resultsValue = irrecvResults.value;
    informer.logLn("IR: resultsValue: " + String(resultsValue, HEX));
    irrecv.resume();
  }
  */
}

void gotoRaCoordsProc(long gotoRaCoords)
{
    // forward:
  long stepsForward = currentRaCoords + microstepsInRa - gotoRaCoords;
  stepsForward = stepsForward >= microstepsInRa ? stepsForward - microstepsInRa : stepsForward;
  // backward:
  long stepsBackward = microstepsInRa - currentRaCoords + gotoRaCoords;
  stepsBackward = stepsBackward >= microstepsInRa ? stepsBackward - microstepsInRa : stepsBackward;
  
  gotoRaSteps = stepsForward <= stepsBackward ? stepsForward : stepsBackward;
  gotoRaDir = stepsForward <= stepsBackward ? raForward : raBackward;
  //gotoRaDir = stepsForward <= stepsBackward ? raBackward : raForward;
  
  if (gotoRaDir == raBackward) {
    PORTD &= ~_BV(PD4);
    if (currentHemisphere == 's') {
      PORTD |= _BV(PD4);
    }
  } else {
    PORTD |= _BV(PD4);
    if (currentHemisphere == 's') {
      PORTD &= ~_BV(PD4);
    }
  }
  
  gotoEnabled = true;
  gotoRaEnabled = true;
}

void gotoDecCoordsProc(long gotoDecCoords)
{
  long newGotoDecSteps = gotoDecCoords - currentDecCoords;
  
  // @TODO RIGHT_LEFT

  /*
  if (currentHemisphere == 'n') {
    if (currentDecWard == 'e') {
      if (newGotoDecSteps < 0) {
        gotoDecDir = decSouth;
        PORTD |= _BV(PD6);
      } else {
        gotoDecDir = decNorth;
        PORTD &= ~_BV(PD6);
      }
    } else {
      if (newGotoDecSteps < 0) {
        gotoDecDir = decNorth;
        PORTD &= ~_BV(PD6);
      } else {
        gotoDecDir = decSouth;
        PORTD |= _BV(PD6);
      }
    }
  } else {
    if (currentDecWard == 'e') {
      if (newGotoDecSteps < 0) {
        gotoDecDir = decSouth;
        PORTD &= ~_BV(PD6);
      } else {
        gotoDecDir = decNorth;
        PORTD |= _BV(PD6);
      }
    } else {
      if (newGotoDecSteps < 0) {
        gotoDecDir = decNorth;
        PORTD |= _BV(PD6);
      } else {
        gotoDecDir = decSouth;
        PORTD &= ~_BV(PD6);
      }
    }
  }
  */

  if (currentDecWard == 'e') {
    if (newGotoDecSteps < 0) {
      gotoDecDir = decSouth;
      PORTD |= _BV(PD6);
      //PORTD &= ~_BV(PD6);
    } else {
      gotoDecDir = decNorth;
      PORTD &= ~_BV(PD6);
      //PORTD |= _BV(PD6);
    }
  } else {
    if (newGotoDecSteps < 0) {
      gotoDecDir = decNorth;
      //PORTD &= ~_BV(PD6);
      PORTD |= _BV(PD6);
    } else {
      gotoDecDir = decSouth;
      //PORTD |= _BV(PD6);
      PORTD &= ~_BV(PD6);
    }
  }

  /*
  if (newGotoDecSteps < 0) {
    gotoDecDir = decSouth;
    PORTD |= _BV(PD6);
  } else {
    gotoDecDir = decNorth;
    PORTD &= ~_BV(PD6);
  }
  */
  
  gotoDecSteps = abs(newGotoDecSteps);
  
  gotoEnabled = true;
  gotoDecEnabled = true;
}
