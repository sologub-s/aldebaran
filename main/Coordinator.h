/**
 * This class is responsible for
 * converting coordinates
 */
class Coordinator
{
  protected:

    Informer informer;

    long microstepsInRa = 0;

    long stepsPerRaSecond = 0;

    long microstepsInDec = 0;

    double stepsPerDecSecond = 0;

    void init()
    {
      this->stepsPerRaSecond = round(this->microstepsInRa / 86400);
      this->stepsPerDecSecond = double(this->microstepsInDec) / 1296000.0;
      //this->informer.logLn("stepsPerDecSecond: " + String(this->stepsPerDecSecond));
      //this->informer.logLn("stepsPerRaSecond: " + String(this->stepsPerRaSecond));
    }
  
  public:

    Coordinator(Informer informer, long microstepsInRa, long microstepsInDec)
    {
      this->informer = informer;
      this->microstepsInRa = microstepsInRa;
      this->microstepsInDec = microstepsInDec;

      this->init();
    }

    long raCoordsToStepsFromString(String raCoordsString)
    {
      int hours;
      int minutes;
      double seconds;

      String currentValue = "";
      String currentPosition = "hours";

      raCoordsString += ":";

      for (int i = 0; i < raCoordsString.length(); i++) {
        char curChar = raCoordsString.charAt(i);
        if (curChar == ':') {

          if (currentPosition == "hours") {
            hours = currentValue.toInt();
            currentValue = "";
            currentPosition = "minutes";
          } else if (currentPosition == "minutes") {
            minutes = currentValue.toInt();
            currentValue = "";
            currentPosition = "seconds";
          } else {
            seconds = currentValue.toDouble();
            currentValue = "";
            currentPosition = "none";
          }
          
          continue;
        }

        
        currentValue += curChar;
      }

      long positionInSteps = 0;

      positionInSteps += this->stepsPerRaSecond * 3600 * hours;

      positionInSteps += this->stepsPerRaSecond * 60 * minutes;

      positionInSteps += round(this->stepsPerRaSecond * seconds);

      //this->informer.logLn("RA hours: " + String(hours));
      //this->informer.logLn("RA minutes: " + String(minutes));
      //this->informer.logLn("RA seconds: " + String(seconds));
      //this->informer.logLn("RA positionInSteps: " + String(positionInSteps));
      
      return positionInSteps;
    }

    String raCoordsToStringFromSteps(long raCoords)
    {
      long totalSeconds = round(raCoords / this->stepsPerRaSecond);
      
      int hours = totalSeconds / 3600;
      totalSeconds = totalSeconds % 3600;
      int minutes = totalSeconds / 60;
      totalSeconds = totalSeconds % 60;
      int seconds = totalSeconds;
      
      return String("") + (hours < 10 ? "0" : "") + String(hours) + ":" + (minutes < 10 ? "0" : "") + String(minutes) + ":" + (seconds < 10 ? "0" : "") + String(seconds);
    }

    long decCoordsToStepsFromString(String decCoordsString)
    {
      int degrees;
      int minutes;
      double seconds;

      String currentValue = "";
      String currentPosition = "degrees";

      //this->informer.logLn("decCoordsString: " + decCoordsString);

      for (int i = 0; i < decCoordsString.length(); i++) {
        char curChar = decCoordsString.charAt(i);

        //this->informer.logLn("curChar: " + String(curChar));

        if (currentPosition == "degrees") {
          if (curChar == 'd') {
            degrees = currentValue.toInt();
            //this->informer.logLn("degrees currentValue: " + currentValue);
            currentValue = "";
            currentPosition = "minutes";
            continue;
          }
        }

        if (currentPosition == "minutes") {
          if (curChar == 'm') {
            minutes = currentValue.toInt();
            //this->informer.logLn("minutes currentValue: " + currentValue);
            currentValue = "";
            currentPosition = "seconds";
            continue;
          }
        }

        if (currentPosition == "seconds") {
          if (curChar == 's') {
            //this->informer.logLn("seconds currentValue: " + currentValue);
            seconds = currentValue.toInt();
            currentValue = "";
            currentPosition = "none";
            continue;
          }
        }

        if (currentPosition != "none") {
          currentValue += curChar;
        }
      }

      long positionInSteps = 0;

      positionInSteps += round(this->stepsPerDecSecond * 3600.0 * double((degrees + 90))); // @TODO RIGHT_LEFT
  
      positionInSteps += round(this->stepsPerDecSecond * 60.0 * double(minutes));
  
      positionInSteps += round(this->stepsPerDecSecond * double(seconds));
  
      //this->informer.logLn("DEC degrees: " + String(degrees));
      //this->informer.logLn("DEC minutes: " + String(minutes));
      //this->informer.logLn("DEC seconds: " + String(seconds));
      //this->informer.logLn("DEC positionInSteps: " + String(positionInSteps));
      
      return positionInSteps;
    }

    String decCoordsToStringFromSteps(long decCoords)
    {
      long totalSeconds = round(decCoords / this->stepsPerDecSecond);
      
      int degrees = totalSeconds / 3600 - 90;
      totalSeconds = totalSeconds % 3600;
      int minutes = totalSeconds / 60;
      totalSeconds = totalSeconds % 60;
      int seconds = totalSeconds;
      
      return String("") + String(degrees) + "d" + String(minutes) + "m" + String(seconds) + "s";
    }
};
