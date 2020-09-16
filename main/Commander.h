/**
 * This class is responsible for
 * getting incoming commands from
 * serial port and other sources
 */
class Commander
{
  private:

    /**
     * Buffer to keep commands
     */
    String commandsInLine = "";

  public:

    /**
     * Clears commands buffer
     */
    void clearCommands()
    {
      this->commandsInLine = "";
    }

    void appendToCommandsLine(String commandString)
    {
      if (this->commandsInLine.length() == 0) {
        this->commandsInLine += commandString;
        return;
      }

      if (this->commandsInLine.charAt(this->commandsInLine.length() - 1) != ';') {
        this->commandsInLine += ";";
      }

      this->commandsInLine += commandString;
      
    }

    /**
     * reads commands from serial port
     */
    void readCommandLineFromSerial()
    {
      if (Serial.available() == 0) {
        return;
      }

      String commands = Serial.readStringUntil('#');
      
      commands.trim();
      this->commandsInLine += commands;;
    }

    /**
     * get commands buffer
     */
    String getCommandsInLine()
    {
      return this->commandsInLine;
    }

    /**
     * count total commands in commands buffer
     */
    int countCommandsInLine()
    {
      int totalCommands = 0;

      if (this->commandsInLine.length() == 0) {
        return 0; // if the line is empty = it's realy 0 of commands in there
      }

      totalCommands++; // otherwise, we have at least 1 command in it

      for (int i = 0; i < this->commandsInLine.length(); i++) {
        if (this->commandsInLine.charAt(i) == ';') {
          /*
           * just count delimiters, cause for non-empty line we have commands amount equal to delimiter count+1
           * i.e. in line "command1=arguments1;command2=arguments2;command3=arguments3" we have 2 delimiters, that means we have 3 commands
           */
          totalCommands++;
        }
      }

      return totalCommands;
    }

    /**
     * Return Command by its index in commands buffer
     */
    Command getCommandAtPosition(int position)
    {
      if (this->commandsInLine.length() == 0) {
        // string is empty - Command will be empty too
        return Command("");
      }

      // buffer for one single command
      String singleCommand = "";

      // current command index in buffer
      int currentPosition = 0;

      // checking every symbol in commands buffer
      for (int i = 0; i < this->commandsInLine.length(); i++) {

        // current symbol
        char curChar = this->commandsInLine.charAt(i);

        // if this is command delimiter ';' or the last symbol in the command buffer
        if (curChar == ';' || i == this->commandsInLine.length() - 1) {

          if (curChar != ';') {
            // if it's not a delimiter then we need this symbol in current command buffer
            singleCommand += curChar;
          }

          currentPosition++;

          if (currentPosition == position) {
            // the position is that was requested by this method's call
            return Command(singleCommand);
          }

          if (currentPosition < position) {
            // empty current command buffer and go to the next symbol
            singleCommand = "";
            continue;
          }
        }

        // add current symbol to current command buffer
        singleCommand += curChar;
      }

      // if we have not found the desired position then return empty command
      return Command("");
    }
};
