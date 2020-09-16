/**
 * This class is data object class that represents
 * a single command from serial port
 */
class Command
{
  private:

    // raw command, as it was detected
    String raw = "";

    // command name (part before '=')
    String name = "";

    // command value (part after '='), may be absent
    String value = "";

  public:

    /*
     * Constructor
     */
    Command(String raw)
    {
      this->raw = raw;
      
      bool writeToValue = false;
      
      for (int i = 0; i < raw.length(); i++) {
        char curChar = raw.charAt(i);
        if (curChar == '=') {
          writeToValue = true;
          continue;
        }

        if (writeToValue) {
          this->value += String(curChar);
          continue;
        }

        this->name += String(curChar);
      }
    }

    /**
     * return raw command
     */
    String getRaw()
    {
      return this->raw;
    }

    /**
     * return command name
     */
    String getName()
    {
      return this->name;
    }

    /**
     * return command value
     */
    String getValue()
    {
      return this->value;
    }
};
