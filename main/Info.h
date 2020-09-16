/**
 * Class representing data object of info message
 * tgo be sent to serial port
 */
class Info
{
  private:

    // serialized data, ready to be sent to serial
    String serializedData = "";

    // name of the info message (a part before the '=')
    String name = "";

    // value of the info message (a part after the '=')
    String value = "";

    /**
     * Serialize the data like name=value
     */
    void serializeData(String name, String value) {
      this->serializedData = name;
      if (value.length() > 0) {
        this->serializedData += "=" + value;
      }
    }

  public:

    /**
     * Constructor
     */
    Info (String name, String value)
    {
        this->name = name;
        this->value = value;

        this->serializeData(this->name, this->value);
    }

    /**
     * Constructor
     */
    Info (String name)
    {
        this->name = name;
        this->value = "";

        this->serializeData(this->name, this->value);
    }

    /**
     * Return serialized message
     */
    String getSerializedData()
    {
      return this->serializedData;
    }
};
