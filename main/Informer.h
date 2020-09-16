/**
 * Class representing serial port to send messages to serial port
 */
class Informer
{
  public:

    /**
     * Send info message to serial port
     */
    void sendInfo(Info info)
    {
      Serial.println(String(info.getSerializedData()) + "#");
    }

    /**
     * Creates and sends log message
     */
    void logLn(String line)
    {
      this->sendInfo(Info("LOG: " + line));
    }

    /**
     * Creates and sends log message
     */
    void send(String line)
    {
      this->sendInfo(Info(line));
    }
};
