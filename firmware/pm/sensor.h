#include <ArduinoJson.h>

class Wire;
class Serial;
class SPI;

/* abstract */ class Sensor {
  public:
    // Factory method: Returns nullptr or a sensor of the selected type
    static Sensor *getSensor(String sensorType);

    // Abstract function to call within setup().
    // This will set up whatever port is necessary, and bring the device out of sleep mode.
    // 
    virtual void begin() = 0;

    // Add others as necessary - the assumption is we'll set the appropriate one when instantiated.
    virtual void setPins(uint8_t pin1, ...) { }
    virtual void setI2C(Wire& i2c) { }
    virtual void setSPI(SPI& spi) { }
    virtual void setSerial(Serial& serial) { }
}
