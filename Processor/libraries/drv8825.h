#include "Arduino.h"
class drv8825
{

  public:
    drv8825();
    void mStep(uint16_t steps);
    void setDirection(uint8_t direct);
    void init();
    void powerMode(uint8_t state);
    void stepMode(uint8_t mode);
   // void setMotorCurrent(uint16_t milliAmps);
    uint8_t getPosition();

};
