#include "Arduino.h"
class logger


{

  public:
    logger();
    void init();
    void LOG(String logString);
    void ERR(String logString);
    void DATA(String logString);

};
