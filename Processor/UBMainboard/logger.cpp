#include "logger.h"
#include "Arduino.h"
#include <SD.h>
#include <SPI.h>

#define microSDSS 77
#define microSDIn 36
#define faultLED 24


logger::logger(){}

void logger::init(){
  //Initialize Status LEDs
  pinMode(faultLED, OUTPUT);
  digitalWrite(faultLED, LOW);

  //Initialize SD-Card
  //LOG("Initializing SD card...");
  if (!SD.begin(microSDSS)) {
    if (!digitalRead(microSDIn)) {
      //   ERR("Card failed, but inserted");
      // don't do anything more:
    } else {
      //    ERR("No MicroSD Card inserted");
      digitalWrite(faultLED, HIGH);
      return;
    }
  }
  LOG("card initialized.");


}
void logger::LOG(String logString) {
  File dataFile = SD.open("log.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(logString);
    dataFile.close();
  }
}
void logger::ERR(String logString) {
  File dataFile = SD.open("data.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(logString);
    dataFile.close();
  }
}
void logger::DATA(String logString) {
  File dataFile = SD.open("error.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(logString);
    dataFile.close();
  }
}

