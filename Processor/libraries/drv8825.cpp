#include "drv8825.h"
#include "Arduino.h"
//#define DEBUG "motordriver"
//#include "h_logger.h"

//Define Stepper Driver Pins
#define nSleep 8
#define nReset 9
//#define decay 51
//#define nFault 50
#define nEnbl 6
#define nDir 7
//#define mode0 48
#define nStep 5
//#define mode2 47
//#define mode1 46
#define nHome 4
#define drvVref DAC0

uint8_t stepmode = 7;
static uint16_t delayTime = 800;
drv8825::drv8825() {}

void drv8825::init() {
//  LOG("Initializing" );
  analogWriteResolution(12);
  pinMode(nSleep, OUTPUT);
  pinMode(nReset, OUTPUT);
  //pinMode(decay, OUTPUT);
  pinMode(nEnbl, OUTPUT);
  pinMode(nDir, OUTPUT);
 // pinMode(mode0, OUTPUT);
  pinMode(nStep, OUTPUT);
 // pinMode(mode2, OUTPUT);
 // pinMode(mode1, OUTPUT);

  pinMode(drvVref, OUTPUT);
  //pinMode(nFault, INPUT);
  pinMode(nHome, INPUT);

  analogWrite(drvVref, 600); //Set Current to 2A
  digitalWrite(nSleep, LOW); //enable drv8825driver
  digitalWrite(nReset, HIGH); //disables reset
  //digitalWrite(decay, LOW); //slow decay
  digitalWrite(nEnbl, LOW); //enable indexer

  //digitalWrite(mode0, LOW); //Full step
 // digitalWrite(mode1, LOW); //Full step
 // digitalWrite(mode2, LOW); //Full step
  digitalWrite(nDir, HIGH); //Motor direction forward
}
void drv8825::mStep(uint16_t steps) {
//  LOG("Stepping " + String(steps));
  for (int a = 0; a < stepmode; a++) {
    for (int i = 0; i < steps; i++) {
      digitalWrite(nStep, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(nStep, LOW);
      delayMicroseconds((delayTime));
     
    }
  }
}
void drv8825::setDirection(uint8_t direct) {
  digitalWrite(nDir, direct); //which direction to go
}
void drv8825::powerMode(uint8_t state) { //Sets the state of the drv8825
//  LOG("Setting Motor Power to " + String(state));
  digitalWrite(nSleep, state);
  digitalWrite(nReset, state);
}
/*
void setMotorCurrent(uint16_t milliAmps) {
  //Current is calculated by I=Vxref/(5*Risense) -> Vxref = 5* I * Risense, Risense is 0.2R
  uint16_t motorVoltage = 5 * milliAmps * 0.2;
  //The due Output is 0.55 to 2.75V
  // 3300/4095*B=A -> (4095*Voltage)/3300=adcValue
  uint16_t writeadcValue = 4095 * motorVoltage / 3300;
//  LOG("Setting Motor Current to " + String(milliAmps) + "mA");
  analogWrite(drvVref, writeadcValue);
}

*/
void drv8825::stepMode(uint8_t mode) {
//  LOG("Setting Motor stepMode to " + mode);
  stepmode = mode;
  switch (mode) {
    case 1:
    //  digitalWrite(mode0, 0);
   //   digitalWrite(mode1, 0);
   //   digitalWrite(mode2, 0);
      delayTime = 400;
      break;
    case 2:
   //   digitalWrite(mode0, 1);
   //   digitalWrite(mode1, 0);
   //   digitalWrite(mode2, 0);
      delayTime = 150;
      break;
    case 3:
    //  digitalWrite(mode0, 0);
    //  digitalWrite(mode1, 1);
   //   digitalWrite(mode2, 0);
      delayTime = 90;
      break;
    case 4:
    //  digitalWrite(mode0, 1);
   //   digitalWrite(mode1, 1);
   //   digitalWrite(mode2, 0);
      delayTime = 90;
      break;
    case 5:
   //   digitalWrite(mode0, 0);
    //  digitalWrite(mode1, 0);
   //   digitalWrite(mode2, 1);
      delayTime = 40;
      break;
    case 6:
    //  digitalWrite(mode0, 1);
    //  digitalWrite(mode1, 0);
    //  digitalWrite(mode2, 1);
      delayTime = 30;
      break;
    case 7:
   //   digitalWrite(mode0, 1);
   //   digitalWrite(mode1, 1);
   //   digitalWrite(mode2, 1);
      delayTime = 15;
      break;
  }
}



uint8_t getPosition() {
  return 1;
}


