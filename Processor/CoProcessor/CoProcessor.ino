#include "HSS.h"
#include "SPI.h"
#include <Wire.h> //Include I2C Library

#define i2CAdress 0x22    // I2C Adress
#define SOFTVERSION 042   //Software Version
#define HARDVERSION 002   //Hardware Version
#define ID "002"     //Serial Number
#define DEBUG 0

byte myArray[2];

//********** Setup **********//
#define lowPressMin -1034    //  -1034mBar (-15PSI)
#define lowPressMax 1034     //  1034mBar (15PSI)
#define sampleRate 1 // Analog Measurement Oversampling Rate

//********** END Setup **********//

#define modulePressurePin A1
#define highPressurePin A2
#define midPressurePin A3 //

#define analogOffset 0
#define inputVoltageReference 5020 //Input Voltage Reference used for calculating values
#define batteryVoltagePin A0


HSS PressCh1 = HSS(3, lowPressMin, lowPressMax); //create HSS object for each Sensor
HSS PressCh2 = HSS(4, lowPressMin, lowPressMax);
HSS PressCh3 = HSS(8, lowPressMin, lowPressMax);
HSS PressCh4 = HSS(9, lowPressMin, lowPressMax);
HSS PressCh5 = HSS(10, lowPressMin, lowPressMax);
uint16_t batterySensorVal = 0;
uint16_t modulePressure = 0;
uint16_t batteryVoltage = 0;
const long sensorRefreshInterval = 50;  //
unsigned long previousSensorRefreshMillis = 0;

const long debugOutInterval = 1000;  //
unsigned long previousdebugOutIntervalMillis = 0;

uint16_t midPressure = 0;
uint16_t highPressure = 0;

volatile byte myRegisterAddress = 0; //I2C request Byte
void setup() {

  if (DEBUG) Serial.begin(9600);
  Wire.begin(i2CAdress); //joining I2C Bus as Master
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

}
void loop()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousSensorRefreshMillis >= sensorRefreshInterval) {

    previousSensorRefreshMillis = currentMillis;


    //read HSS Sensor
    PressCh1.readSensor();
    PressCh2.readSensor();
    PressCh3.readSensor();
    PressCh4.readSensor();
    PressCh5.readSensor();

    readModulePressure(); //read Module Pressure
    getMidPressure();
    getHighPressure();
    getBatteryVoltage();
  }
}
//********** read Module Pressure **********//
uint16_t readModulePressure() {
  int16_t values;
  for (int i = 0; i < sampleRate; i++) {
    //delay(1);
    values += (analogRead(modulePressurePin));
  }
  uint16_t sensorValue = (values / sampleRate) + analogOffset; //Digital Voltage
  int U = (inputVoltageReference / 1023.0) * sensorValue; //Voltage on Chip Pin
  modulePressure = ((1 / 318.0) * ((U * 100000 / inputVoltageReference ) + 353)) * 10; //External Voltage calculated through Voltage divider

  return modulePressure;
}


uint16_t getMidPressure() {
  int16_t values;
  for (int i = 0; i < sampleRate; i++) {
    //delay(1);
    values += (analogRead(midPressurePin));
  }
  uint16_t sensorValue = (values / sampleRate) + analogOffset; //Digital Voltage


  midPressure = float(sensorValue) * 0.0111589 * 1000;

  return midPressure;

}
uint32_t getHighPressure() {

  int16_t values;
  for (int i = 0; i < sampleRate; i++) {
    //delay(1);
    values += (analogRead(highPressurePin));
  }
  uint16_t sensorValue = (values / sampleRate) + analogOffset; //Digital Voltage

  //highPressure = float(sensorValue) * 0.223177;
  highPressure = ((float(sensorValue)*200)/896)*10;
  
  return highPressure;
}
//********** read Battery Voltage **********//
uint16_t getBatteryVoltage() {
/*
  uint16_t values;
  for (int i = 0; i < sampleRate; i++) {
    //delay(1);
    values += (analogRead(batteryVoltagePin));
  }
  uint16_t sensorValue = (values / sampleRate) + analogOffset; //Digital Voltage
  */
  batterySensorVal = analogRead(batteryVoltagePin);
  //int U = sensorValue * (inputVoltageReference / 1023.0); //Voltage on Chip Pin
 //batteryVoltage = (sensorValue * 8161 / 1001)*100;
  //batteryVoltage =  (U *163) * 163;

  return batterySensorVal;
}


//********** I2C functions **********//
void receiveEvent(int howMany) {
  myRegisterAddress = Wire.read();    // set the index (or pointer) to the registers.
}

void requestEvent()
{

  switch (myRegisterAddress) {
    case 1: {
        //does nothing
        break;
      }
    case 2: { //Sends the Device Hardware ID (Serial)
        Wire.write(ID);
        break;
      }
    case 3: { //Sends the Device Hardware Version
        Wire.write(HARDVERSION);
        break;
      }
    case 4: { //Sends the Device Software Version
        Wire.write(SOFTVERSION);
        break;
      }
    case 6: { // Sends the Module Presssure

        I2cBigNum(modulePressure);
        break;
      }
    case 7: {
        I2cBigNum(PressCh1.getPressure());
        break;
      }
    case 8: {
        I2cBigNum(PressCh2.getPressure());
        break;
      }
    case 9: {
        I2cBigNum(PressCh3.getPressure());
        break;
      }
    case 10: {
        I2cBigNum(PressCh4.getPressure());
        break;
      }
    case 11: {
        I2cBigNum(PressCh5.getPressure());
        break;
      }
    case 12: {
        I2cBigNum(highPressure);
        break;
      }
    case 13: {
        I2cBigNum(midPressure);
        break;
      }
    case 14: {
       I2cBigNum(batterySensorVal);
       //I2cBigNum(512);
        break;
      }
  }
}

void I2cBigNum(int16_t bigNum) {
  myArray[0] = (bigNum >> 8) & 0xFF;
  myArray[1] = bigNum & 0xFF;
  Wire.write(myArray, 2);
}

