
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "invertedServo.h"
#include "Adafruit_MCP23017.h"
#include "AccelStepper.h"
#include "Adafruit_MCP9808.h"

//********** Setup **********//

#define loglevel 1                //Info Message Log level
#define AnalogReadSampleRate 10    //Oversampling rate for analog readings
#define AnalogOffset 0            //Analog Offset hast to be calibrated for each chip
#define oBPSoversamplingRate 20   //Onboard Pressure Sensor Oversampling rate
#define pushDatainterval 500     //Interval at which data is pushed down to the Groundstation
#define sensorRefreshInterval 300 //Intervall in which Sensors are read
#define RS422 Serial1             //Serialport Used for data Pushing
#define SerialBaud 38400          //Baudrate for SerialPort
#define debounceDelay 50            //debounce time for signal Input Pins
#define stepsForDegree 18          //Steppermotor Steps for 1° Angle
#define degreesBetweenChambers 49  // Degrees between Chambers
#define stepperAccel 10000         //Steppermotor acceleration
uint32_t stepperSpeed = 5000;        //Steppermotor Speed
#define stepperMinPulseWidth 10   //Minimum Pulsewidth
#define stepperCurrent 100        //Stepper Current

//********** END Setup **********//


#define InputVoltagePin A8 //Pin for measuring the Input Voltage
#define IGNBatteryVoltagePin DAC1
//Define Pins
#define LOPIN 71
#define SOEPIN 13
#define SODSPIN 41
#define homeAPin 52
#define homeBPin 65

//Pins on the I2C to Parallel driver
#define ValveMainPin 5
#define ValveEvacPin1 6
#define ValveEvacPin2 7
#define ServoPin 3
#define CameraPowerPin 63
#define CameraRecordPin 53

#define NTC1Pin A2
#define NTC2Pin A3
#define NTC3Pin A4
#define NTC4Pin A5
#define NTC5Pin A6
#define NTC6Pin A7

//Define LEDS
#define faultLED 24
#define statusLED 23
#define rocketNoseLED 11
#define rocketMiddleLED 12
#define rocketFlameLED 13

//Defines so the device can do a self reset
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ)

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

//CoProcessor
#define COPROCESSOR_ADRESS 0x22 //0x22

//Port Expander
#define MCP23017_ADRESS 0x20
Adafruit_MCP23017 mcp;

//Define Status Variables
bool testMode = 0;
bool isArmed = 0;
bool BatteryStatus = 0;
bool SDCard = 0;
//bool TEEMPOOR = 0; //Temperature out of Range
bool ETC_ERROR = 0;
bool LO = 0;
bool SOE = 0;
bool SODS = 0;

bool LOState = 0;
bool SOEState = 0;
bool SODSState = 0;
bool LastLOState = 0;
bool LastSOEState = 0;
bool LastSODSState = 0;

long lastDebounceTimeLO = 0;
long lastDebounceTimeSOE = 0;
long lastDebounceTimeSODS = 0;
uint16_t messageID = 0;

unsigned long timer0 = 0;
unsigned long timer1 = 0;
unsigned long timer2 = 0;

//Define Temperature Variables
// 0    = Module Temp (onboard)
// 1-5  = Chamber Temp
// 6-9  = Mid Pressure / High Pressure
float temperature[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};


//Define Pressure Variables
// 0    = Module Pressure (onboard)
// 1-5  = Chamber Pressure
// 6    = High Pressure
// 7    = Mid Pressure
int16_t pressure[8] = {10, 10, 10, 10, 10, 10, 10, 10};

uint16_t inputVoltage = 0;
int16_t IGNBatteryVoltage = 1;
//Turntable and Motor devices
uint8_t sampleFacing = 0;
bool motorPowered = 0;
bool cameraPowered = 0;
bool cameraRec = 0;
bool tableLocked = 0;
bool HPOutputs[8] = {0, 0, 0, 0, 0, 0, 0, 0}; //Relays
bool switchBoardGPIO[8] = {0, 0, 0, 0, 1, 0, 0, 0};

//Define Stepper Driver Pins
#define nReset 9
#define nSleep 8
#define nDir 7
#define nEnbl 6
#define nStep 5
#define nHome 4
#define drvVref DAC0

//Status Variables
uint8_t progress = 0;
unsigned long previousMillis = 0;
int fileVersion = 0;
unsigned long previousSensorRefreshMillis = 0;
char RS422received[2];
byte RS422receivedPos = 0;

AccelStepper stepper(AccelStepper::DRIVER, nStep, nDir);

Servo servo;

#define microSDSS 77

void setup() {
  RS422.begin(SerialBaud); //initialize the Serial1 Port with a baudrate of 38.4 kbit/s


  pinMode(faultLED, OUTPUT);
  digitalWrite(faultLED, LOW);
  pinMode(statusLED, OUTPUT);
  digitalWrite(statusLED, HIGH); //Set high to signalize initialisation

  if (loglevel < 3) {
    LOGln("Starting initialization", 1);
  }

  if (!SD.begin(microSDSS)) {
    SDCard = 0;
    ERRln("Failed to recognize SD - Card", 1);
  } else {
    SDCard = 1;
    setLatestFileVersion(); //get the latest file Version
    LOGln("SD-Card succesfully initialized", 1);
    LOG("File Version set to ", 1);
    LOGln(String(fileVersion), 0);
  }
  if (loglevel < 3) LOGln("Delaying 50ms until all devices are booted up", 1);
  delay(50);
  //Initialize MCU LEDS
  if (loglevel == 1) LOGln("Initializing MCU LEDS", 1);

  //Initialize MCU LEDS
  if (loglevel == 1) LOGln("Initializing Motor Driver", 1);
  stepper.setMaxSpeed(stepperSpeed);
  stepper.setAcceleration(stepperAccel);

  if (loglevel == 1) {
    LOG("initializing Serial Port with Baudrate: ", 1);
    LOGln(String(SerialBaud), 0);
  }

  if (loglevel == 1) {
    LOGln("Setting ADC resolution to 12 bit", 1);
  }
  analogReadResolution(12); // set ADC resolution to 12 bit

  //Signal Pins
  pinMode(LOPIN, INPUT);
  pinMode(SOEPIN, INPUT);
  pinMode(SODSPIN, INPUT);

  //Ignition Pins
  for (int i = 8; i <= 12; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, LOW);
  }
  if (loglevel == 1) LOGln("joining I2C Port as Master", 1);
  Wire.begin(); //join I2C Port as Master

  //initialize Switchboard Pins
  if (loglevel == 1) {
    LOG("Initializing MCP23017 with address: ", 1);
    LOGln(String(MCP23017_ADRESS), 0);
  }


  //LEDs
  mcp.begin();
  mcp.pinMode(rocketNoseLED, OUTPUT);
  mcp.pinMode(rocketMiddleLED, OUTPUT);
  mcp.pinMode(rocketFlameLED, OUTPUT);

  //Valve
  mcp.pinMode(ValveMainPin, OUTPUT);
  mcp.pinMode(ValveEvacPin1, OUTPUT);
  mcp.pinMode(ValveEvacPin2, OUTPUT);
  mcp.pinMode(0, OUTPUT);
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(2, OUTPUT);
  mcp.pinMode(3, OUTPUT);
  mcp.pinMode(4, OUTPUT);
  switchRelais(LOW);


  if (loglevel == 1) {
    LOG("Initializing CoProcessor with address: ", 1);
    LOGln(String(COPROCESSOR_ADRESS), 0);
  }
  Wire.beginTransmission(COPROCESSOR_ADRESS);
  if (Wire.endTransmission() == 0) { //Check if the CoProcessor is available
    if (loglevel == 1) {

      LOG("Coprocessor recognized with Software Version: ", 1);
      Wire.requestFrom(COPROCESSOR_ADRESS, 1);   // request from
      LOGln(String(Wire.read()), 0);
    }
  } else {
    ERRln("Failed to recognize CoProcessor", 1);
    transmittData(99, "Failed to recognize CoProcessor");

  }

  if (!tempsensor.begin()) {
    ERRln("Couldn't find MCP9808!", 1);
    transmittData(99, "Couldn't find MCP9808!");
  }
  else {
    LOGln("MCP9808 succesfully initialized", 1);
  }



  if (loglevel == 1) {
    LOGln("Setting progress to Step: 1", 1);
  }
  progress = 1; // set progess to step 1



  servo.attach(ServoPin); //initialize Servo
  tableLock(1); //lock Table

  pinMode(drvVref, OUTPUT);
  pinMode(nHome, INPUT);
  pinMode(homeAPin, INPUT);
  pinMode(homeBPin, INPUT);
  pinMode(nSleep, OUTPUT);
  pinMode(nHome, INPUT);
  pinMode(nReset, OUTPUT);
  pinMode(nStep, OUTPUT);
  analogWrite(drvVref, 100);
  digitalWrite(nReset, HIGH);
  digitalWrite(nSleep, HIGH);
  delay(1);
  digitalWrite(nEnbl, LOW);
  delay(1);
  digitalWrite(nEnbl, HIGH);

  //Camera
  pinMode(CameraRecordPin, OUTPUT);
  pinMode(CameraPowerPin, OUTPUT);
  digitalWrite(CameraRecordPin, HIGH);
  digitalWrite(statusLED, LOW); // End of initialization
  cameraPower(LOW);
  setMotorCurrent(stepperCurrent);

}

void loop() {
  unsigned long currentMillis = millis();
  checkSignals();
  eventhandler();

  //********** Read Sensors **********//
  if (currentMillis - previousSensorRefreshMillis >= sensorRefreshInterval) {
    previousSensorRefreshMillis = currentMillis;

    readCoProcessor(); //read Coprocessor (High/Mide/Module Pressure, Temperature Sensors, Battery Voltage)
    readBoardTemp(); //read Onboard Temperature
    setNtcVal();
    //setNtcVal(); //read all NTC Sensors
    if (!SDCard) {
      digitalWrite(faultLED, !digitalRead(faultLED)); //If no SD Card detected, blink
    }
  }


  //********** Push/Log Data **********//
  if (currentMillis - previousMillis >= pushDatainterval) { //Pushing data
    previousMillis = currentMillis;
    // checkSDCard(0); //check for micro SD Card
    pushData();
    DATAln("");


  }

  //********** Read Serialport **********//

  if (RS422.available() > 0) {
    char receiveBuffer = RS422.read();
    if (receiveBuffer == '*') {
      int num;
      num = atoi( RS422received );
      //transmittData(99, num);
      RS422received[0] = 0;
      RS422received[1] = 0;
      RS422receivedPos = 0;
      inputWork(num);
    }
    else {
      RS422received[RS422receivedPos] = receiveBuffer;
      RS422receivedPos++;
    }

  }
}
//********** END MAIN LOOP **************//
void inputWork(byte input) {
  switch (input) {
    case 0: {
        switchMainValve(0);
        transmittData(99, "CMD_MV_CLOSE");
        break;
      }
    case 1: {
        switchMainValve(1);
        transmittData(99, "CMD_MV_OPEN");
        break;
      }
    case 2: {
        switchEvacValve(1);
        transmittData(99, "CMD_EV_CHAMBERS");
        break;
      }
    case 3: {
        switchEvacValve(2);
        transmittData(99, "CMD_EV_CLOSED");
        break;
      }
    case 4: {
        switchEvacValve(3);
        transmittData(99, "CMD_EV_EVAC");
        break;
      }

    case 50: {
        testMode = 0;
        isArmed = 0;
        transmittData(99, "CMD_DISARMED");
        break;
      }

    case 51: {
        testMode = 1;
        isArmed = 0;
        transmittData(99, "CMD_TESTMODE");
        break;
      }

    case 52: {
        testMode = 0;
        isArmed = 1;
        transmittData(99, "CMD_ARMED");

        break;
      }

    case 53: {
        // RESET EXPERIMENT
        break;
      }
    case 54: {
        // Evac Bottles
        break;
      }
    case 55: { // Lock Table
        tableLock(1);
        transmittData(99, "CMD_TBL_LOCKED");
        break;
      }
    case 56: { // unlock Table
        tableLock(0);
        transmittData(99, "CMD_TBL_UNLOCKED");
        break;
      }
    case 57: { //driving to home position
        transmittData(99, "CMD_HOME");
        setHome();
        break;
      }
    case 58: { //camera ON
        transmittData(99, "CAM_ON");
        cameraPower(1);
        break;
      }
    case 59: { //camera OFF
        transmittData(99, "CAM_OFF");
        cameraPower(0);
        break;
      }
    case 70: {
        transmittData(99, "EXP_RESET");
        resetProgress();
      }
  }
}
void switchMainValve(uint8_t state) {
  mcp.digitalWrite(ValveMainPin, state);
  mcp.digitalWrite(rocketNoseLED, state);
  switchBoardGPIO[2] = state;

  // led.updateLED(5, state);
}

void switchEvacValve(uint8_t state) {
  switch (state) {
    case 1: {
        mcp.digitalWrite(ValveEvacPin1, HIGH);
        mcp.digitalWrite(ValveEvacPin2, LOW);
        mcp.digitalWrite(rocketNoseLED, HIGH);
        mcp.digitalWrite(rocketFlameLED, LOW);
        switchBoardGPIO[3] = 1;
        switchBoardGPIO[4] = 0;
        switchBoardGPIO[5] = 0;


        break;
      }
    case 2: {
        mcp.digitalWrite(ValveEvacPin1, LOW);
        mcp.digitalWrite(ValveEvacPin2, LOW);
        mcp.digitalWrite(rocketNoseLED, LOW);
        mcp.digitalWrite(rocketFlameLED, LOW);
        switchBoardGPIO[3] = 0;
        switchBoardGPIO[4] = 1;
        switchBoardGPIO[5] = 0;

        //      led.updateLED(6, 0);
        //       led.updateLED(7, 0);
        break;
      }
    case 3: {
        mcp.digitalWrite(ValveEvacPin1, LOW);
        mcp.digitalWrite(ValveEvacPin2, HIGH);
        mcp.digitalWrite(rocketNoseLED, LOW);
        mcp.digitalWrite(rocketFlameLED, HIGH);
        switchBoardGPIO[3] = 0;
        switchBoardGPIO[4] = 0;
        switchBoardGPIO[5] = 1;

        //        led.updateLED(6, 0);
        //       led.updateLED(7, 1);
        break;
      }
  }
}



void checkSignals() { // REWRITE!
  LO = !digitalRead(LOPIN);
  SOE = !digitalRead(SOEPIN);
  SODS = !digitalRead(SODSPIN);
}

void switchRelais(uint8_t state) {
  for (int i = 0; i <= 4; i++) {
    mcp.digitalWrite(i, state);
    HPOutputs[i] = state;
  }
}


//********** Reading Temperature **********//

int8_t readBoardTemp() { // read Onboard Temperature
  float floatTemp = tempsensor.readTempC();
  temperature[0] = tempsensor.readTempC();
}

//********** Reading Voltages **********//
uint16_t getInputVoltage() {
  uint16_t sensorValue = 0;
  sensorValue = (analogRead(InputVoltagePin));
  uint32_t U = 3300 * sensorValue / 4096; //Voltage on Chip Pin
  inputVoltage = (14700 * U / 1400) + U; //External Voltage calculated through Voltage divider
  return inputVoltage;
}

//********** Direct Write **********//
void directDigitalWrite(char Port, char Pin, boolean state) {
  switch (Port) {
    case 'A': {
        if (state) REG_PIOA_SODR = 0x1 << Pin;
        else       REG_PIOA_CODR = 0x1 << Pin;
      }
      break;
    case 'B': {
        if (state) REG_PIOB_SODR = 0x1 << Pin;
        else       REG_PIOB_CODR = 0x1 << Pin;
      }
      break;
    case 'C': {
        if (state) REG_PIOC_SODR = 0x1 << Pin;
        else       REG_PIOC_CODR = 0x1 << Pin;
      }
      break;
    case 'D': {
        if (state) REG_PIOD_SODR = 0x1 << Pin;
        else       REG_PIOD_CODR = 0x1 << Pin;
      }
      break;
  }
}

//********** Sending data to Serial Port **********//
void pushData() {
  //$01 Status Flags
  int readyForStart = 0;
  if (progress == 1) {
    readyForStart = 1;
  } else {
    readyForStart = 0;
  }
  bool  boolArray[8] = {isArmed, BatteryStatus, SDCard, readyForStart, testMode, LO, SOE, SODS};
  transmittData(1, boolToByte(boolArray));

  //$02 Input Voltage
  transmittData(2, getInputVoltage());

  //$03 Battery Voltage
  transmittData(3, IGNBatteryVoltage);

  //$04 Message ID
  transmittData(4, messageID);
  messageID++; //Increment MessageID

  //$05 Current Uptime
  transmittData(5, uint64_t((millis() + 500) / 1000));

  //$10 Module Temp
  transmittData(10, temperature[0]);
  transmittData(11, temperature[1]);
  transmittData(12, temperature[2]);
  transmittData(13, temperature[3]);
  transmittData(14, temperature[4]);
  transmittData(15, temperature[5]);
  transmittData(16, temperature[6]);
  //$1-15 Temperature 0-5
  // for (int i = 0; i <= 9; i++) {
  //   transmittData(i + 10, temperature[i]);
  //  }


  //$20-27 Pressure 1-6
  for (int i = 0; i <= 7; i++) {
    transmittData(i + 20, pressure[i]);
  }
  //$30 Turntable Position
  transmittData(30, sampleFacing);

  //$31 motorPowered
  transmittData(31, motorPowered);

  //32 Turntable locked
  transmittData(32, tableLocked);

  //33 Switchboard HP Output
  transmittData(33, boolToByte(HPOutputs));

  //34 Switchboard GPIO
  transmittData(34, boolToByte(switchBoardGPIO));

  //36 Camera Powered
  transmittData(36, cameraPowered);

  //37 Camera Recording
  transmittData(37, cameraRec);

  //99 Text
  // transmittData(99, "test");
  //transmittData(99, setNtcVal());


  setNtcVal();
}

void readData() {
  REQUEST_EXTERNAL_RESET;
}

void transmittData(uint8_t command, int16_t data) {
  String stringToSend = "$";
  if (command < 10) {
    stringToSend += 0;
  }
  stringToSend += command;
  stringToSend += ",";
  stringToSend += data;
  stringToSend += ";";
  char buf[sizeof(stringToSend)];
  stringToSend.toCharArray(buf, sizeof(stringToSend));
  getCheckSum(buf);
  String transmittString = stringToSend + getCheckSum(buf);
  digitalWrite(statusLED, HIGH);
  RS422.print(transmittString);
  DATA(transmittString);
  digitalWrite(statusLED, LOW);
}
void transmittData(uint8_t command, String data) {
  String stringToSend = "$";
  if (command < 10) {
    stringToSend += 0;
  }
  stringToSend += command;
  stringToSend += ",";
  stringToSend += data;
  stringToSend += ";";
  char buf[sizeof(stringToSend)];
  stringToSend.toCharArray(buf, sizeof(stringToSend));
  getCheckSum(buf);
  String transmittString = stringToSend + getCheckSum(buf);
  digitalWrite(statusLED, HIGH);
  RS422.print(transmittString);
  DATA(transmittString);
  digitalWrite(statusLED, LOW);
}

int getCheckSum(char *string) {
  int XOR = 0;
  int c;
  // Calculate checksum ignoring any $'s in the string
  for (int i = 0; i < strlen(string); i++) {
    c = string[i];
    if (c == ';') break;
    if (c != '$') XOR ^= c;
  }
  return XOR;
}

byte boolToByte(bool input[8])
{
  int len = 8;
  if (len > 8)
    len = 8;
  int output = 0;
  for (int i = 0; i < len; i++)
    if (input[i])
      output += (1 << (len - 1 - i)); //this part depends on your system (Big/Little)
  //output += (1 << i); //depends on system
  return (byte)output;
}


//********** Eventhandler **********//
void eventhandler() {
  if (isArmed || testMode) { //start only if the Device is Armed or in Testmode.
    switch (progress) {
      case 1: {
          if (LO) {
            progress++;
            LOGln("LO received", 1);
          }
          break;
        }
      case 2: {
          if (SOE) {
            cameraPower(1); //turn on camera

            cameraRecord(1); //start recording
            switchMainValve(1); //Turn on main valve
            switchEvacValve(1); //switch to chambers
            LOGln("SOE received", 1);

            progress++;
          }
          break;
        }
      case 3: {
          if (isArmed) {
            delay(2000); //wait 2 sec. for airflow
            switchRelais(1); //ignite samples
            LOGln("ignition ON", 1);
          }
          progress++;
          timer0 = millis();
          break;
        }
      case 4: {
          if (isArmed) {
            if (millis() - timer0 >= 20000) {
              //transmittData(99, "turn off ignition");
              switchRelais(0); //turn off ignition
              LOGln("ignition OFF", 1);
              progress++;
            }
          } else { progress++;}
          break;
        }

      case 5: {
          //transmittData(99, "unlock table");

          mcp.digitalWrite(rocketMiddleLED, 1);
          tableLock(0); //unlock turntable
          setMotorEnable(0);
          setMotorEnable(1);
          turnsmoothlyToLastSample();
          progress++;
          timer0 = millis();
          break;
        }

      case 6: {
          if (!SODS) { //Do as long as there is no SODS
            //transmittData(99, "turning...");
            if (millis() - timer0 >= 1000) { //1000ms
              //turn camera to next sample
              mcp.digitalWrite(rocketFlameLED, 1);

              turnToNextSample();
              timer0 = millis(); //reset timer
            }

          }
          else { //If SODS received
            //transmittData(99, "SODS received");
            LOGln("SODS received", 1);
            findHome("A", 1);
            tableLock(1);
            delay(1000); //servo needs some time
            setMotorEnable(0);

            timer0 = millis();
            progress++;
          }
          break;
        }
      case 7: {
          if (isArmed) { //Bottle evacuation process
            switchEvacValve(3);

            if (millis() - timer0 >= 20000) {
              switchEvacValve(2); //Close experiment valve
              progress++;
              timer0 = millis();
            }
          }

          else {
            switchEvacValve(2); //Close experiment valve
            progress++;
          }
          break;
        }
      case 8: {
         
            //transmittData(99, "stop recording");
            cameraRecord(0);

            progress++;
            timer0 = millis();
          
          break;
        }
      case 9: {
          if (millis() - timer0 >= 5000) {
            //transmittData(99, "turn off camera power");
            cameraPower(0);
            switchMainValve(0); //turn off Main Valve
            progress++;
            timer0 = millis();
            transmittData(99, "finish =)");
          }
          break;
        }
    }
  }

}

//********** turntable Control **********//
void turnToNextSample() {
  setMotorEnable(1);
  sampleFacing--;
      LOG("Turn to Sample ", 1);
    LOGln(String(sampleFacing), 0);
  turnCameraTo(sampleFacing);
  if (sampleFacing == 0) {
    sampleFacing = 5;
  }
}

void turnsmoothlyToLastSample() {
  setMotorEnable(1);
  sampleFacing = 4;

  uint32_t stepperSpeedOld = stepperSpeed;
  stepper.setMaxSpeed(1000);
  turnCameraTo(sampleFacing);
  stepper.setMaxSpeed(stepperSpeedOld);

}
void turnCameraTo(uint8_t sampletoTurnTo) {
  setMotorEnable(1);
  uint16_t pos = 0;
  if (tableLocked) {
    tableLock(0);
  }

  switch (sampletoTurnTo) {
    case 0: {
        uint16_t pos = 100;
        stepper.runToNewPosition(pos);
        findHome("A", 1);
        break;
      }
    case 1: {
        pos = sampletoTurnTo * stepsForDegree * degreesBetweenChambers;
        stepper.runToNewPosition(pos);

        break;
      }
    case 2: {
        pos = sampletoTurnTo * stepsForDegree * degreesBetweenChambers;
        stepper.runToNewPosition(pos);
        break;
      }
    case 3: {
        pos = sampletoTurnTo * stepsForDegree * degreesBetweenChambers;
        stepper.runToNewPosition(pos);
        break;
      }
    case 4: {
        pos = sampletoTurnTo * stepsForDegree * (degreesBetweenChambers); //3528
        stepper.runToNewPosition(3528);
        findHome("B", 1);
        break;
      }
  }
}

void tableLock(bool locked) {
  if (locked) {
    servo.write(36);
    tableLocked = true;
  }
  else {
    servo.write(120);
    tableLocked = false;
  }
}

void findHome(char* pos, bool power) {
  uint64_t startHomeTime = 0;
  setMotorEnable(1);
  setMotorCurrent(stepperCurrent);

  if (pos == "A") {
    startHomeTime = millis();
    stepper.runToNewPosition(0);
    while (!digitalRead(homeAPin)) {
      stepper.runToNewPosition(-60);
      stepper.setCurrentPosition(0);
      if (((millis() - startHomeTime ) >= 5000)) {
        ERRln("Could not find home switch A!", 1);
        break;
      }
    }
    sampleFacing = 0;
  }
  else {
    startHomeTime = millis();
    uint16_t cameraPosition = stepper.currentPosition();
    while (!digitalRead(homeBPin)) {
      cameraPosition += 60;
      stepper.runToNewPosition(cameraPosition);
      if (((millis() - startHomeTime ) >= 10000)) {
        ERRln("Could not find home switch B!", 1);
        break;
      }

    }
    sampleFacing = 4;
    cameraPosition = 0;
    stepper.setCurrentPosition(3528);
  }
  if (!power) setMotorEnable(0);
}

//********** Logging Functions **********//
void LOG(String logString, bool formatting) {
  char filename[] = "000LOG.txt";
  sprintf(filename, "%03dLOG.txt", fileVersion);
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (formatting) {
    dataFile.print("[LOG] ");
    dataFile.print(millis());
    dataFile.print(": ");
  }
  if (dataFile) {
    dataFile.print(logString);
    dataFile.close();
  }
}

void LOGln(String logString, bool formatting) {
  char filename[] = "000LOG.txt";
  sprintf(filename, "%03dLOG.txt", fileVersion);
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (formatting) {
    dataFile.print("[LOG] ");
    dataFile.print(millis());
    dataFile.print(": ");
  }
  if (dataFile) {
    dataFile.println(logString);
    dataFile.close();
  }
}

void ERR(String logString, bool formatting) {
  char filename[] = "000ERROR.txt";
  sprintf(filename, "%03dERROR.txt", fileVersion);
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  transmittData(98, logString);
  if (formatting) {
    dataFile.print("[ERROR] ");
    dataFile.print(millis());
    dataFile.print(": ");
  }
  if (dataFile) {
    dataFile.print(logString);
    dataFile.close();
  }
}

void ERRln(String logString, bool formatting) {
  char filename[] = "000ERROR.txt";
  sprintf(filename, "%03dERROR.txt", fileVersion);
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  transmittData(98, logString);
  if (formatting) {
    dataFile.print("[ERROR] ");
    dataFile.print(millis());
    dataFile.print(": ");
  }
  if (dataFile) {
    dataFile.println(logString);
    dataFile.close();
  }
}

void DATAln(String logString) {
  char filename[] = "000DATA.txt";
  sprintf(filename, "%03dDATA.txt", fileVersion);
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(logString);
    dataFile.close();
  }
}

void DATA(String logString) {
  char filename[] = "000DATA.txt";
  sprintf(filename, "%03dDATA.txt", fileVersion);
  File dataFile = SD.open(filename, FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(logString);
    dataFile.close();
  }
}

void setLatestFileVersion() {
  File dataFile;
  char filename[] = "000LOG.txt";
  for (int i = 0; i < 1000; i++) {
    sprintf(filename, "%03dLOG.txt", i);
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      fileVersion = i;
      break; // leave the loop!
    }
  }
}


//********** CoProcessor reading **********//
void readCoProcessor() {
  //Reading the Pressure Values
  for (int i = 0; i <= 7; i++) {
    Wire.beginTransmission(COPROCESSOR_ADRESS);
    Wire.write( i + 6); // set the Pointer
    Wire.endTransmission();
    delay(2);
    int n = Wire.requestFrom(COPROCESSOR_ADRESS, 2);   // request one byte from Slave
    if (n == 2) {
      // Read low byte into buffer
      pressure[i] = Wire.read() << 8;

      // Read high byte into buffer
      pressure[i] += Wire.read() ;
    }
  }

  //Reading the Temperature Values
  for (int i = 0; i <= 4; i++) {
    Wire.beginTransmission(COPROCESSOR_ADRESS);
    Wire.write(i + 20); // set the Pointer
    Wire.endTransmission();
    delay(2);
    int n = Wire.requestFrom(COPROCESSOR_ADRESS, 2);   // request one byte from Slave
    if (n == 2) {
      // Read low byte into buffer
      temperature[i + 1] = Wire.read();

      // Read high byte into buffer
      temperature[i + 1] += Wire.read() << 8 ;
    }

  } //Read the battery Voltage
  Wire.beginTransmission(COPROCESSOR_ADRESS);
  Wire.write(14); // set the Pointer
  Wire.endTransmission();
  delay(2);
  int n = Wire.requestFrom(COPROCESSOR_ADRESS, 2);   // request one byte from Slave
  if (n == 2) {
    uint8_t buff1 = 0;
    uint8_t buff2 = 0;
    uint16_t buff3 = 0;
    // Read bytes into buffer
    buff1 =  Wire.read();
    buff2 = Wire.read();
    buff3 = buff2;
    buff3 += buff1 << 8;
    IGNBatteryVoltage = 163 * float(buff3) / 4092 * 1000;
  }
}

void checkSDCard(bool text) {
  if (!SD.begin(microSDSS)) {
    // If SD Card not properly connected blink
    SDCard = 0;
    if (text) {
      ERRln("Failed to recognize SD - Card", 1);
    }
  }
  else {
    SDCard = 1;
    if (text) {
      if (loglevel < 3) {
        LOGln("SD - Card succesfully initialized", 1);
        transmittData(99, "SD erkannt");
      }
    }
    setLatestFileVersion(); //get the latest file Version
  }
}

void setMotorEnable(bool state) {
  if (state == 1) {
    digitalWrite(nSleep, HIGH);
    delay(2);
    digitalWrite(nEnbl, LOW);
  } else {
    digitalWrite(nSleep, LOW);
    delay(2);
    digitalWrite(nEnbl, HIGH);



  }
  motorPowered = state;
}


void setMotorCurrent(uint16_t milliAmps) {
  //Current is calculated by I=Vxref/(5*Risense) -> Vxref = 5* I * Risense, Risense is 0.2R
  uint16_t motorVoltage = 5 * milliAmps * 0.2;
  //The due Output is 0.55 to 2.75V
  // 3300/4095*B=A -> (4095*Voltage)/3300=adcValue
  uint16_t writeadcValue = 4095 * motorVoltage / 3300;
  //  LOG("Setting Motor Current to " + String(milliAmps) + "mA");
  analogWrite(drvVref, writeadcValue);
}


void cameraPower(bool state) {
  digitalWrite(CameraPowerPin, state);
  cameraPowered = state;
}

void cameraRecord(bool state) {
  digitalWrite(CameraRecordPin, !state);
  cameraRec = state;
}

uint16_t readAnalogPin(int AnalogPin) {
  uint32_t values = 0;
  for (int i = 0; i < AnalogReadSampleRate; i++) {
    values += (analogRead(AnalogPin));
  }
  uint16_t V = (values / AnalogReadSampleRate); //Digital Voltage
  V = V * 3300 / 4096;
  return V;
}

void setNtcVal() {

  uint16_t V = 0;
  float R = 0;
  uint16_t W = 22350;

  V = readAnalogPin(NTC6Pin); //Kammer 1
  R = float((V * W) / (3300 - V));
  temperature[1] = (-3984 / float(log(1 / R) - 4.84521));

  V = readAnalogPin(NTC5Pin); //Kammer 2
  R = float((V * W) / (3300 - V));
  temperature[2] = (-3984 / float(log(1 / R) - 4.84521));

  V = readAnalogPin(NTC4Pin);
  R = float((V * W) / (3300 - V));
  temperature[3] = (-3984 / float(log(1 / R) - 4.84521));

  V = readAnalogPin(NTC3Pin);
  R = float((V * W) / (3300 - V));
  temperature[4] = (-3984 / float(log(1 / R) - 4.84521));

  V = readAnalogPin(NTC2Pin);
  R = float((V * W) / (3300 - V));
  temperature[5] = (-3984 / float(log(1 / R) - 4.84521));

  V = readAnalogPin(NTC1Pin);
  R = float((V * W) / (3300 - V));
  temperature[6] = (-3984 / float(log(1 / R) - 4.84521));
}

void setHome() {
  tableLock(0); //unlocking the table
  findHome("B", 1); //goes to Position B, leaves Motor Power on
  findHome("A", 1); //goes to Position A, leaves Motor Power on
  tableLock(1); //locking table
  delay(500); //waiting 0.5sec for servo to lock
  setMotorEnable(0);  //turns of stepper Motor Power.
}

void resetProgress() {
  progress = 1;
}
