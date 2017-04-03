/*
   Smart Sensor Simulator 2
   Controlling the  Quadtrature Knob, Ignition Relay, and Voltage Regulator
   Hardware Revision 3

   Written by Dr. Jeremy S. Daily
   Synercon Technologies, LLC
   
   Proprietary Software - Not for Public Release

   Copyright (c) 2017        Jeremy S. Daily
*/

//softwareVersion
char softwareVersion[200] = "SSS2*Rev3*0.4*bb1672fcd2fb80092faaea9b7877db6d12e86da2"; //Hash of the previous git commit
char componentID[200] = "SYNER*SSS2-R03*0017*UNIVERSAL"; //Add the serial number for hard coded values.

byte sourceAddress = 0xFA; 


#include <EEPROM.h>
#include <FlexCAN.h>
#include "SSS2.h"
#include <TimeLib.h>




uint8_t DM13_00_Count = 0;
uint8_t DM13_FF_Count = 0;





//set up a display buffer

/****************************************************************/
/*              Setup millisecond timers and intervals          */

#define LEDtoggleTime 500

//Set up message transmitters
uint16_t TXperiods[11] = {10, 20, 50, 100, 200, 250, 500, 1000, 5000, 10000, 30000};

//Declare a millisecond timer to execute the switching of the LEDs on a set time
elapsedMillis toggleTimer;
elapsedMillis RXCAN0timer;
elapsedMillis RXCAN1orJ1708timer;
elapsedMillis CANTX_10ms_timer;
elapsedMillis CANTX_20ms_timer;
elapsedMillis CANTX_50ms_timer;
elapsedMillis CANTX_100ms_timer;
elapsedMillis CANTX_200ms_timer;
elapsedMillis CANTX_250ms_timer;
elapsedMillis CANTX_500ms_timer;
elapsedMillis CANTX_1000ms_timer;
elapsedMillis CANTX_1005ms_timer;
elapsedMillis CANTX_5000ms_timer;
elapsedMillis CANTX_10000ms_timer;
elapsedMillis CANTX_30000ms_timer;
elapsedMillis c59timer;
elapsedMillis c5Btimer;
elapsedMillis CAN0RXtimer;
elapsedMillis CAN1RXtimer;
elapsedMillis transportTimer;

elapsedMicros microsecondsPerSecond;

/********************************************************************************************/
/*                         Begin Function Calls for Knob Buttons                            */

OneButton button(buttonPin, true);



void longPressStart() {
  setLimits(255);
  setSetting(50, !ignitionCtlState, DEBUG_ON);
  digitalWrite(ignitionCtlPin, ignitionCtlState);
  digitalWrite(greenLEDpin, ignitionCtlState);
  CANTX_5000ms_timer = 4950;
  DM13_00_Count = 0;
  DM13_FF_Count = 0;
}

void myClickFunction() {
  turnOffAdjustMode();
  }
void myDoubleClickFunction() {}
void longPress() {} //Do nothing at this time.
void longPressStop() {} 



/*                         End Function Calls for Knob Buttons                            */
/********************************************************************************************/



/****************************************************************/
/*                    CAN Setup                                 */


//Set up the CAN data structures
static CAN_message_t txmsg;

//set up a counter for each received message
uint32_t RXCount0 = 0;
uint32_t RXCount1 = 0;

int CANWaitTimeout = 20;
uint32_t BAUDRATE0 = 250000;
uint32_t BAUDRATE1 = 250000;
const uint32_t baudRateList[4] = {250000,500000,666000,125000};
uint8_t baudRateIndex0 = 0;
uint8_t baudRateIndex1 = 0;


CAN_filter_t allPassFilter;

byte blankCANdata[8] = {255,255,255,255,255,255,255,255};
byte sessionControlMessage[8] = {0x20,0x0E,0x00,0x01,0xFF,0xCA,0xFE,0x00};
byte sessionTransportMessage[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; // This eliminates an unknown code in DDEC Reports

byte feca03Data[8] ={0x00,0xFF,0x00,0x00,0x00,0x00,0xFF,0xFF};
byte ff0903DataList[2][8] ={
 {0x00,0x00,0x00,0x08,0x00,0x00,0x00,0x00},
 {0x00,0x00,0x00,0x58,0x00,0x00,0x00,0x00}
};
byte ff0903Data[8] ={0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,0xAA};
byte ff0803Data[8] ={0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,0xAA};
byte fef803Data[8] ={0xFF,0xFF,0xFF,0xFF,0x59,0x29,0xFF,0xFF};
byte _8ff0203Data[8] ={0xD2,0x04,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF};
byte _4ff0203Data[8] ={0xFF,0xFF,0x00,0xFF,0xFF,0xFF,0xFF,0xFF};
byte ff0503Data[8] ={0xFF,0x2D,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte ff8003Data[8] ={0xFC,0x03,0x3F,0xFF,0xFF,0xFF,0xFF,0xFF};

byte ff0403Data[8] ={0xFC,0x03,0x64,0xE0,0x00,0xC3,0x00,0x00};
byte ff0403sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
byte ff0403sevenByte[16]={0x25,0x17,0x41,0x73,0xED,0xDF,0x89,0xBB,0x2E,0x1C,0x4A,0x78,0xE6,0xD4,0x82,0xB0};
int index_ff0403 =0;

byte ff0003Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
byte ff0003sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
// This works:
//byte ff0003sevenByte[16]={0x70,0x42,0x14,0x26,0xB8,0x8A,0xDC,0xEE,0x7B,0x49,0x1F,0x2D,0xB3,0x81,0xD7,0xE5};
 byte ff0003sevenByte[16]={0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int index_ff0003 =0;

//This block Works for J1939 Transmission Output shaft speed
//byte ff0103Data[8] ={0xFF,0xFF,0x00,0x00,0x00,0x5C,0x00,0x00};
//byte ff0103sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
//byte ff0103sevenByte[16]={0xD6,0xE4,0xB2,0x80,0x1E,0x2C,0x7A,0x48,0xDD,0xEF,0xB9,0x8B,0x15,0x27,0x94,0x43};

//This block also works for J1939 Transmission Output Shaft Speed
//byte ff0103Data[8] ={0xFF,0xFF,0x00,0x00,0xFF,0xDF,0x00,0x00};
  byte ff0103Data[8] ={0xFF,0xFF,0x00,0x00,0xFF,0xDF,0x00,0x00};
byte ff0103sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
byte ff0103sevenByte[16]={0x33,0x01,0x57,0x65,0xFB,0xC9,0x9F,0xAD,0x38,0x0A,0x5C,0x6E,0xF0,0xC2,0x94,0xA6};
// byte ff0103sevenByte[16]={0x00,0x00,0x00,0x65,0xFB,0xC9,0x9F,0xAD,0x38,0x0A,0x5C,0x6E,0xF0,0xC2,0x94,0xA6};
//byte ff0103sevenByte[16]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5C,0x00,0x00,0x00,0x00,0x00};
int index_ff0103 =0;

//Changes J1939 FE4A some.
byte ff0203Data[8] ={0xFF,0xFF,0xFF,0xFF,0xD2,0xF1,0x00,0x00};
byte ff0203sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
byte ff0203sevenByte[16]={0x8D,0xBF,0xE9,0xD8,0x45,0x77,0x21,0x13,0x86,0x84,0xE2,0xD0,0x4E,0x7C,0x2A,0x18};
int index_ff0203 =0;

byte ff0603Data[8] ={0xFF,0xFF,0xFF,0xFF,0xF3,0x7F,0x00,0x00};
byte ff0603sixthByte[16]={0x0C,0x1C,0x2C,0x3C,0x4C,0x5C,0x6C,0x7C,0x8C,0x9C,0xAC,0xBC,0xCC,0xDC,0xEC,0xFC};
byte ff0603sevenByte[16]={0xD7,0xE5,0xB3,0x81,0x1F,0x2D,0x7B,0x49,0xDC,0xEE,0xB8,0x8A,0x14,0x26,0x70,0x42};
int index_ff0603 =0;

byte ff0703Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
byte ff0703sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
//byte ff0703sevenByte[16]={0x70,0x42,0x14,0x26,0xB8,0x8A,0xDC,0xEE,0x7B,0x49,0x1F,0x2D,0xB3,0x81,0xD7,0xE5};
byte ff0703sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
int index_ff0703 =0;

byte ff0903sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
byte ff0903sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
int index_ff0903 =0;


byte Cff0703Data[8] ={0xD0,0x20,0xC3,0x00,0xFF,0x5A,0x00,0x00};
//byte Cff0703fifthByte[16]={0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82};
byte Cff0703sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
// This works: 
//byte Cff0703sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
byte Cff0703sevenByte[16]={0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
int index_Cff0703 =0;

//byte ff0303Data[8] ={0x7D,0x8C,0xFF,0xFF,0xA8,0x61,0xA8,0x61};
byte ff0303Data[8] ={0xFF,0x8C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

byte Cff0203Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte Cff0303Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//byte Cff0303Data[8] ={0xE2,0xE2,0xE2,0xE2,0xE2,0xE2,0xE2,0xE2};
byte Cff0403Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

byte DFFFF9[8] = {0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF};
byte DF00F9[8] = {0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF};

int k = 0;
byte LINDataBytes[3];
byte outByte[5];
boolean firstLINtime=true;
boolean LIN0send = false;
boolean LIN1send = false;
boolean LIN2send = false;
boolean LIN3send = false;
boolean LIN4send = false;

unsigned long currentMicros;
unsigned long previousMicros;
unsigned long serialSend0Micros;
unsigned long serialSend1Micros;
unsigned long serialSend2Micros;
unsigned long serialSend3Micros;
unsigned long serialSend4Micros;


/****************************************************************/
/*                 Binary State Variables                       */
//Keep track of the current state of the LEDs

boolean ADJUST_MODE_ON  = 0;
boolean SAFE_TO_ADJUST = 0;
boolean displayCAN = 0;
boolean displayCAN0 = 0;
boolean displayCAN1 = 0;
boolean displayCAN2 = 0;
boolean CAN0baudNotDetected = true;
boolean CAN1baudNotDetected = true;
boolean TXCAN = true;
boolean enableSendComponentInfo = true;

boolean send08FF0001 = true;
boolean send08FF0003 = true;
boolean send08FF0103 = true;
boolean send08FF0203 = true;
boolean send08FF0303 = true;
boolean send08FF0603 = true;
boolean send08FF0703 = true;
boolean send0CFF0703 = true;
boolean send0CFE6E0B = true;
boolean send10FF0903 = true;
boolean send18F00131 = true;
boolean send18F0010B = true;
boolean send18FEF117 = true;
boolean send18FEF128 = true;
boolean send18FEF121 = true;
boolean send18FEF131 = true;
boolean send18E00017 = true;
boolean send18E00019 = true;
boolean send18E00021 = true;
boolean send18E00028 = true;
boolean send18E00031 = true;
boolean send10ECFF3D = true;
boolean send10ECFF01 = true;
boolean send18FEF803 = true;
boolean send18FEF521 = true;
boolean send18FEF017 = true;
boolean send18FEF021 = true;
boolean send18FEF028 = true;
boolean send18FEF031 = true;
boolean send18DF00F9 = false;
boolean send18DFFFF9 = false;
boolean send0CF00203 = true;
boolean send18F00503 = true;

boolean displayJ1939 = false;
const uint8_t numPeriodicCANMessages = 33;

String inputString = "";

String commandChars = "";
String commandString = "";
String commandExtension = "";

uint16_t currentSetting = 0;



void  adjustError() {
  Serial.println(F("INFO SS - Condition not met. Turn adjust mode on by typing AO, then select a setting with CS"));
}






/********************************************************************************************/
/*                       Begin Function Calls for Serial Commands                           */

void turnOnAdjustMode() {
  ADJUST_MODE_ON = 1;
  Serial.println(F("INFO AO - Turned Adjustment mode on. Type AF or click to turn off. Type SS,XXXX  or scroll knob and click to set settings."));
  Serial.print(F("INFO Current Setting for Adjustement is "));
  Serial.print(currentSetting);
  Serial.print(" - ");
  Serial.println(settingNames[currentSetting]);
  knob.write(setSetting(currentSetting,-1,DEBUG_OFF));
  setLimits(currentSetting);

}

void turnOffAdjustMode() {
  ADJUST_MODE_ON = 0;
  Serial.println(F("INFO AF - Turned Setting Adjustment mode off. Type AO to turn on. Scroll knob to select a setting."));
  knob.write(currentSetting);
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
}

void fastSetSetting(){
  int returnval;
  currentSetting = commandChars.toInt();
  if (currentSetting > 0 && currentSetting < numSettings){
    setLimits(currentSetting);
    if (commandString.length() > 0){ 
      long settingValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
      returnval = setSetting(currentSetting, settingValue,DEBUG_OFF);
    }
    else{
      returnval = setSetting(currentSetting, -1, DEBUG_OFF);
    }
    Serial.print("SET ");
    Serial.print(currentSetting);
    Serial.print(",");
    Serial.println(returnval);  
  }
  else Serial.println(F("ERROR in setting value."));
  
}

void changeSetting() {
  Serial.println(F("INFO CS - Change or Select Setting."));
  if (commandString.length() > 0) {
    currentSetting = constrain(commandString.toInt(), 0, numSettings);
    
  }
  //listSetting(currentSetting);
  if (ADJUST_MODE_ON){
    setLimits(currentSetting);
    knob.write(setSetting(currentSetting,-1,DEBUG_OFF));
  }
  else{
    if (knob.read() == currentSetting){
      Serial.print("INFO ");
      setSetting(currentSetting,-1,DEBUG_ON);
    }
    else knob.write(currentSetting); //automatic listSetting if knob changes
    knobLowLimit = 1;
    knobHighLimit = numSettings - 1;
  }
}

void listSettings(){
  Serial.println(F("INFO LS - List Settings. "));
  for (int i = 1; i < numSettings; i++) {
    Serial.print("INFO ");
    setSetting(i,-1,DEBUG_ON);
  }
}

void changeValue(){
  //Set value from Serial commands
  if (ADJUST_MODE_ON && currentSetting != 0) {
    Serial.println(F("INFO SS - Set Setting."));
    int adjustmentValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
    currentKnob = setSetting(currentSetting, adjustmentValue,DEBUG_ON);
    knob.write(currentKnob);
  }
  else
  {
    adjustError();
  }
}

void saveEEPROM(){
  //Save settings to EEPROM
  Serial.println(F("INFO SA - Saving Settings to EEPROM."));
  setDefaultEEPROMdata();
}

void displayVoltage(){
  float reading = analogRead(A21);
  Serial.print("A21 ");
  Serial.println((reading*reading*.008003873 + 8.894535*reading)*.001);
}


/*                End Function Calls for Serial and Knob Commands                           */
/********************************************************************************************/












/**************************************************************************************/
/*               Begin Function calls for User input data                             */


void listInfo() {
  Serial.print("INFO Component ID (Make*Model*Serial*Unit): ");
  Serial.println(componentID);
  Serial.print("INFO Programmed for: ");
  Serial.println(programmedFor);
  Serial.print("INFO Programmed by: ");
  Serial.println(programmedBy);
  Serial.print("INFO Program Date: ");
  Serial.println(programDate);
  Serial.print("INFO Software Version: ");
  Serial.println(softwareVersion);
  Serial.print("INFO Program Notes: ");
  Serial.println(programNotes);
  
}

void setProgrammedFor() {
  if (commandString.length() > 0) commandString.toCharArray(programmedFor,200);
  Serial.print("SET PF - SSS unit is programmed for: ");
  Serial.println(programmedFor);
}

void setProgrammedBy() {
  if (commandString.length() > 0) commandString.toCharArray(programmedBy,200);
  Serial.print("SET PB - SSS unit was programmed by: ");
  Serial.println(programmedBy);
}

void setProgramDate() {
  if (commandString.length() > 0) commandString.toCharArray(programDate,200);
  Serial.print("SET PD - Date SSS unit was programmed: ");
  Serial.println(programDate);
}

void getSoftwareVersion() {
  Serial.print("INFO SW - Software/Firmware version: ");
  Serial.println(softwareVersion);
}

void changeComponentID() {
  if (commandString.length() > 5) commandString.toCharArray(componentID,200);
  Serial.print(F("SET CI - Component Information (Make*Model*Serial*Unit): "));
  Serial.println(componentID);
  if (commandString.length() <= 5 && commandString.length() > 0) Serial.println(F("Please make the component ID longer than 5 characters to change it."));
}

void setProgramNotes(){
  if (commandString.length() > 0) commandString.toCharArray(programNotes,1000);
  Serial.println(F("SET PN - Programmer Notes: "));
  Serial.println(programNotes);
}

void setVIN(){
  if (commandString.length() > 0) commandString.toCharArray(vehicleIdentificationNum,20);
  Serial.println(F("SET VI - Vehicle Identification Number (VIN): "));
  Serial.println(vehicleIdentificationNum);
}

void setDisplayCAN(){
  Serial.print(F("INFO DS - Display CAN Messages "));
  displayCAN = !displayCAN;
  if (displayCAN) Serial.println("on.");
  else Serial.println("off.");
}

void autoBaud0(){
  Serial.println(F("B0 - Set the baudrate for CAN 0 or select AutoBaud"));
  char baudstring[9];
  if (commandString.length() > 0){
    commandString.toCharArray(baudstring,9);
    BAUDRATE0 = strtoul(baudstring,0,10);
    Serial.print("SET CAN0 baudrate set to ");
    Serial.println(BAUDRATE0);
    for (uint8_t baudRateIndex = 0; baudRateIndex<sizeof(baudRateList); baudRateIndex++){
      if (BAUDRATE0 == baudRateList[baudRateIndex]){
        Can0.begin(BAUDRATE0);
        for (uint8_t filterNum = 4; filterNum < 16;filterNum++) Can0.setFilter(allPassFilter,filterNum);
        CAN0baudNotDetected = false;
        break; 
      }
      else{
        CAN0baudNotDetected = true;
      }
      
    }
  }
  else {
    BAUDRATE0 = 0;
  } 
  if (BAUDRATE0 == 0){
    CAN0baudNotDetected = true;
    Serial.println("INFO CAN0 set to automatically set baudrate.");
  }
}

void autoBaud1(){
  Serial.println(F("INFO B1 - Set the baudrate for CAN 1 or select AutoBaud"));
  char baudstring[9];
  if (commandString.length() > 0){
    commandString.toCharArray(baudstring,9);
    BAUDRATE1 = strtoul(baudstring,0,10);
    Serial.print("INFO CAN1 baudrate set to ");
    Serial.println(BAUDRATE1);
    for (uint8_t baudRateIndex = 0; baudRateIndex<sizeof(baudRateList); baudRateIndex++){
      if (BAUDRATE1 == baudRateList[baudRateIndex]){
          Can1.begin(BAUDRATE1);
          for (uint8_t filterNum = 4; filterNum < 16;filterNum++) Can1.setFilter(allPassFilter,filterNum);
          CAN1baudNotDetected = false;
          break;
        }
        else{
         CAN1baudNotDetected = true;
        }
      }
  }
  else {
    BAUDRATE1 = 0;
  } 
  if (BAUDRATE1 == 0){
    CAN1baudNotDetected = true;
    Serial.println(F("INFO CAN1 set to automatically set baudrate."));
  }
}

void displayBaud(){
  Serial.print("SET CAN0 Baudrate");
  Serial.println(BAUDRATE0);
  Serial.print("SET CAN1 Baudrate");
  Serial.println(BAUDRATE1);  
}

void startStopCAN0Streaming(){
  if (commandString.toInt() > 0) displayCAN0 = true;
  else  displayCAN0 = false;
}

void startStopCAN1Streaming(){
  if (commandString.toInt() > 0) displayCAN1 = true;
  else  displayCAN1 = false;
}

void startStopCAN2Streaming(){
  if (commandString.toInt() > 0) displayCAN2 = true;
  else  displayCAN1 = false;
}

void setEnableComponentInfo(){
  if (commandString.toInt() > 0){
    enableSendComponentInfo = true;
    Serial.print(F("SET Enable CAN transmission of Component ID"));
  }
  else{
    enableSendComponentInfo = false;
    Serial.print(F("SET Disable CAN transmission of Component ID"));  
  }
  
}


time_t getTeensy3Time(){
  microsecondsPerSecond = 0;
  return Teensy3Clock.get();
}


  
void setup() {
  Serial.begin(4000000);
  Serial1.begin(19200);
  
  setSyncProvider(getTeensy3Time);
  setSyncInterval(1);
  
  setPinModes();

  digitalWrite(redLEDpin,redLEDstate);
  digitalWrite(greenLEDpin,LOW);
  digitalWrite(CSdispPin,HIGH);
  digitalWrite(CSCANPin,HIGH);
  digitalWrite(CSanalogPin,HIGH);
  digitalWrite(CStermPin,HIGH);
  digitalWrite(CStouchPin,HIGH);
  digitalWrite(IH1Pin,LOW);
  digitalWrite(IH2Pin,LOW);
  digitalWrite(IL1Pin,LOW);
  digitalWrite(IL2Pin,LOW);
  digitalWrite(ignitionCtlPin,LOW);
  
  inputString.reserve(200);
 
  PotExpander.begin(7);  //U33
  ConfigExpander.begin(3); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
  }
  
  delay(800);
  Serial.println(F("Welcome to the Smart Sensor Simulator 2."));
  
  
  Serial.println(F("Setting up the I2C GPIO extenders."));
  Wire.begin();
  Wire.setDefaultTimeout(200000); // 200ms
  PotExpander.writeGPIOAB(0xFFFF);
  ConfigExpander.writeGPIOAB(0xFFFF);
  Serial.println("Finished Setting Up MCP23017 extender chips.");
  
  SPI.begin();
  Serial.print(F("Termination Switches (U29): "));
  terminationSettings = setTerminationSwitches();
  Serial.println(terminationSettings,BIN);

  Serial.print("Configration Switches (U21): ");
  uint16_t configSwitchSettings = setConfigSwitches();
  Serial.println(configSwitchSettings,BIN);
  
  
  
  Serial.println(F("Attaching Button Press Handlers."));
  button.attachClick(myClickFunction);
  button.attachDoubleClick(myDoubleClickFunction);
  button.attachLongPressStart(longPressStart);
  button.attachLongPressStop(longPressStop);
  button.attachDuringLongPress(longPress);
  button.setPressTicks(2000);
  button.setClickTicks(250);

  Serial.println(F("Initializing the Analog Out Converter."));
  initializeDACs(Vout2address);


  listInfo();
  for (int i = 1; i < numSettings; i++) {
    currentSetting = setSetting(i, -1,DEBUG_OFF);
    setSetting(i, currentSetting ,DEBUG_ON);
  }
  
  currentSetting = 0;
  

  
  
  
  Serial.println("Done.");
  
  
  setConfigSwitches();
    
   
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
  
  Serial.println(F("Finished Starting Up... Type a command:"));
 
 
}



void loop() {
  
  
  
    
  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 140) {
    commandChars = Serial.readStringUntil(',');
    if (Serial.available()) 
    commandString = Serial.readStringUntil('\n');
    else commandString = "";
    //if (Serial.available()) commandExtension = Serial.readStringUntil('\n');
   // else commandExtension = "-1";
    //Serial.println(F("Please put a comma after the two command characters."));
    if      (commandChars.toInt() > 0) fastSetSetting();  
    else if (commandChars.startsWith("SS") || commandChars.startsWith("ss")) changeValue();
    else if (commandChars.startsWith("SC") || commandChars.startsWith("sc")) Serial.println(F("SC - Not implemented yet."));
    else if (commandChars.startsWith("SA") || commandChars.startsWith("sa")) saveEEPROM();
    else if (commandChars.startsWith("AO") || commandChars.startsWith("ao")) turnOnAdjustMode();
    else if (commandChars.startsWith("AF") || commandChars.startsWith("af")) turnOffAdjustMode();
    else if (commandChars.startsWith("CS") || commandChars.startsWith("cs")) changeSetting(); //Select setting to change
    else if (commandChars.startsWith("CI") || commandChars.startsWith("ci")) changeComponentID();
    else if (commandChars.startsWith("LS") || commandChars.startsWith("ls")) listSettings();
    else if (commandChars.startsWith("LI") || commandChars.startsWith("li")) listInfo();
    else if (commandChars.startsWith("PF") || commandChars.startsWith("pf")) setProgrammedFor();
    else if (commandChars.startsWith("PB") || commandChars.startsWith("pb")) setProgrammedBy();
    else if (commandChars.startsWith("PD") || commandChars.startsWith("pd")) setProgramDate();
    else if (commandChars.startsWith("PN") || commandChars.startsWith("pn")) setProgramNotes();
    else if (commandChars.startsWith("SW") || commandChars.startsWith("sw")) getSoftwareVersion();
    else if (commandChars.startsWith("VI") || commandChars.startsWith("vi")) setVIN();
     else if (commandChars.startsWith("DB") || commandChars.startsWith("db")) displayBaud();
    else if (commandChars.startsWith("AI") || commandChars.startsWith("ai")) displayVoltage();
   
    
    else Serial.println(F("ERROR Unrecognized Command Characters. Use a comma after the command.\nERROR Known commands are CN, B0, B1, DS, VI, SW, PN, PD, PB, PF, LI, LS, CI, CS, AF, AO, SA, SC, SS, or SM."));
    Serial.flush();
  }
  /*              End Serial Command Processing                   */
  /****************************************************************/

  /****************************************************************/
  /*            Begin Quadrature Knob Processing                  */
  button.tick(); //check for presses
  int32_t newKnob = knob.read(); //check for turns
  if (newKnob != currentKnob) {
    if (newKnob >= knobHighLimit) {  //note: knob limits are for each input parameter
      knob.write(knobHighLimit);
      currentKnob = knobHighLimit;
    }
    else if (newKnob <= knobLowLimit) {
      knob.write(knobLowLimit);
      currentKnob = knobLowLimit;
    }
    else
    {
      currentKnob = newKnob;
    }
    //Place function calls to execute when the knob turns.
    if (ADJUST_MODE_ON) {
      //setLimits(currentSetting);
      setSetting(currentSetting, currentKnob,DEBUG_ON);
      
    }
    else {
      currentSetting = currentKnob;
      setSetting(currentSetting,-1,DEBUG_ON);
    }
  }
  /*            End Quadrature Knob Processing                    */
  /****************************************************************/


 
  
}

