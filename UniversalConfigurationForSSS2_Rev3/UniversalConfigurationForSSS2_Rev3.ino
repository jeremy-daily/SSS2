

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
char softwareVersion[200] = "SSS2*Rev3*0.5*CAN-Fixes*d05f5f9df9027442f473471a37743c64a8cdbd42"; //Hash of the previous git commit
char componentID[200] = "SYNER*SSS2-R03*XXXX*UNIVERSAL"; //Add the serial number for hard coded values.

byte sourceAddress = 0xFA; 
#include <SPI.h>
#include <EEPROM.h>
#include <FlexCAN.h>
#include "SSS2.h"
#include <TimeLib.h>
#include <TeensyID.h>

//The Unique CHIP ID variable
uint32_t uid[4];

//Sequential CAN message counters
uint8_t DM13_00_Count = 0;
uint8_t DM13_FF_Count = 0;

/****************************************************************/
/*              Setup millisecond timers and intervals          */
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
elapsedMillis A21TX_Timer;
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
static CAN_message_t rxmsg,txmsg;

//set up a counter for each received message
uint32_t RXCount0 = 0;
uint32_t RXCount1 = 0;

int CANWaitTimeout = 200;
uint32_t BAUDRATE0 = 250000;
uint32_t BAUDRATE1 = 250000;
const uint32_t baudRateList[5] = {250000,500000,666666,125000,1000000};
uint8_t baudRateIndex0 = 0;
uint8_t baudRateIndex1 = 0;


CAN_filter_t allPassFilter;

byte blankCANdata[8] = {0,0,0,0,0,0,0,0}; 
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
byte Cff0703sixthByte[16]={0x0F,0x1F,0x2F,0x3F,0x4F,0x5F,0x6F,0x7F,0x8F,0x9F,0xAF,0xBF,0xCF,0xDF,0xEF,0xFF};
// This works: 
byte Cff0703sevenByte[16]={0x02,0x30,0x66,0x54,0xCA,0xF8,0xAE,0x9C,0x09,0x3B,0x6D,0x5F,0xC1,0xF3,0xA5,0x97};
int index_Cff0703 =0;

byte ff0303Data[8] ={0xFF,0x8C,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

byte Cff0203Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
byte Cff0303Data[8] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
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

boolean sendA21voltage = false;

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
  sprintf(displayBuffer,"A21 %10lu,%2.6f",millis(),(reading*reading*.008003873 + 8.894535*reading)*.001);
  Serial.println(displayBuffer);
}

void streamVoltage(){
  
  if (commandString.toInt() > 0){
    sendA21voltage = true;
    Serial.println("SET Stream analog in data on."); 
  }
  else {
    Serial.println("SET Stream analog In data off.");
    sendA21voltage = false;
  }
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

void checkAgainstUID(){
  String secret = kinetisUID();
  if(commandString==secret) Serial.println("OK:Authenticated");
  else Serial.println("OK:Denied");
}


void startStopCAN(){
  int signalNumber = 0;
  int state = 0;
  char commandCharBuffer[8];
  commandString.toCharArray(commandCharBuffer,8);
  
  sscanf(commandCharBuffer, "%d,%b", &signalNumber, &state); 

//  //Serial.println(F("INFO CN - CAN Transmission."));
//  if (commandString.length() > 0) {
//    commandString.toCharArray(commandCharBuffer,6);
//    signalNumber = atoi(commandCharBuffer);
//  }
       if (signalNumber == 1)  send08FF0001 = state; 
  else if (signalNumber == 2)  send08FF0003 = state;
  else if (signalNumber == 3)  send08FF0103 = state;
  else if (signalNumber == 4)  send08FF0203 = state;
  else if (signalNumber == 5)  send08FF0303 = state;
  else if (signalNumber == 6)  send08FF0603 = state;
  else if (signalNumber == 7)  send08FF0703 = state;
  else if (signalNumber == 8)  send0CFF0703 = state;
  else if (signalNumber == 9)  send0CFE6E0B = state;
  else if (signalNumber == 10) send10FF0903 = state;
  else if (signalNumber == 11) send18F00131 = state;
  else if (signalNumber == 12) send18F0010B = state;
  else if (signalNumber == 13) send18FEF117 = state;
  else if (signalNumber == 14) send18FEF128 = state;
  else if (signalNumber == 15) send18FEF121 = state;
  else if (signalNumber == 16) send18FEF131 = state;
  else if (signalNumber == 17) send18E00017 = state;
  else if (signalNumber == 18) send18E00019 = state;
  else if (signalNumber == 19) send18E00021 = state;
  else if (signalNumber == 20) send18E00028 = state;
  else if (signalNumber == 21) send18E00031 = state;
  else if (signalNumber == 22) send10ECFF3D = state;
  else if (signalNumber == 23) send10ECFF01 = state;
  else if (signalNumber == 24) send18FEF803 = state;
  else if (signalNumber == 25) send18FEF521 = state;
  else if (signalNumber == 26) send18FEF017 = state;
  else if (signalNumber == 27) send18FEF021 = state;
  else if (signalNumber == 28) send18FEF028 = state;
  else if (signalNumber == 29) send18FEF031 = state;
  else if (signalNumber == 30) {send18DF00F9 = state; DM13_00_Count = 0;}
  else if (signalNumber == 31) {send18DFFFF9 = state; DM13_FF_Count = 0;}
  else if (signalNumber == 32) send0CF00203 = state; 
  else if (signalNumber == 33) send18F00503 = state;
  
  
  else if (signalNumber == 254) {
    TXCAN = true;
    Serial.println(F("INFO CAN Transmission turned on for all messages."));
    send08FF0001 = true;
    send08FF0003 = true;
    send08FF0103 = true;
    send08FF0203 = true;
    send08FF0303 = true;
    send08FF0603 = true;
    send08FF0703 = true;
    send0CFF0703 = true;
    send0CFE6E0B = true;
    send10FF0903 = true;
    send18F00131 = true;
    send18F0010B = true;
    send18FEF117 = true;
    send18FEF128 = true;
    send18FEF121 = true;
    send18FEF131 = true;
    send18E00017 = true;
    send18E00019 = true;
    send18E00021 = true;
    send18E00028 = true;
    send18E00031 = true;
    send10ECFF3D = true;
    send10ECFF01 = true;
    send18FEF803 = true;
    send18FEF017 = true;
    send18FEF021 = true;
    send18FEF028 = true;
    send18FEF031 = true;
    send18DF00F9 = true;
    send18DFFFF9 = true;
    send0CF00203 = true;
    send18F00503 = true;
    
  }
  else if (signalNumber == 255) {
    TXCAN=false;
    Serial.println(F("INFO CAN Transmission turned off for all messages."));
    send08FF0001 = false;
    send08FF0003 = false;
    send08FF0103 = false;
    send08FF0203 = false;
    send08FF0303 = false;
    send08FF0603 = false;
    send08FF0703 = false;
    send0CFF0703 = false;
    send0CFE6E0B = false;
    send10FF0903 = false;
    send18F00131 = false;
    send18F0010B = false;
    send18FEF117 = false;
    send18FEF128 = false;
    send18FEF121 = false;
    send18FEF131 = false;
    send18E00017 = false;
    send18E00019 = false;
    send18E00021 = false;
    send18E00028 = false;
    send18E00031 = false;
    send10ECFF3D = false;
    send10ECFF01 = false;
    send18FEF803 = false;
    send18FEF521 = false;
    send18FEF803 = false;
    send18FEF017 = false;
    send18FEF021 = false;
    send18FEF028 = false;
    send18FEF031 = false;
    send18DF00F9 = false;
    send18DFFFF9 = false;
    send0CF00203 = false;
    send18F00503 = false;
    
  }
  else {
    for (signalNumber = 1; signalNumber < numPeriodicCANMessages+1; signalNumber++) displayCANTXswitch (signalNumber);
  }
  if (signalNumber > 0 && signalNumber < numPeriodicCANMessages+1) {
    displayCANTXswitch (signalNumber);
    TXCAN = true;
  }
}

void displayCANTXswitch (int signalNumber){
  if      (signalNumber == 1)  {Serial.print(F("SET 1: send08FF0001 = ")); Serial.println(send08FF0001);}
  else if (signalNumber == 2)  {Serial.print(F("SET 2: send08FF0003 = ")); Serial.println(send08FF0003);}
  else if (signalNumber == 3)  {Serial.print(F("SET 3: send08FF0103 = ")); Serial.println(send08FF0103);}
  else if (signalNumber == 4)  {Serial.print(F("SET 4: send08FF0203 = ")); Serial.println(send08FF0203);}
  else if (signalNumber == 5)  {Serial.print(F("SET 5: send08FF0303 = ")); Serial.println(send08FF0303);}
  else if (signalNumber == 6)  {Serial.print(F("SET 6: send08FF0603 = ")); Serial.println(send08FF0603);}
  else if (signalNumber == 7)  {Serial.print(F("SET 7: send08FF0703 = ")); Serial.println(send08FF0703);}
  else if (signalNumber == 8)  {Serial.print(F("SET 8: send0CFF0703 = ")); Serial.println(send0CFF0703);}
  else if (signalNumber == 9)  {Serial.print(F("SET 9: send0CFE6E0B = ")); Serial.println(send0CFE6E0B);}
  else if (signalNumber == 10) {Serial.print(F("SET 10: send10FF0903 = ")); Serial.println(send10FF0903);}
  else if (signalNumber == 11) {Serial.print(F("SET 11: send18F00131 = ")); Serial.println(send18F00131);}
  else if (signalNumber == 12) {Serial.print(F("SET 12: send18F0010B = ")); Serial.println(send18F0010B);}
  else if (signalNumber == 13) {Serial.print(F("SET 13: send18FEF117 = ")); Serial.println(send18FEF117);}
  else if (signalNumber == 14) {Serial.print(F("SET 14: send18FEF128 = ")); Serial.println(send18FEF128);}
  else if (signalNumber == 15) {Serial.print(F("SET 15: send18FEF121 = ")); Serial.println(send18FEF121);}
  else if (signalNumber == 16) {Serial.print(F("SET 16: send18FEF131 = ")); Serial.println(send18FEF131);}
  else if (signalNumber == 17) {Serial.print(F("SET 17: send18E00017 = ")); Serial.println(send18E00017);}
  else if (signalNumber == 18) {Serial.print(F("SET 18: send18E00019 = ")); Serial.println(send18E00019);}
  else if (signalNumber == 19) {Serial.print(F("SET 19: send18E00021 = ")); Serial.println(send18E00021);}
  else if (signalNumber == 20) {Serial.print(F("SET 20: send18E00028 = ")); Serial.println(send18E00028);}
  else if (signalNumber == 21) {Serial.print(F("SET 21: send18E00031 = ")); Serial.println(send18E00031);}
  else if (signalNumber == 22) {Serial.print(F("SET 22: send10ECFF3D = ")); Serial.println(send10ECFF3D);}
  else if (signalNumber == 23) {Serial.print(F("SET 23: send10ECFF01 = ")); Serial.println(send10ECFF01);}
  else if (signalNumber == 24) {Serial.print(F("SET 24: send18FEF803 = ")); Serial.println(send18FEF803);}
  else if (signalNumber == 25) {Serial.print(F("SET 25: send18FEF521 = ")); Serial.println(send18FEF521);}
  else if (signalNumber == 26) {Serial.print(F("SET 26: send18FEF017 = ")); Serial.println(send18FEF017);}
  else if (signalNumber == 27) {Serial.print(F("SET 27: send18FEF021 = ")); Serial.println(send18FEF021);}
  else if (signalNumber == 28) {Serial.print(F("SET 28: send18FEF028 = ")); Serial.println(send18FEF028);}
  else if (signalNumber == 30) {Serial.print(F("SET 30: send18DF00F9 = ")); Serial.println(send18DF00F9);}
  else if (signalNumber == 31) {Serial.print(F("SET 31: send18DFFFF9 = ")); Serial.println(send18DFFFF9);}
  else if (signalNumber == 32) {Serial.print(F("SET 32: send0CF00203 = ")); Serial.println(send0CF00203);}
  else if (signalNumber == 33) {Serial.print(F("SET 33: send18F00503 = ")); Serial.println(send18F00503);}

}

void sendMessage(){
  /* Sends a CAN message from the following string fields:
   *  channel,id,data
   *  channel is 0 for CAN0 and 1 for CAN1
   *  id is the CAN ID. if the ID is less than 11 bits and the first bit is set, then it will be transmitted as an extended id
   *  data length code is determined by the length of the data
   *  data is the hex representation in ascii with no spaces. there will be 16 hex characters for 8 bytes. Data longer than 8 bytes will be ignored. 
   */
  boolean goodID = false;
  boolean goodData = false;
  
  //Serial.println(F("SM - Send Message."));
  //Serial.println(commandString);
  char commandCharBuffer[100];
  char IdCharBuffer[9];
  char dataCharBuffer[17];
  char *endptr;
  if (commandString.length() > 0) {
    commandString.toCharArray(commandCharBuffer,commandString.length());
    int channel;
    if (commandCharBuffer[0]=='1') channel = 1;
    else if (commandCharBuffer[0]=='0') channel = 0;
    else channel = -1;
    
    //Serial.print(channel);  
    
    int commaIndex0 = commandString.indexOf(',');
    int commaIndex1 = commandString.indexOf(',',commaIndex0+1);
    if (commaIndex0 > 0 && commaIndex1 > 1){
      String idString = commandString.substring(commaIndex0+1,commaIndex1);
      //Serial.print("idString = ");
      //Serial.println(idString);
      idString.toCharArray(IdCharBuffer,9);
      for (uint8_t i = 0; i < strlen(IdCharBuffer) ; i++){
        //Serial.print(IdCharBuffer[i]);
        if (isxdigit(IdCharBuffer[i])) goodID = true; 
        else { goodID = false; break; }
      }
      if (goodID){
        uint32_t tempID = strtoul(IdCharBuffer,&endptr,16);
        //Serial.print(" ");
        //Serial.print(tempID,HEX);
        if (  (tempID >> 11) > 0){
          txmsg.ext = 1;
          txmsg.id = (0x3FFFFFFF & tempID); //29 bit ID
        }
        else {
          txmsg.ext = 0;
          txmsg.id = (0x7FF & tempID); //11 bit ID
        }
        //Serial.print(" ");
        //Serial.print(txmsg.id,HEX);
      }
      else Serial.println("ERROR Invalid ID format");
      
      String dataString = commandString.substring(commaIndex1+1);
      
      dataString.toCharArray(dataCharBuffer,17);
      txmsg.len = strlen(dataCharBuffer)/2;
      //Serial.print(" ");
      //Serial.print(txmsg.len,HEX);
      // Serial.print(" ");
      for (uint8_t i = 0; i < txmsg.len*2 ; i++){
        if (isxdigit(dataCharBuffer[i])) goodData = true; 
        else { goodData = false; Serial.println("ERROR Non Hex Characters or Odd number of nibbles or ID is too long"); break; }
      } 
      if (goodData){
        for (int i = 0; i <  txmsg.len ; i++){
          char byteStringChars[3] = {dataCharBuffer[2*i],dataCharBuffer[2*i+1],0x00};
          txmsg.buf[i] = strtol(byteStringChars,&endptr,16);
          //Serial.print(txmsg.buf[i],HEX);
          //Serial.print(" ");
        }
      }
    }
    
    if (goodData && goodID ){
      if (channel == 0) {
        Can0.write(txmsg);
      }
      else if (channel == 1){
        Can1.write(txmsg);
      }
      else Serial.println("ERROR Invalid Channel for SM.");
    }
    
    else
      Serial.println(F("ERROR Invalid input data for SM. Input should be using hex characters with no spaces in the form SM,channel,ID,data/n"));
  }
  else
  {
    Serial.println(F("ERROR Missing or invalid data to send."));
  }
  txmsg.ext = 1; //set default
  txmsg.len = 8;
}



/*                 End Function calls for User input data                             */
/**************************************************************************************/

void sendComponentInfo()
{
  if (enableSendComponentInfo){
       char id[29];
       strncpy(id,componentID,29);
       
       Serial.print(F("INFO Received Request for Component ID. Sending  "));
       for (int i = 0; i<28;i++) Serial.print(id[i]);
       Serial.println();
       
       byte transport0[8] = {32,28,0,4,0xFF,0xEB,0xFE,0};
       byte transport1[8] = {1,id[0],id[1],id[2],id[3],id[4],id[5],id[6]};
       byte transport2[8] = {2,id[7],id[8],id[9],id[10],id[11],id[12],id[13]};
       byte transport3[8] = {3,id[14],id[15],id[16],id[17],id[18],id[19],id[20]};
       byte transport4[8] = {4,id[21],id[22],id[23],id[24],id[25],id[26],id[27]};
       txmsg.id = 0x1CECFFFA;
       txmsg.len = 8;
       memcpy(txmsg.buf,transport0,8);
       Can0.write(txmsg);
       delay(3);
       txmsg.id = 0x1CEBFFFA;
       memcpy(txmsg.buf,transport1,8);
       Can0.write(txmsg);
       memcpy(txmsg.buf,transport2,8);
       Can0.write(txmsg);
       memcpy(txmsg.buf,transport3,8);
       Can0.write(txmsg);
       memcpy(txmsg.buf,transport4,8);
       Can0.write(txmsg);
  }     
}       
void parseJ1939(CAN_message_t &rxmsg ){
  uint32_t ID = rxmsg.id;
  uint8_t DLC = rxmsg.len;
  uint8_t SA = (ID & 0xFF);
  uint8_t PF = (ID & 0x03FF0000) >> 16;
  uint8_t PRIORITY = (ID & 0x3C000000) >> 26;
  uint8_t DA = 0xFF;
  uint32_t PGN;
  if (PF >= 240){
     PGN = (ID & 0x03FFFF00) >> 8;
  }
  else{
    //pdu1 format
    PGN = (ID & 0x03FF0000) >> 8;
    DA = (ID & 0x00000FF00);
  }
  if (displayJ1939){
    char J1939Characters[30];
    sprintf(J1939Characters,"%d,%08ul,%d,%d,%d,",PRIORITY,PGN,DA,SA,DLC);
    Serial.print(J1939Characters);
    for (uint8_t i = 0; i<DLC; i++){
      sprintf(J1939Characters,"%02X,",rxmsg.buf[i]);
      Serial.print(J1939Characters);
    }
    Serial.println();
  }
  if (PGN == 0xEA00){
    //request message
    if (rxmsg.buf[0] == 0xEB && rxmsg.buf[1] == 0xFE){
      //Component ID 
      sendComponentInfo();
      //sendJ1939(0,6,0xFEEB,0xFF,sourceAddress,strlen(componentID),componentID);
    }
  }
  else if (PGN == 0xEB00){
    //Transport Protocol - Data
    
  }
  else if (PGN == 0xEC00){
    //Transport Protocol - Connection Management
    uint8_t controlByte = rxmsg.buf[0];
    if (controlByte == 16){
      //Connection Mode - Request to Send
      uint16_t totalMessageSize = rxmsg.buf[1] + rxmsg.buf[2]*256;
      uint8_t totalPackets = rxmsg.buf[3];
      uint8_t numPacketsToBeSent = rxmsg.buf[4];
      uint32_t requestedPGN = rxmsg.buf[5] + (rxmsg.buf[6] << 8) + (rxmsg.buf[7] << 16);
      //TODO: Set up a response
    }
    else if (controlByte == 17){
      //Connection Mode - Clear to Send 
      uint8_t numPacketsToBeSent = rxmsg.buf[1];
    }
    else {
      uint16_t totalMessageSize = rxmsg.buf[1] + rxmsg.buf[2]*256;
      uint8_t totalPackets = rxmsg.buf[3];
      uint8_t numPacketsToBeSent = rxmsg.buf[4];
      uint8_t maxNumOfPackets =rxmsg.buf[5];
      uint8_t nextPacketToBeSent =rxmsg.buf[6];
      uint8_t sequenceNumber =rxmsg.buf[7];
    }


    
    
    
  }
}

void sendJ1939(uint8_t channel, uint8_t priority, uint32_t pgn, uint8_t DA, uint8_t SA, int numBytes, char J1939Buffer[]){
  if (numBytes <= 8){
    if((pgn & 0xFF00) >= 240) txmsg.id = (priority << 26) + (pgn << 8) + SA;
    else txmsg.id = (priority << 26) + (pgn << 8) + (DA << 8) + SA;
    txmsg.len = numBytes;
    for (uint8_t i = 0; i<numBytes;i++) txmsg.buf[i]=J1939Buffer[i];
    if (channel == 0) Can0.write(txmsg);
    else if(channel == 1) Can1.write(txmsg);
    else Serial.println("0: J1939 Message not sent.");
  }
  else {
    //transport
    uint8_t numFrames = numBytes;
    transportTimer = 0;
    
  }
}


//A generic CAN Frame print function for the Serial terminal
void printFrame(CAN_message_t rxmsg, int mailbox, uint8_t channel, uint32_t RXCount)
{
  Serial.printf("CAN%d %10lu %10lu %5u %08X %d %d %02X %02X %02X %02X %02X %02X %02X %02X\n",
          channel,RXCount,micros(),rxmsg.timestamp,rxmsg.id,rxmsg.ext,rxmsg.len,
          rxmsg.buf[0],rxmsg.buf[1],rxmsg.buf[2],rxmsg.buf[3],
          rxmsg.buf[4],rxmsg.buf[5],rxmsg.buf[6],rxmsg.buf[7]);
}


time_t getTeensy3Time(){
  microsecondsPerSecond = 0;
  return Teensy3Clock.get();
}


void print_uid()  {  
  Serial.printf("ID: %s\n", kinetisUID());
}

void displayStats(){
  CAN_stats_t currentStats = Can0.getStats();
  printStats(currentStats,0);
  currentStats = Can1.getStats();
  printStats(currentStats,1);
}
 
void printStats(CAN_stats_t currentStats,int channel){
  Serial.printf("STATS for CAN%d: Enabled:%d, RingRXHighWater: %ul, ringRXFramesLost: %lu, ringTXHighWater: %lu, mailbox use count:[%lu",
                            channel,currentStats.enabled,currentStats.ringRxHighWater,currentStats.ringRxFramesLost,currentStats.ringTxHighWater,currentStats.mb[0].refCount);
  for (int i=1;i<16;i++){
    Serial.printf(", %lu",currentStats.mb[i].refCount); 
  }
  Serial.printf("], overrunCount: [%lu",currentStats.mb[0].overrunCount);
  for (int i=1; i<16; i++){
    Serial.printf(", %lu",currentStats.mb[i].overrunCount); 
  }
  Serial.println("]");
}

void clearStats(){
  Can0.clearStats();
  Can1.clearStats();
  
  Can0.startStats();
  Can1.startStats();
  
  Serial.println("INFO Cleared CAN Statisitics");
}
boolean configed=false;
    
void setup() {
  Serial.begin(9600);
  Serial1.begin(19200);
  
  kinetisUID(uid);
  
  analogWriteResolution(12);
  
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
  

  Can0.begin(BAUDRATE0);
  Can1.begin(BAUDRATE1);

  
  
  
  Serial.print(F("Setting CAN Filters..."));
  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    Can1.setFilter(allPassFilter,filterNum); 
  }

  txmsg.ext = 1;
  txmsg.len = 8;

  setConfigSwitches();
    
  Serial1.begin(19200);
  Serial.println(F("Started LIN at 19200."));
  
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
  
  Serial.println(F("Finished Starting Up... Type a command:"));
 
  k=0;
  firstLINtime =true;
  Serial1.flush();
  Serial1.clear();
  
  Can0.startStats();
  Can1.startStats();
}

void loop() {

  while (Can0.available()) {
    Can0.read(rxmsg);
    RXCount0++;
    if (displayCAN0) printFrame(rxmsg, -1, 0, RXCount0);
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);
  }
  while (Can1.available()) {
    Can1.read(rxmsg);
    RXCount1++;
    if (displayCAN1) printFrame(rxmsg, -1, 1, RXCount1);
    if (ignitionCtlState){
      greenLEDstate = !greenLEDstate;
      digitalWrite(greenLEDpin, greenLEDstate);
    }
  }
  

  /************************************************************************/
  /*            Begin PERIODIC CAN Message Transmission                            */
  if (A21TX_Timer >=100){
    A21TX_Timer=0;
    if (sendA21voltage) displayVoltage();
  }
  
  if (ignitionCtlState && TXCAN){
     if (CANTX_10ms_timer >= 10){
        CANTX_10ms_timer = 0;
       
        //signal 32
        if (send0CF00203){
          memcpy(txmsg.buf,blankCANdata,8);
          txmsg.id = 0x0CF00203; //ETC1
          Can1.write(txmsg);
        }
        
        //signal 1
        if (send08FF0001){
          memcpy(txmsg.buf,blankCANdata,8);
          txmsg.id = 0x08FF0001; //MCM ID
          Can1.write(txmsg);
        }
        
        //signal 2 
        if (send08FF0003){
          ff0003Data[6]=ff0003sixthByte[index_ff0003];
          ff0003Data[7]=ff0003sevenByte[index_ff0003];
          memcpy(txmsg.buf,ff0003Data,8);
          txmsg.id = 0x08FF0003; //TCM System ID Message ID
          Can1.write(txmsg);
          index_ff0003 += 1;
          if (index_ff0003 >= 16) index_ff0003=0;
        } 

        //signal 3
        //J1939 Transmission Output Shaft Speed Signal which may correspond to PGN 0x00F002
        if (send08FF0103) {
          ff0103Data[6]=ff0103sixthByte[index_ff0103];
          ff0103Data[7]=ff0103sevenByte[index_ff0103];
          memcpy(txmsg.buf,ff0103Data,8);
          txmsg.id = 0x08FF0103; 
          Can1.write(txmsg);
          index_ff0103 += 1;
          if (index_ff0103 >= 16) index_ff0103=0;
        }

        //signal 4
        if (send08FF0203) { //default off
          ff0203Data[6]=ff0203sixthByte[index_ff0203];
          ff0203Data[7]=ff0203sevenByte[index_ff0203];
          memcpy(txmsg.buf,ff0203Data,8);
          txmsg.id = 0x08FF0203; 
          Can1.write(txmsg);
          index_ff0203 += 1;
          if (index_ff0203 >= 16) index_ff0203 = 0;
        }

        //signal 5
        if (send08FF0303){
          //No Changing data for TCM message
          memcpy(txmsg.buf,ff0303Data,8);
          txmsg.id = 0x08FF0303; 
          Can1.write(txmsg);
        }

        //signal 6
        if (send08FF0603){
          //TCM transport layer Message
          ff0603Data[6]=ff0603sixthByte[index_ff0603];
          ff0603Data[7]=ff0603sevenByte[index_ff0603];
          memcpy(txmsg.buf,ff0603Data,8);
          txmsg.id = 0x08FF0603; 
          Can1.write(txmsg);
          index_ff0603 += 1;
          if (index_ff0603 >= 16) index_ff0603=0;
        }

        //signal 7
        if (send08FF0703){
        //TCM transport layer Message
          ff0703Data[6]=ff0703sixthByte[index_ff0703];
          ff0703Data[7]=ff0703sevenByte[index_ff0703];
          memcpy(txmsg.buf,ff0703Data,8);
          txmsg.id = 0x08FF0703; 
          Can1.write(txmsg);
          index_ff0703 += 1;
          if (index_ff0703 >= 16) index_ff0703=0;
        }
    
     }
     if (CANTX_20ms_timer >= 20){
        CANTX_20ms_timer = 0;

        //signal 8
        if (send0CFF0703){
          if (c59timer >= 2000) {
            c59timer = 0;
            Cff0703Data[5] = 0x5A;
          }
          else {
            Cff0703Data[5] = 0x59;  
          }
          if (c5Btimer >= 3000) {
            c5Btimer = 0;
            Cff0703Data[5] = 0x5B;
          }
          else {
            Cff0703Data[5] = 0x59;  
          }
          //TCM transport layer Message with TCM System ID
          Cff0703Data[6]=Cff0703sixthByte[index_Cff0703];
          Cff0703Data[7]=Cff0703sevenByte[index_Cff0703];
          memcpy(txmsg.buf,Cff0703Data,8);
          txmsg.id = 0xCFF0703; //
          Can1.write(txmsg);
          index_Cff0703 += 1;
          if (index_Cff0703 >= 16) index_Cff0703=0;
        }
        
        //signal 9
        if (send0CFE6E0B){
          memcpy(txmsg.buf,blankCANdata,8);
          txmsg.id = 0x0CFE6E0B; //High Resolution wheel speed message from SA=11 (brake controller)
          Can0.write(txmsg);
        }
     }
     
     if (CANTX_50ms_timer >= 50){
       CANTX_50ms_timer = 0;

       //signal 10
       if (send10FF0903){
         ff0903Data[6]=ff0903sixthByte[index_ff0903];
         ff0903Data[7]=ff0903sevenByte[index_ff0903];
         memcpy(txmsg.buf,fef803Data,8);
         txmsg.id = 0x10FF0903; //
         Can1.write(txmsg);
         index_ff0903 += 1;
         if (index_ff0903 >= 2) index_ff0903 = 0;
       }  


        //signal 30
       if (send18DF00F9 && DM13_00_Count < 8){
         memcpy(txmsg.buf,DF00F9,8);
         txmsg.id = 0x18DF00F9; //ACM
         Can0.write(txmsg);
         DM13_00_Count++;
       }
       if (send18DFFFF9  && DM13_FF_Count < 8){
         memcpy(txmsg.buf,DFFFF9,8);
         txmsg.id = 0x18DFFFF9; //ACM
         Can0.write(txmsg);
         DM13_FF_Count++;
       }
     }
     
     if (CANTX_100ms_timer >= 100){
       CANTX_100ms_timer = 0;
       
     
      
       memcpy(txmsg.buf,blankCANdata,8);
       //signal 11
       txmsg.id = 0x18F00131; // Electronic Brake Controller from SA=49
       if (send18F00131) Can0.write(txmsg);
       //signal 12
       txmsg.id = 0x18F0010B; // Electronic Brake Controller from SA=11
       if (send18F0010B) Can0.write(txmsg);
       //signal 13
       txmsg.id = 0x18FEF117; //CCVS from SA=23
       if (send18FEF117) Can0.write(txmsg);
       //signal 14
       txmsg.id = 0x18FEF128; //CCVS from SA=40
       if (send18FEF128) Can0.write(txmsg);
       //signal 15
       txmsg.id = 0x18FEF121; //CCVS from SA=40
       if (send18FEF121) Can0.write(txmsg);
       //signal 16
       txmsg.id = 0x18FEF131; //CCVS from SA=49
       if (send18FEF131) Can0.write(txmsg);
       //signal 17
       txmsg.id = 0x18E00017; //Cab Message 1 from SA=23
       if (send18E00017) Can0.write(txmsg);
       //signal 18
       txmsg.id = 0x18E00019; //Cab Message 1 from SA=25
       if (send18E00019) Can0.write(txmsg);
       //signal 19
       txmsg.id = 0x18E00021; //Cab Message 1 from SA=33
       if (send18E00021) Can0.write(txmsg);
       //signal 20
       txmsg.id = 0x18E00028; //Cab Message 1 from SA=40
       if (send18E00028) Can0.write(txmsg);
       txmsg.id = 0x18E00028; //Cab Message 1 from SA=40
       //signal 21
       txmsg.id = 0x18E00031; //Cab Message 1 from SA=40
       if (send18E00031) Can0.write(txmsg);
       //signal 26
       txmsg.id = 0x18FEF017; //PTO message from SA=23
       if (send18FEF017) Can0.write(txmsg);
       //signal 27
       txmsg.id = 0x18FEF021; //PTO message from SA=33
       if (send18FEF021) Can0.write(txmsg);
       //signal 28
       txmsg.id = 0x18FEF028; //PTO message from SA=40
       if (send18FEF028) Can0.write(txmsg);
       //signal 29
       txmsg.id = 0x18FEF031; //Cab Message 1 from SA=49
       if (send18FEF031) Can0.write(txmsg);
       //signal 33
       txmsg.id = 0x18F00503; //ETC2
       if (send18F00503) Can1.write(txmsg);
        
        
     }
     
     if (CANTX_200ms_timer >= 200){
       CANTX_200ms_timer = 0; 
     }
     if (CANTX_250ms_timer >= 250){
       CANTX_250ms_timer = 0; 
     }
     if (CANTX_500ms_timer >= 500){
       CANTX_500ms_timer = 0; 
      
       
     }
     if (CANTX_1000ms_timer >= 1000){
       CANTX_1000ms_timer = 0; 
       CANTX_1005ms_timer = 0; 

       //signal 22
       if (send10ECFF01){
         memcpy(txmsg.buf,sessionControlMessage,8);
         txmsg.id = 0x10ECFF01; //MCM
         Can1.write(txmsg);
       }

       //signal 23
       if (send10ECFF3D){
         memcpy(txmsg.buf,sessionControlMessage,8);
         txmsg.id = 0x10ECFF3D; //ACM
         Can1.write(txmsg);
       }

       //signal 24
       if (send18FEF803){
         memcpy(txmsg.buf,fef803Data,8);
         txmsg.id = 0x18FEF803; //Transmission Oil Temperature
         Can1.write(txmsg);
       }

       //signal 25
       if (send18FEF521){
         memcpy(txmsg.buf,blankCANdata,8);
         txmsg.id = 0x18FEF521; //PGN 65269 Ambient Conditions
         Can0.write(txmsg);
       }

         
     }
     if (CANTX_1005ms_timer >= 1005){
       if (send10ECFF01) {
         memcpy(txmsg.buf,sessionTransportMessage,8);
         txmsg.id = 0x10ECFF01; //MCM
         Can1.write(txmsg);
       }
       
       if (send10ECFF3D){
         txmsg.id = 0x10ECFF3D; //ACM
         Can1.write(txmsg);
       }
     }
     
     if (CANTX_5000ms_timer >= 5000){
       CANTX_5000ms_timer = 0; 
       txmsg.buf[0]=0xFF;
       txmsg.buf[1]=0xFF;
       txmsg.buf[2]=0xFF;
       txmsg.buf[3]=0x0F;
       txmsg.buf[4]=0xFF;
       txmsg.buf[5]=0xFF;
       txmsg.buf[6]=0xFF;
       txmsg.buf[7]=0xFF;
        //signal 30
       if (send18DF00F9 && DM13_00_Count == 8){
         txmsg.id = 0x18DF00F9; //DM13 Hold
         Can0.write(txmsg);
       }
       if (send18DFFFF9  && DM13_FF_Count == 8){
         txmsg.id = 0x18DFFFF9; //ACM
         Can0.write(txmsg);
       }
       
     }
     if (CANTX_10000ms_timer >= 10000){
       CANTX_10000ms_timer = 0; 
     }
     if (CANTX_30000ms_timer >= 30000){
       CANTX_30000ms_timer = 0;
     }
  }
  /*             End CAN Message Transmission                              */
  /*************************************************************************/
  
    
  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 140) {
    commandChars = Serial.readStringUntil(',');
    if (Serial.available()) commandString = Serial.readStringUntil('\n');
    else commandString = "";
    if      (commandChars.toInt() > 0) fastSetSetting();  
    else if (commandChars.startsWith("SM") || commandChars.startsWith("sm")) sendMessage();
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
    else if (commandChars.startsWith("DS") || commandChars.startsWith("ds")) setDisplayCAN();
    else if (commandChars.startsWith("B0") || commandChars.startsWith("b0")) autoBaud0();
    else if (commandChars.startsWith("B1") || commandChars.startsWith("b1")) autoBaud1();
    else if (commandChars.startsWith("DB") || commandChars.startsWith("db")) displayBaud();
    else if (commandChars.startsWith("CN") || commandChars.startsWith("cn")) startStopCAN();
    else if (commandChars.startsWith("C0") || commandChars.startsWith("c0")) startStopCAN0Streaming();
    else if (commandChars.startsWith("C1") || commandChars.startsWith("c1")) startStopCAN1Streaming();
    else if (commandChars.startsWith("C2") || commandChars.startsWith("c2")) startStopCAN2Streaming();
    else if (commandChars.startsWith("DJ") || commandChars.startsWith("dj")) displayJ1939 = !displayJ1939;
    else if (commandChars.startsWith("AI") || commandChars.startsWith("ai")) displayVoltage();
    else if (commandChars.startsWith("MK") || commandChars.startsWith("mk")) setEnableComponentInfo();
    else if (commandChars.startsWith("ID") || commandChars.startsWith("id")) print_uid();
    else if (commandChars.startsWith("SV") || commandChars.startsWith("sv")) streamVoltage();
    else if (commandChars.startsWith("OK") || commandChars.startsWith("ok")) checkAgainstUID();
    else if (commandChars.startsWith("ST") || commandChars.startsWith("st")) displayStats();
    else if (commandChars.startsWith("CL") || commandChars.startsWith("cl")) clearStats();

   
    
    else Serial.println(F("ERROR Unrecognized Command Characters. Use a comma after the command.\nERROR Known commands are CN, B0, B1, DS, VI, SW, PN, PD, PB, PF, LI, LS, CI, CS, AF, AO, SA, SC, SS, or SM."));
  
  }
  Serial.clear();
  Serial.flush();
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


  /****************************************************************/
  /*           Begin LED Indicators for messages                  */
  /*
  /*Reset the greenLED after a timeout in case the last CAN message was on the wrong state*/
  if (RXCAN0timer >= 200) { 
    RXCAN0timer = 0;
    redLEDstate = true;
    digitalWrite(redLEDpin, redLEDstate); //Use red because it is the power button.
  }
  
  /*Reset the greenLED after a timeout in case the last CAN message was on the wrong state*/
  if (RXCAN1orJ1708timer >= 200) { 
    RXCAN1orJ1708timer = 0;
    if (ignitionCtlState) greenLEDstate = true;
    else greenLEDstate = false;
    digitalWrite(greenLEDpin, greenLEDstate); 
  }
  /*             End LED Indicators for messages                        */
  /**********************************************************************/

  /**********************************************************************/
  /*            Begin LIN for Shifter                                   */
  currentMicros = micros();
  if (Serial1.available()>=3) 
  {
    byte firstChar = Serial1.read();
    byte secondChar = Serial1.read() ;
    byte thirdChar = Serial1.read() ;
    if (firstChar == 0x00 && secondChar == 0xF0){
      Serial.println("LIN Start");
      Serial1.write(0xF0);
    }
    else if (firstChar == 0x00 && secondChar == 0x55 && thirdChar == 0x20 && firstLINtime){
        firstLINtime =false;
         Serial1.write(0xFF);
         Serial1.write(0xF);
         Serial1.write(0xFF);
         Serial1.write(0x3F);
         Serial1.write(0x91);
         Serial.println("first LIN Pass");
    }
    else if (firstChar == 0x00 && secondChar == 0x55 && thirdChar == 0x20 && !firstLINtime)//Serial.print(micros());
    {
      outByte[0] = 0x14;
      outByte[1] = (k << 4) + 0x01;
      k+=1;
      outByte[2] = 0x02;
      outByte[3] = 0x0D;
      
      if      (outByte[1]==0x01) outByte[4] =0xBB;
      else if (outByte[1]==0x11) outByte[4] =0xAB;
      else if (outByte[1]==0x21) outByte[4] =0x9B;
      else if (outByte[1]==0x31) outByte[4] =0x8B;
      else if (outByte[1]==0x41) outByte[4] =0x7B;
      else if (outByte[1]==0x51) outByte[4] =0x6B;
      else if (outByte[1]==0x61) outByte[4] =0x5B;
      else if (outByte[1]==0x71) outByte[4] =0x4B;
      else if (outByte[1]==0x81) outByte[4] =0x3B;
      else if (outByte[1]==0x91) outByte[4] =0x2B;
      else if (outByte[1]==0xA1) outByte[4] =0x1B;
      else if (outByte[1]==0xB1) outByte[4] =0x0B;
      else if (outByte[1]==0xC1) outByte[4] =0xFA;
      else if (outByte[1]==0xD1) outByte[4] =0xEA;
      else if (outByte[1]==0xE1) outByte[4] =0xDA;
      else if (outByte[1]==0xF1) outByte[4] =0xCA;

      serialSend0Micros = currentMicros+500;
      LIN0send = true;
      serialSend1Micros = currentMicros+1000;
      LIN1send = true;
      serialSend2Micros = currentMicros+1500;
      LIN2send = true;
      serialSend3Micros = currentMicros+2000;
      LIN3send = true;
      serialSend4Micros = currentMicros+2500;
      LIN4send = true;
      

    }
  }    

  if (currentMicros - serialSend0Micros >=0 && LIN0send) {
    Serial1.write(outByte[0]);
    LIN0send=false;
  }
  if (currentMicros - serialSend1Micros >=0 && LIN1send) {
    Serial1.write(outByte[1]);
    LIN1send=false;
  }
  if (currentMicros - serialSend2Micros >=0 && LIN2send) {
    Serial1.write(outByte[2]);
    LIN2send=false;
  }
  if (currentMicros - serialSend3Micros >=0 && LIN3send){
    Serial1.write(outByte[3]);
    LIN3send=false;
  }
  if (currentMicros - serialSend4Micros >=0 && LIN4send) {
    Serial1.write(outByte[4]);
    LIN4send=false;
  }

  
}

