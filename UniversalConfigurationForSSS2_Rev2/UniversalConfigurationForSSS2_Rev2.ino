/*
   Smart Sensor Simulator 2
   Controlling the  Quadtrature Knob, Ignition Relay, and Voltage Regulator
   Hardware Revision 2

   Written by Dr. Jeremy S. Daily
   Synercon Technologies, LLC
   
   Proprietary Software - Not for Public Release

   Copyright (c) 2017        Jeremy S. Daily
*/

//softwareVersion
char softwareVersion[200] = "SSS2*Rev2*0.4*ec644a7e5406124655f49ae9d5e27038f4b450b5"; //Hash of the previous git commit
char componentID[200] = "SYNER*SSS2-R02*0000*UNIVERSAL"; //Add the serial number for hard coded values.

byte sourceAddress = 0xFA; 


#include <SPI.h>
#include <i2c_t3.h>
#include <EEPROM.h>
#include "OneButton.h"
#include <FlexCAN.h>

/**********************************************************************/
/*  Begin Definitions for Digital Potentiometer Terminal Connections  */
#define TCON_B_ONLY       1
#define TCON_WIPER_ONLY   2
#define TCON_WIPER_AND_B  3
#define TCON_A_ONLY       4
#define TCON_A_AND_B      5
#define TCON_WIPER_AND_A  6
#define TCON_CONNECT_ALL  7

#define IODIRA 0x00
#define IODIRB 0x10
#define IOCONA 0x0A
#define IOCONB 0x0B
#define OLATA  0x0A
#define OLATB  0x1A

//const uint8_t ExtenderOpCode = 0b01000110;
const uint8_t ExtenderOpCode = 0b01001110;

uint8_t bitPositions[8] = { ~0b00000001,
                            ~0b00000010,
                            ~0b00000100,
                            ~0b00001000,
                            ~0b00010000,
                            ~0b00100000,
                            ~0b01000000,
                            ~0b10000000
                          };
/* End Definitions for Digital Potentiometer Terminal Connections  */
/*******************************************************************/


//Variables that get loaded from contents in the EEPROM
const uint16_t potWiperSettingsAddress = 0;
uint8_t potWiperSettings[16] ={16,32,48,64,80,96,112,128,144,160,176,192,208,224,240,255};
const uint16_t potTCONSettingsAddress = 16;
uint8_t potTCONSettings[16] ={7,7,7,7,7,7,7,7,7,7,0,0,7,7,7,7};
const uint16_t DAC2valueAddress = 32;
uint16_t DAC2value[8] = {500,1000,1500,2000,2500,3000,3500,4000};
const uint16_t pwm1valueAddress = 48;
uint8_t pwm1value = 50;
const uint16_t pwm2valueAddress = 49;
uint8_t pwm2value = 100;
const uint16_t pwm3valueAddress = 50;
uint8_t pwm3value = 150;
const uint16_t pwm4valueAddress = 51;
uint8_t pwm4value = 200;
const uint16_t HVoutAdjAddress = 52;
uint8_t HVoutAdjValue = 168;
const uint16_t terminationSettingsAddress = 54;
uint8_t terminationSettings = 0xFF; //0b11111111;
const uint16_t connectionSettingsAddress = 55;
uint8_t connectionSettings = 0b000000000 ;
const uint16_t HS1Address = 56;
uint8_t HS1state = 0;
const uint16_t HS2Address = 57;
uint8_t HS2state = 1;
const uint16_t LS1Address = 58;
uint8_t LS1state = 0;
const uint16_t LS2Address = 59;
uint8_t LS2state = 1;
const uint16_t DAC3valueAddress = 61;
uint16_t DAC3value[8] {500,1000,1500,2000,2500,3000,3500,4000}; 



const uint16_t componentIDAddress = 1000;

const uint16_t programmedForAddress = 1200;
char programmedFor[200] = "UNIVERSAL COMMUNICATIONS";

const uint16_t programmedByAddress = 1400;
char programmedBy[200] = "J. DAILY";

const uint16_t programmedDateAddress = 1600;
char programDate[200] = "01 Jan 2016";

const uint16_t softwareVersionAddress = 1800;

const uint16_t programNotesAddress = 2000;
char programNotes[1000] = "Notes: ";

const uint16_t vehicleIdentificationNumAddress = 3000;
char vehicleIdentificationNum[20] = "A Fake VIN";

//const uint16_t number_of_CAN_msgsAddress = 256;
//uint8_t number_of_CAN_msgs = 1;
//
//struct CANTXmessage {
//  uint8_t  period_index = 2; //in milliseconds
//  uint8_t  channel = 0; //0=CAN0, 1=CAN1, or 2=CAN2
//  uint8_t DLC = 8;
//  uint32_t ID = 0x18FEF117; //assume extended, unless less than 0x7FF
//  uint8_t dataField[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//};
//
//struct CANTXmessage CANTXmessages[256];


/****************************************************************/
/*                         Pin Defintions                       */
const int greenLEDpin       = 2;
const int redLEDpin         = 5;
const int CSdispPin         = 9;
const int CSpotsPin         = 15;
const int CSanalogPin       = 31;
const int CShvadjPin        = 55;
const int CStermPin         = 41;//21; //41;
const int CSVoutPin         = 42;//26; //42;
const int CSCANPin          = 54;
const int ignitionCtlPin    = 53;//20; //53;
const int buttonPin         = 24;
const int pwm1              = 16;
const int pwm2              = 17;
const int pwm3              = 22;
const int pwm4              = 23;
const int IH1               = 35;
const int IH2               = 36;
const int IL1               = 37;
const int IL2               = 38;


//i2C Device Addresses
const uint8_t Vout2address = 0x49;
const uint8_t Vout3address = 0x4B;

/****************************************************************/
/*                    Quadrature Knob Setup                     */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
Encoder knob(28, 25);
int32_t currentKnob = 0;

OneButton button(buttonPin, true);



//set up a display buffer
char displayBuffer[100];

/****************************************************************/
/*              Setup millisecond timers and intervals          */

#define LEDtoggleTime 500//Set up message transmitters
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


/****************************************************************/
/*                    CAN Setup                                 */


//Set up the CAN data structures
static CAN_message_t rxmsg, txmsg;

//set up a counter for each received message
uint32_t RXCAN0count = 0;
uint32_t RXCAN1count = 0;

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

/****************************************************************/
/*                 Binary State Variables                       */
//Keep track of the current state of the LEDs
boolean greenLEDstate = false;
boolean redLEDstate = true;
boolean LEDstate = true;
boolean ignitionCtlState = false;
boolean U1andU2POA;
boolean U3andU4POA;
boolean U5andU6POA;
boolean U7andU8POA;
boolean CAN0term = true;
boolean CAN1term = true;
boolean CAN2term = true;
boolean LINmaster;
boolean PWM1Connect;
boolean PWM2Connect;
boolean Vout3CConnect;
boolean Vout3DConnect;
boolean LINSHLDConnect;
boolean LIN16Connect;
boolean ADJUST_MODE_ON  = 0;
boolean SAFE_TO_ADJUST = 0;
boolean displayCAN = 0;
boolean CAN0baudNotDetected = true;
boolean CAN1baudNotDetected = true;
boolean TXCAN = true;
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

uint16_t currentSetting = 0;

int knobLowLimit = 0;
int knobHighLimit = 255;
int knobJump = 1;

/********************************************************************/
/*               Begin Define Settings                              */
/*
 * To add another setting, update the numSettings, populate the settingNames and settingPins array and add 
 * new knob limits. The knob limits are used for value checking. These are for numerical Settings.
 */


#define numSettings  72
char settingNames[numSettings][40] = {
  "Nothing Selected",
  "Digital Potentiometer  1 Wiper",
  "Digital Potentiometer  2 Wiper",
  "Digital Potentiometer  3 Wiper",
  "Digital Potentiometer  4 Wiper",
  "Digital Potentiometer  5 Wiper",
  "Digital Potentiometer  6 Wiper",
  "Digital Potentiometer  7 Wiper",
  "Digital Potentiometer  8 Wiper",
  "Digital Potentiometer  9 Wiper",
  "Digital Potentiometer 10 Wiper",
  "Digital Potentiometer 11 Wiper",
  "Digital Potentiometer 12 Wiper",
  "Digital Potentiometer 13 Wiper",
  "Digital Potentiometer 14 Wiper",
  "Digital Potentiometer 15 Wiper",
  "Digital Potentiometer 16 Wiper",

  "Vout2-A",
  "Vout2-B",
  "Vout2-C",
  "Vout2-D",
  "Vout2-E",
  "Vout2-F",
  "Vout2-G",
  "Vout2-H",

  "Vout3-A",
  "Vout3-B",
  "Vout3-C",
  "Vout3-D",
  "Vout3-E",
  "Vout3-F",
  "Vout3-G",
  "Vout3-H",

  "PWM 1",
  "PWM 2",
  "PWM 3",
  "PWM 4",

  "U1andU2POA to 12V",
  "U3andU4POA to 12V",
  "U5andU6POA to 12V",
  "U7andU8POA to 12V",
  "CAN0 Termination Resistor",
  "CAN1 Termination Resistor",
  "CAN2 Termination Resistor",
  "LIN Master Pullup Resistor",

  "12V Out 1",
  "12V Out 2",
  "Ground Out 1",
  "Ground Out 2",

  "HVoutAdj",

  "Dig. Pot.  1 Terminal Connect",
  "Dig. Pot.  2 Terminal Connect",
  "Dig. Pot.  3 Terminal Connect",
  "Dig. Pot.  4 Terminal Connect",
  "Dig. Pot.  5 Terminal Connect",
  "Dig. Pot.  6 Terminal Connect",
  "Dig. Pot.  7 Terminal Connect",
  "Dig. Pot.  8 Terminal Connect",
  "Dig. Pot.  9 Terminal Connect",
  "Dig. Pot. 10 Terminal Connect",
  "Dig. Pot. 11 Terminal Connect",
  "Dig. Pot. 12 Terminal Connect",
  "Dig. Pot. 13 Terminal Connect",
  "Dig. Pot. 14 Terminal Connect",
  "Dig. Pot. 15 Terminal Connect",
  "Dig. Pot. 16 Terminal Connect",

  "PWM1 Connect",
  "PWM2 Connect",
  "Vout 3-C Connect",
  "Vout 3-D Connect",
  "LIN to Shield Connect",
  "LIN to Port 16 Connect"
};

char settingPins[numSettings][40] = {
  "",
  "Port  1 (J24- 1)",
  "Port  2 (J24- 2)",
  "Port  3 (J24- 3)",
  "Port  4 (J24- 4)",
  "Port  5 (J24- 5)",
  "Port  6 (J24- 6)",
  "Port  7 (J24- 7)",
  "Port  8 (J24- 8)",
  "Port  9 (J24- 9)",
  "Port 10 (J24-10)",
  "Port 11 (J24-11)",
  "Port 12 (J24-12)",
  "Port 13 (J18-11)",
  "Port 14 (J18-12)",
  "Port 15 (J24-15)",
  "Port 16 (J24-16)",

  "Port 18 (J18- 2)",
  "Port 19 (J18- 3)",
  "Port 20 (J18- 4)",
  "Port 21 (J18- 5)",
  "Port 22 (J18- 6)",
  "Port 23 (J18- 7)",
  "Port 24 (J18- 8)",
  "Port 25 (J18- 9)",

  "Port 29 (J18-13)",
  "Port 30 (J18-14)",
  "Port 31 (J18-15)",
  "Port 32 (J18-16)",
  "Ports  9 and 10",
  "Ports 11 and 12",
  "Ports 13 and 14",
  "Ports 15 and 16",

  
  "Port 17 (J18- 1)",
  "Port 26 (J18-10)",
  "Port 27 (J24-13)",
  "Port 28 (J24-14)",
  
  "Ports 1 and 2",
  "Ports 3 and 4",
  "Ports 5 and 6",
  "Ports 7 and 8",
  "R44",
  "R45",
  "R46",
  "R47",

  "Port 26 (J18-10)",
  "Port 11 (J24-11)",
  "Port 17 (J18- 1)",
  "Port 12 (J24-12)",

  "(J24-19)",

  "Port  1 (J24- 1)",
  "Port  2 (J24- 2)",
  "Port  3 (J24- 3)",
  "Port  4 (J24- 4)",
  "Port  5 (J24- 5)",
  "Port  6 (J24- 6)",
  "Port  7 (J24- 7)",
  "Port  8 (J24- 8)",
  "Port  9 (J24- 9)",
  "Port 10 (J24-10)",
  "Port 11 (J24-11)",
  "Port 12 (J24-12)",
  "Port 13 (J18-11)",
  "Port 14 (J18-12)",
  "Port 15 (J24-15)",
  "Port 16 (J24-16)",

  "Port 17 (J18- 1)",
  "Port 26 (J18-10)",
  "Port 31 (J18-15)",
  "Port 32 (J18-16)",
  "(J10- 5)",
  "Port 16 (J24-16)"

};


void setLimits(uint8_t settingNum) {
  if (settingNum > 0 && settingNum <= 16) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum > 16 && settingNum <= 24) {
    knobLowLimit = 0;
    knobHighLimit = 4095;
    knobJump = 20;
  }
  else if (settingNum > 24 && settingNum <= 32) {
    knobLowLimit = 0;
    knobHighLimit = 4095;
    knobJump = 20;
  }
  else if (settingNum == 33) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum == 34) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum == 35) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum == 36) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum == 37) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 38) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 39) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 40) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 41) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 42) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 43) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 44) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 45) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 46) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 47) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 48) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 49) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum >= 50 && settingNum <= 65) {
    knobLowLimit = 0;
    knobHighLimit = 7;
    knobJump = 1;
  }
  else if (settingNum == 66) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 67) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 68) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 69) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 70) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 71) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 72) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  
  else {
    knobLowLimit = 1;
    knobHighLimit = numSettings-1;
    knobJump = 1;
  }

  Serial.print("Low Limit: ");
  Serial.println(knobLowLimit);
  Serial.print("High Limit: ");
  Serial.println(knobHighLimit);
}


/*               End Define Settings                              */
/******************************************************************/


void connectionString(boolean switchState) {
  //memset(displayBuffer,0,sizeof(displayBuffer));
  Serial.print(switchState);
  if (switchState) strcpy(displayBuffer, " Connected");
  else strcpy(displayBuffer, " Open");
}

void terminalString(uint8_t setting) {
  switch (setting) {
    case TCON_B_ONLY:
      strcpy(displayBuffer, "TCON_B_ONLY");
      break;
    case TCON_WIPER_ONLY:
      strcpy(displayBuffer, "TCON_WIPER_ONLY");
      break;
    case TCON_WIPER_AND_B:
      strcpy(displayBuffer, "TCON_WIPER_AND_B");
      break;
    case TCON_A_ONLY:
      strcpy(displayBuffer, "TCON_A_ONLY");
      break;
    case TCON_A_AND_B :
      strcpy(displayBuffer, "TCON_A_AND_B ");
      break;
    case TCON_WIPER_AND_A :
      strcpy(displayBuffer, "TCON_WIPER_AND_A ");
      break;
    case TCON_CONNECT_ALL:
      strcpy(displayBuffer, "TCON_CONNECT_ALL");
      break;
    default:
      strcpy(displayBuffer, "Nothing connected");
      break;
  }
}

void listSetting(uint8_t settingNum) {
  Serial.print(settingNum);
  Serial.print(", ");
  Serial.print(settingNames[settingNum]);
  Serial.print(", ");
  Serial.print(settingPins[settingNum]);
  Serial.print(" = ");
  if (settingNum > 0 && settingNum <= 16)  Serial.println(potWiperSettings[settingNum - 1]);
  else if (settingNum > 16 && settingNum <= 24) Serial.println(DAC2value[settingNum - 17]);
  else if (settingNum > 24 && settingNum <= 32) Serial.println(DAC3value[settingNum - 25]);
  else if (settingNum == 33) Serial.println(pwm1value);
  else if (settingNum == 34) Serial.println(pwm2value);
  else if (settingNum == 35) Serial.println(pwm3value);
  else if (settingNum == 36) Serial.println(pwm4value);
  else if (settingNum == 37) {
    connectionString(U1andU2POA);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 38) {
    connectionString(U3andU4POA);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 39) {
    connectionString(U5andU6POA);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 40) {
    connectionString(U7andU8POA);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 41) {
    connectionString(CAN0term);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 42) {
    connectionString(CAN1term);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 43) {
    connectionString(CAN2term);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 44) {
    connectionString(LINmaster);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 45) {
    connectionString(HS1state);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 46) {
    connectionString(HS2state);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 47) {
    connectionString(LS1state);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 48) {
    connectionString(LS2state);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 49) Serial.println(HVoutAdjValue);
  else if (settingNum >= 50 && settingNum <= 65) {
    terminalString(potTCONSettings[settingNum - 50]);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 66) {
    connectionString(PWM1Connect);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 67) {
    connectionString(PWM2Connect);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 68) {
    connectionString(Vout3CConnect);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 69) {
    connectionString(Vout3DConnect);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 70) {
    connectionString(LINSHLDConnect);
    Serial.println(displayBuffer);
  }
  else if (settingNum == 71) {
    connectionString(LIN16Connect);
    Serial.println(displayBuffer);
  }
  else Serial.println();
}

uint16_t getSetting(uint8_t settingNum) {
  if      (settingNum > 0 && settingNum <= 16)  return potWiperSettings[settingNum - 1];
  else if (settingNum > 16 && settingNum <= 24) return DAC2value[settingNum - 17];
  else if (settingNum > 24 && settingNum <= 32) return DAC3value[settingNum - 25];
  else if (settingNum == 33) return pwm1value;
  else if (settingNum == 34) return pwm2value;
  else if (settingNum == 35) return pwm3value;
  else if (settingNum == 36) return pwm4value;
  else if (settingNum == 37) return U1andU2POA;
  else if (settingNum == 38) return U3andU4POA;
  else if (settingNum == 39) return U5andU6POA;
  else if (settingNum == 40) return U7andU8POA;
  else if (settingNum == 41) return CAN0term;
  else if (settingNum == 42) return CAN1term;
  else if (settingNum == 43) return CAN2term;
  else if (settingNum == 44) return LINmaster;
  else if (settingNum == 45) return HS1state;
  else if (settingNum == 46) return HS2state;
  else if (settingNum == 47) return LS1state;
  else if (settingNum == 48) return LS2state;
  else if (settingNum == 49) return HVoutAdjValue;
  else if (settingNum >  49 && settingNum <= 65) return potTCONSettings[settingNum - 50];
  else if (settingNum == 66) return PWM1Connect;
  else if (settingNum == 67) return PWM2Connect;
  else if (settingNum == 68) return Vout3CConnect;
  else if (settingNum == 69) return Vout3DConnect;
  else if (settingNum == 70) return LINSHLDConnect;
  else if (settingNum == 71) return LIN16Connect;
  
  else Serial.println();
}


void setSetting(uint8_t settingNum, int settingValue) {
  if (settingNum > 0 && settingNum <= 16) {
    potWiperSettings[settingNum - 1] = settingValue;
    MCP41HVExtender_SetWiper(settingNum - 1, potWiperSettings[settingNum - 1]);
    if (settingNum == 11) {
      //HS2state = false;
      digitalWrite(IH2, HS2state); //turn off the NFET to +12 on HVOut2
    }
    else if (settingNum == 12) {
      //LS2state = false;
      digitalWrite(IL2, LS2state); //turn off the NFET to Ground
    }
  }
  else if (settingNum > 16 && settingNum <= 24) {
    DAC2value[settingNum - 17] = settingValue;
    setDAC(DAC2value[settingNum - 17], settingNum - 17, Vout2address);
  }
  else if (settingNum > 24 && settingNum <= 32) {
    DAC3value[settingNum - 25] = settingValue;
    setDAC(DAC3value[settingNum - 25], settingNum - 25, Vout3address);
  }
  else if (settingNum == 33) {
    pwm1value = uint8_t(settingValue);
    LS1state = false;
    digitalWrite(IL1, LS1state); //turn off the NFET to ground in the H Bridge on port 17
    analogWrite(pwm1, pwm1value);
  }
  else if (settingNum == 34) {
    pwm2value = uint8_t(settingValue);
    HS1state = false;
    digitalWrite(IL1, HS1state); //turn off the NFET to ground in the H Bridge on port 17
    analogWrite(pwm2, pwm2value);
  }
  else if (settingNum == 35) {
    pwm3value = uint8_t(settingValue);
    analogWrite(pwm3, pwm3value);
  }
  else if (settingNum == 36) {
    pwm4value = uint8_t(settingValue);
    
    analogWrite(pwm4, pwm4value);
  }
  else if (settingNum == 37) {
    U1andU2POA = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 38) {
    U3andU4POA = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 39) {
    U5andU6POA = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 40) {
    U7andU8POA = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 41) {
    CAN0term = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 42) {
    CAN1term = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 43) {
    CAN2term = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 44) {
    LINmaster = boolean(settingValue);
    setTerminationSwitches();
  }
  else if (settingNum == 45) {
    HS1state = uint8_t(settingValue);
    if (HS2state){
      PWM1Connect = false;
      setConnectionSwitches();
    }
    digitalWrite(IH1,HS1state);
  }
  else if (settingNum == 46) { //These share pins
    HS2state = uint8_t(settingValue);
    if (HS2state){
      potTCONSettings[10] = 0;
      MCP41HVExtender_SetTerminals(10, potTCONSettings[10]); //Port 11
    }
    
    digitalWrite(IH2, HS2state);
  }
  else if (settingNum == 47) {
    pwm1value = 0;
    analogWrite(pwm1, pwm1value);
    LS1state = uint8_t(settingValue);
    digitalWrite(IL1, LS1state);
  }
  else if (settingNum == 48) {
    LS2state = uint8_t(settingValue);
    if (LS2state){
      potTCONSettings[11] = 0;
      MCP41HVExtender_SetTerminals(11, potTCONSettings[11]);
    }
    digitalWrite(IL2, LS2state);
  }
  else if (settingNum == 49) {
    HVoutAdjValue = uint8_t(settingValue);
    MCP41HV_SetWiper(CShvadjPin, HVoutAdjValue);
  }
  else if (settingNum >= 50 && settingNum <= 65) {
    if (settingNum == 60 && settingValue == 7){
      HS2state = 0;
      digitalWrite(IH2, HS2state);
    }
    
    potTCONSettings[settingNum - 50] = uint8_t(settingValue);
    MCP41HVExtender_SetTerminals(settingNum - 50, potTCONSettings[settingNum - 50]);
    
  }
  else if (settingNum == 66) {
    LS1state = 0;
    digitalWrite(IL1,LS1state);
    PWM1Connect = boolean(settingValue);
    setConnectionSwitches();
    
  }
  else if (settingNum == 67) {
    PWM2Connect = boolean(settingValue);
    setConnectionSwitches();
    
  }
  else if (settingNum == 68) {
    Vout3CConnect = boolean(settingValue);
    setConnectionSwitches();
  }
  else if (settingNum == 69) {
    Vout3DConnect = boolean(settingValue);
    setConnectionSwitches();
  }
  else if (settingNum == 70) {
    LINSHLDConnect = boolean(settingValue);
    setConnectionSwitches();
  }
  else if (settingNum == 71) {
    LIN16Connect = boolean(settingValue);
    setConnectionSwitches();
    //TODO Change TCON16
  }
  
  listSetting(settingNum);

}



void setConnectionSwitches() {
  //Set the connection Switches of U31

  connectionSettings = PWM1Connect | PWM2Connect << 1 | Vout3CConnect << 2 | Vout3DConnect << 3 | LINSHLDConnect << 4 | !LINSHLDConnect << 5 | LIN16Connect << 6 | !LIN16Connect << 7;
  digitalWrite(CSVoutPin, LOW);
  delay(1);
  SPI1.transfer(connectionSettings);
  delay(1);
  digitalWrite(CSVoutPin, HIGH);
}

void getConnectionSwitches() {
  //Set the termination Switches for U31
  PWM1Connect    = (connectionSettings & 0b00000001) >> 0;
  PWM2Connect    = (connectionSettings & 0b00000010) >> 1;
  Vout3CConnect  = (connectionSettings & 0b00000100) >> 2;
  Vout3DConnect  = (connectionSettings & 0b00001000) >> 3;
  LINSHLDConnect = (connectionSettings & 0b00010000) >> 4;
  LIN16Connect   = (connectionSettings & 0b01000000) >> 6;
  //don't include 5 and 7 because they are tied together.
}

void setTerminationSwitches() {
  //Set the termination Switches of U21

  terminationSettings = U1andU2POA | U3andU4POA << 1 | U5andU6POA << 2 | U7andU8POA << 3 | CAN0term << 4 | CAN1term << 5 | CAN2term << 6 | LINmaster << 7;
  digitalWrite(CStermPin, LOW);
  delay(1);
  SPI1.transfer(terminationSettings);
  delay(1);
  digitalWrite(CStermPin, HIGH);
}

void getTerminationSwitches() {
  //Set the termination Switches for U21
  U1andU2POA = (terminationSettings & 0b00000001) >> 0;
  U3andU4POA = (terminationSettings & 0b00000010) >> 1;
  U5andU6POA = (terminationSettings & 0b00000100) >> 2;
  U7andU8POA = (terminationSettings & 0b00001000) >> 3;
  CAN0term   = (terminationSettings & 0b00010000) >> 4;
  CAN1term   = (terminationSettings & 0b00100000) >> 5;
  CAN2term   = (terminationSettings & 0b01000000) >> 6;
  LINmaster  = (terminationSettings & 0b10000000) >> 7;
}


void getEEPROMdata () {
  Serial.println(F("Getting EEPROM Contents..."));
  EEPROM.get(potWiperSettingsAddress, potWiperSettings);
  EEPROM.get(potTCONSettingsAddress, potTCONSettings);
  EEPROM.get(DAC2valueAddress, DAC2value);
  EEPROM.get(DAC3valueAddress, DAC3value);
  EEPROM.get(pwm1valueAddress, pwm1value);
  EEPROM.get(pwm2valueAddress, pwm2value);
  EEPROM.get(pwm3valueAddress, pwm3value);
  EEPROM.get(pwm4valueAddress, pwm4value);
  EEPROM.get(HS1Address, HS1state);
  EEPROM.get(HS2Address, HS2state);
  EEPROM.get(LS1Address, LS1state);
  EEPROM.get(LS2Address, LS2state);
  EEPROM.get(HVoutAdjAddress, HVoutAdjValue);
  EEPROM.get(componentIDAddress, componentID);
 // EEPROM.get(number_of_CAN_msgsAddress, number_of_CAN_msgs);
  EEPROM.get(terminationSettingsAddress, terminationSettings);
  getTerminationSwitches();
  EEPROM.get(connectionSettingsAddress, connectionSettings);
  getConnectionSwitches();
  EEPROM.get(programmedForAddress, programmedFor);
  EEPROM.get(programmedByAddress, programmedBy);
  EEPROM.get(programmedDateAddress, programDate);
  //EEPROM.get(softwareVersionAddress, softwareVersion);
  EEPROM.get(programNotesAddress, programNotes);

 // int CANmsgSize = sizeof(CANTXmessage);

//  for (int msg = 0; msg < number_of_CAN_msgs; msg++) {
//    EEPROM.get(number_of_CAN_msgsAddress + 1 + msg * CANmsgSize, CANTXmessages[msg]);
//  }
  
}


void setDefaultEEPROMdata () {
  Serial.println(F("Setting EEPROM Contents..."));
  EEPROM.put(potWiperSettingsAddress, potWiperSettings);
  EEPROM.put(potTCONSettingsAddress, potTCONSettings);
  EEPROM.put(DAC2valueAddress, DAC2value);
  EEPROM.put(DAC3valueAddress, DAC3value);
  EEPROM.put(pwm1valueAddress, pwm1value);
  EEPROM.put(pwm2valueAddress, pwm2value);
  EEPROM.put(pwm3valueAddress, pwm3value);
  EEPROM.put(pwm4valueAddress, pwm4value);
  EEPROM.put(HS1Address, HS1state);
  EEPROM.put(HS2Address, HS2state);
  EEPROM.put(LS1Address, LS1state);
  EEPROM.put(LS2Address, LS2state);
  EEPROM.put(HVoutAdjAddress, HVoutAdjValue);
  EEPROM.put(componentIDAddress, componentID);
//ao  EEPROM.put(number_of_CAN_msgsAddress, number_of_CAN_msgs);
  EEPROM.put(terminationSettingsAddress, terminationSettings);
  EEPROM.put(connectionSettingsAddress, connectionSettings);
  EEPROM.put(programmedForAddress, programmedFor);
  EEPROM.put(programmedByAddress, programmedBy);
  EEPROM.put(programmedDateAddress, programDate);
  EEPROM.put(programNotesAddress, programNotes);
  EEPROM.put(softwareVersionAddress, softwareVersion);
//  int CANmsgSize = sizeof(CANTXmessage);
//
//  for (int msg = 0; msg < number_of_CAN_msgs; msg++) {
//    EEPROM.put(number_of_CAN_msgsAddress + 1 + msg * CANmsgSize, CANTXmessages[msg] );
//  }
  
  Serial.println("Done.");

}


void  adjustError() {
  Serial.println(F("SS - Condition not met. Turn adjust mode on by typing AO, then select a setting with CS"));
}



/********************************************************************************************/
/*                         Begin Function Calls for Knob Buttons                            */

void myClickFunction() {
  //ADJUST_MODE_ON = false;
  turnOffAdjustMode();
//  if (SAFE_TO_ADJUST) {
//    ADJUST_MODE_ON = !ADJUST_MODE_ON;
//    if (ADJUST_MODE_ON) turnOnAdjustMode();
//    else turnOffAdjustMode();
//  }
//  else {
//    ADJUST_MODE_ON = false;
//    Serial.println(F("Double Click to Inititate Adjustment by the knob"));
//  }
}
void myDoubleClickFunction() {

//  SAFE_TO_ADJUST = !SAFE_TO_ADJUST;
//  if (SAFE_TO_ADJUST) {
//    Serial.println(F("Adjustments by knob are enabled. Click to toggle modes. Double click to lock."));
//  }
//  else
//  {
//    Serial.println(F("Adjustments by knob are locked. Double click to enable."));
//    
//  }
//  ADJUST_MODE_ON = false;
//  knobLowLimit = 1;
//  knobHighLimit = numSettings - 1;
//  knob.write(currentSetting);
  
}
void longPressStart() {
  ignitionCtlState = !ignitionCtlState;
  digitalWrite(ignitionCtlPin, ignitionCtlState);
  digitalWrite(greenLEDpin, ignitionCtlState);
  CANTX_500ms_timer = 490;
}

void longPress() {} //Do nothing at this time.
void longPressStop() {} 

/*                         End Function Calls for Knob Buttons                            */
/********************************************************************************************/




/********************************************************************************************/
/*                       Begin Function Calls for Serial Commands                           */

void turnOnAdjustMode() {
  ADJUST_MODE_ON = 1;
  Serial.println(F("AO - Turned Adjustment mode on. Type AF or click to turn off. Type SS,XXXX  or scroll knob and click to set settings."));
  Serial.print(F("Current Setting for Adjustement is "));
  Serial.print(currentSetting);
  Serial.print(" - ");
  Serial.println(settingNames[currentSetting]);
  knob.write(getSetting(currentSetting));
  setLimits(currentSetting);

}

void turnOffAdjustMode() {
  ADJUST_MODE_ON = 0;
  Serial.println(F("AF - Turned Setting Adjustment mode off. Type AO to turn on. Scroll knob to select a setting."));
  knob.write(currentSetting);
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
}


void changeSetting() {
  Serial.println(F("CS - Change or Select Setting."));
  //Serial.print(currentSetting);
  //Serial.print(" - ");
  //Serial.print(settingNames[currentSetting]);
  //Serial.print(" to ");
  if (commandString.length() > 0) {
    currentSetting = constrain(commandString.toInt(), 0, numSettings);
    
  }
  //listSetting(currentSetting);
  if (ADJUST_MODE_ON){
    setLimits(currentSetting);
    knob.write(getSetting(currentSetting));
  }
  else{
    if (knob.read() == currentSetting) listSetting(currentSetting);
    else knob.write(currentSetting); //automatic listSetting if knob changes
    knobLowLimit = 1;
    knobHighLimit = numSettings - 1;
  }
}

void listSettings(){
  Serial.println(F("LS - List Settings. "));
  for (int i = 1; i < numSettings; i++) listSetting(i);
}

void changeValue(){
  //Set value from Serial commands
  if (ADJUST_MODE_ON && currentSetting != 0) {
    Serial.println(F("SS - Set Setting."));
    int adjustmentValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
    setSetting(currentSetting, adjustmentValue);
    currentKnob = getSetting(currentSetting);
    knob.write(currentKnob);
  }
  else
  {
    adjustError();
  }
}

void saveEEPROM(){
  //Save settings to EEPROM
  Serial.println(F("SA - Saving Settings to EEPROM."));
  setDefaultEEPROMdata();
}
/*                End Function Calls for Serial and Knob Commands                           */
/********************************************************************************************/





/****************************************************************/
/*   Begin Function Calls for Digital Potentiometer             */

uint8_t MCP41HV_SetTerminals(uint8_t pin, uint8_t TCON_Value) {
  digitalWrite(pin, LOW);
  delay(1);
  SPI1.transfer(0x40); //Write to TCON Register
  SPI1.transfer(TCON_Value + 8);
  //SPI1.transfer(0x4C); //Read Command
  //uint8_t result = SPI1.transfer(0xFF); //Read Terminal Connection (TCON) Register
  digitalWrite(pin, HIGH);
  delay(1);
  return 0;//result & 0x0F;
}

uint8_t MCP41HVExtender_SetTerminals(uint8_t pin, uint8_t TCON_Value) {
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  uint8_t latReg = 0x0A;
  if (pin >= 8) {
    pin -= 8;
    latReg = 0x1A;
  }
  SPI1.transfer(latReg);
  SPI1.transfer(bitPositions[pin]);
  digitalWrite(CSpotsPin, HIGH);
  delay(1);

  SPI1.transfer(0x40); //Write to TCON Register
  SPI1.transfer(TCON_Value + 8);
  SPI1.transfer(0x4C); //Read Command
  uint8_t result = SPI1.transfer(0xFF); //Read Terminal Connection (TCON) Register

  delay(1);
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(latReg);
  SPI1.transfer(0xFF);
  digitalWrite(CSpotsPin, HIGH);
  delay(1);
  return result & 0x0F;
}

uint8_t MCP41HV_SetWiper(uint8_t pin, uint8_t potValue)
{
  digitalWrite(pin, LOW);
  delay(1);
  SPI1.transfer(0x00); //Write to wiper Register
  SPI1.transfer(potValue);
  //SPI1.transfer(0x0C); //Read command
  //uint8_t result = SPI1.transfer(0xFF); //Read Wiper Register
  digitalWrite(pin, HIGH);
  delay(1);
  return 0;// result;
}

uint8_t MCP41HVExtender_SetWiper(uint8_t pin, uint8_t potValue)
{
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  uint8_t latReg = 0x0A;
  if (pin >= 8) {
    pin -= 8;
    latReg = 0x1A;
  }
  SPI1.transfer(latReg);
  SPI1.transfer(bitPositions[pin]);
  //Serial.print(bitPositions[pin],BIN);
  //Serial.print(" ");
  digitalWrite(CSpotsPin, HIGH);
  delay(1);

  SPI1.transfer(0x00); //Write to wiper Register
  SPI1.transfer(potValue);

  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(latReg);
  SPI1.transfer(0xFF);
  digitalWrite(CSpotsPin, HIGH);
  delay(1);

  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(latReg);
  SPI1.transfer(bitPositions[pin]);
  digitalWrite(CSpotsPin, HIGH);
  delay(1);

  SPI1.transfer(0x0C); //Read command
  uint8_t result = SPI1.transfer(0xFF); //Read Wiper Register

  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(latReg);
  SPI1.transfer(0xFF);
  digitalWrite(CSpotsPin, HIGH);
  delay(1);
  return result;
} 
/*   End Function Calls for Digital Potentiometer               */
/****************************************************************/


/****************************************************************/
/*   Begin Function Calls for DAC                               */

void initializeDACs(uint8_t address) {
  Serial.print("Setting DAC Internal Reference register with address of 0x");
  Serial.println(address, HEX);
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b10000000);
  Wire.write(0x00);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b10000000);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  char highC = Wire.read();
  char lowC = Wire.read();
  Serial.print("Internal Reference Register: ");
  Serial.println(lowC);         // print the character

  Serial.println("Setting LDAC register.");
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01100000);
  Wire.write(0xFF);
  Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01100000);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  highC = Wire.read();
  lowC = Wire.read();
  Serial.print("LDAC Register: ");
  Serial.println(lowC, HEX);        // print the character

  Serial.println("Setting DAC Power Down register to On.");
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01000000);
  Wire.write(0b00011111);
  Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01000000);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  highC = Wire.read();
  lowC = Wire.read();
  Serial.print("Power Register: ");
  Serial.println(lowC, BIN);        // print the character

  Serial.print(F("Done with DAC at address 0x"));
  Serial.println(address, HEX);
}

uint16_t setDAC(uint16_t setting, uint8_t DACchannel, uint8_t address) {
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b00100000 + DACchannel);
  Wire.write((setting & 0x0FF0) >> 4);
  Wire.write((setting & 0x000F) << 4);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b00010000 + DACchannel);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  uint8_t highC = Wire.read();
  uint8_t lowC =  Wire.read();
  uint16_t result = (highC << 4) + (lowC >> 4);
  return result;
}

/*   End Function Calls for DAC                              */
/****************************************************************/


/**************************************************************************************/
/*               Begin Function calls for User input data                             */

void listInfo() {
  Serial.print("Component ID (Make*Model*Serial*Unit): ");
  Serial.println(componentID);
  Serial.print("Programmed for: ");
  Serial.println(programmedFor);
  Serial.print("Programmed by: ");
  Serial.println(programmedBy);
  Serial.print("Program Date: ");
  Serial.println(programDate);
  Serial.print("Software Version: ");
  Serial.println(softwareVersion);
  Serial.print("Program Notes: ");
  Serial.println(programNotes);
  
}

void setProgrammedFor() {
  if (commandString.length() > 0) commandString.toCharArray(programmedFor,200);
  Serial.print("PF - SSS unit is programmed for: ");
  Serial.println(programmedFor);
}

void setProgrammedBy() {
  if (commandString.length() > 0) commandString.toCharArray(programmedBy,200);
  Serial.print("PB - SSS unit was programmed by: ");
  Serial.println(programmedBy);
}

void setProgramDate() {
  if (commandString.length() > 0) commandString.toCharArray(programDate,200);
  Serial.print("PD - Date SSS unit was programmed: ");
  Serial.println(programDate);
}

void getSoftwareVersion() {
  Serial.print("SW - Software/Firmware version: ");
  Serial.println(softwareVersion);
}

void changeComponentID() {
  if (commandString.length() > 5) commandString.toCharArray(componentID,200);
  Serial.print(F("CI - Component Information (Make*Model*Serial*Unit): "));
  Serial.println(componentID);
  if (commandString.length() <= 5 && commandString.length() > 0) Serial.println(F("Please make the component ID longer than 5 characters to change it."));
}

void setProgramNotes(){
  if (commandString.length() > 0) commandString.toCharArray(programNotes,1000);
  Serial.println(F("PN - Programmer Notes: "));
  Serial.println(programNotes);
}

void setVIN(){
  if (commandString.length() > 0) commandString.toCharArray(vehicleIdentificationNum,20);
  Serial.println(F("VI - Vehicle Identification Number (VIN): "));
  Serial.println(vehicleIdentificationNum);
}

void setDisplayCAN(){
  Serial.print(F("DS - Display CAN Messages "));
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
    Serial.print("CAN0 set to ");
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
    Serial.println("CAN0 set to automatically set baudrate.");
  }
}

void autoBaud1(){
  Serial.println(F("B1 - Set the baudrate for CAN 1 or select AutoBaud"));
  char baudstring[9];
  if (commandString.length() > 0){
    commandString.toCharArray(baudstring,9);
    BAUDRATE1 = strtoul(baudstring,0,10);
    Serial.print("CAN1 set to ");
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
    Serial.println(F("CAN1 set to automatically set baudrate."));
  }
}

void displayBaud(){
  Serial.print("CAN0 ");
  Serial.println(BAUDRATE0);
  Serial.print("CAN1 ");
  Serial.println(BAUDRATE1);  
}

void startStopCAN(){
  int signalNumber = 0;
  char commandCharBuffer[4];
  Serial.println(F("CN - CAN Transmission."));
  if (commandString.length() > 0) {
    commandString.toCharArray(commandCharBuffer,4);
    signalNumber = atoi(commandCharBuffer);
  }
       if (signalNumber == 1)  send08FF0001 = !send08FF0001; 
  else if (signalNumber == 2)  send08FF0003 = !send08FF0003;
  else if (signalNumber == 3)  send08FF0103 = !send08FF0103;
  else if (signalNumber == 4)  send08FF0203 = !send08FF0203;
  else if (signalNumber == 5)  send08FF0303 = !send08FF0303;
  else if (signalNumber == 6)  send08FF0603 = !send08FF0603;
  else if (signalNumber == 7)  send08FF0703 = !send08FF0703;
  else if (signalNumber == 8)  send0CFF0703 = !send0CFF0703;
  else if (signalNumber == 9)  send0CFE6E0B = !send0CFE6E0B;
  else if (signalNumber == 10) send10FF0903 = !send10FF0903;
  else if (signalNumber == 11) send18F00131 = !send18F00131;
  else if (signalNumber == 12) send18F0010B = !send18F0010B;
  else if (signalNumber == 13) send18FEF117 = !send18FEF117;
  else if (signalNumber == 14) send18FEF128 = !send18FEF128;
  else if (signalNumber == 15) send18FEF121 = !send18FEF121;
  else if (signalNumber == 16) send18FEF131 = !send18FEF131;
  else if (signalNumber == 17) send18E00017 = !send18E00017;
  else if (signalNumber == 18) send18E00019 = !send18E00019;
  else if (signalNumber == 19) send18E00021 = !send18E00021;
  else if (signalNumber == 20) send18E00028 = !send18E00028;
  else if (signalNumber == 21) send18E00031 = !send18E00031;
  else if (signalNumber == 22) send10ECFF3D = !send10ECFF3D;
  else if (signalNumber == 23) send10ECFF01 = !send10ECFF01;
  else if (signalNumber == 24) send18FEF803 = !send18FEF803;
  else if (signalNumber == 25) send18FEF521 = !send18FEF521;
  else if (signalNumber == 26) send18FEF017 = !send18FEF017;
  else if (signalNumber == 27) send18FEF021 = !send18FEF021;
  else if (signalNumber == 28) send18FEF028 = !send18FEF028;
  else if (signalNumber == 29) send18FEF031 = !send18FEF031;
  else if (signalNumber == 30) send18DF00F9 = !send18DF00F9;
  else if (signalNumber == 31) send18DFFFF9 = !send18DFFFF9;
  else if (signalNumber == 32) send0CF00203 = !send0CF00203;
  else if (signalNumber == 33) send18F00503 = !send18F00503;
  
  
  else if (signalNumber == 254) {
    TXCAN = true;
    Serial.println(F("CAN Transmission turned on for all messages."));
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
    Serial.println(F("CAN Transmission turned off for all messages."));
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
  if      (signalNumber == 1)  {Serial.print(F(" 1: send08FF0001 = ")); Serial.println(send08FF0001);}
  else if (signalNumber == 2)  {Serial.print(F(" 2: send08FF0003 = ")); Serial.println(send08FF0003);}
  else if (signalNumber == 3)  {Serial.print(F(" 3: send08FF0103 = ")); Serial.println(send08FF0103);}
  else if (signalNumber == 4)  {Serial.print(F(" 4: send08FF0203 = ")); Serial.println(send08FF0203);}
  else if (signalNumber == 5)  {Serial.print(F(" 5: send08FF0303 = ")); Serial.println(send08FF0303);}
  else if (signalNumber == 6)  {Serial.print(F(" 6: send08FF0603 = ")); Serial.println(send08FF0603);}
  else if (signalNumber == 7)  {Serial.print(F(" 7: send08FF0703 = ")); Serial.println(send08FF0703);}
  else if (signalNumber == 8)  {Serial.print(F(" 8: send0CFF0703 = ")); Serial.println(send0CFF0703);}
  else if (signalNumber == 9)  {Serial.print(F(" 9: send0CFE6E0B = ")); Serial.println(send0CFE6E0B);}
  else if (signalNumber == 10) {Serial.print(F("10: send10FF0903 = ")); Serial.println(send10FF0903);}
  else if (signalNumber == 11) {Serial.print(F("11: send18F00131 = ")); Serial.println(send18F00131);}
  else if (signalNumber == 12) {Serial.print(F("12: send18F0010B = ")); Serial.println(send18F0010B);}
  else if (signalNumber == 13) {Serial.print(F("13: send18FEF117 = ")); Serial.println(send18FEF117);}
  else if (signalNumber == 14) {Serial.print(F("14: send18FEF128 = ")); Serial.println(send18FEF128);}
  else if (signalNumber == 15) {Serial.print(F("15: send18FEF121 = ")); Serial.println(send18FEF121);}
  else if (signalNumber == 16) {Serial.print(F("16: send18FEF131 = ")); Serial.println(send18FEF131);}
  else if (signalNumber == 17) {Serial.print(F("17: send18E00017 = ")); Serial.println(send18E00017);}
  else if (signalNumber == 18) {Serial.print(F("18: send18E00019 = ")); Serial.println(send18E00019);}
  else if (signalNumber == 19) {Serial.print(F("19: send18E00021 = ")); Serial.println(send18E00021);}
  else if (signalNumber == 20) {Serial.print(F("20: send18E00028 = ")); Serial.println(send18E00028);}
  else if (signalNumber == 21) {Serial.print(F("21: send18E00031 = ")); Serial.println(send18E00031);}
  else if (signalNumber == 22) {Serial.print(F("22: send10ECFF3D = ")); Serial.println(send10ECFF3D);}
  else if (signalNumber == 23) {Serial.print(F("23: send10ECFF01 = ")); Serial.println(send10ECFF01);}
  else if (signalNumber == 24) {Serial.print(F("24: send18FEF803 = ")); Serial.println(send18FEF803);}
  else if (signalNumber == 25) {Serial.print(F("25: send18FEF521 = ")); Serial.println(send18FEF521);}
  else if (signalNumber == 26) {Serial.print(F("26: send18FEF017 = ")); Serial.println(send18FEF017);}
  else if (signalNumber == 27) {Serial.print(F("27: send18FEF021 = ")); Serial.println(send18FEF021);}
  else if (signalNumber == 28) {Serial.print(F("28: send18FEF028 = ")); Serial.println(send18FEF028);}
  else if (signalNumber == 30) {Serial.print(F("30: send18DF00F9 = ")); Serial.println(send18DF00F9);}
  else if (signalNumber == 31) {Serial.print(F("31: send18DFFFF9 = ")); Serial.println(send18DFFFF9);}
  else if (signalNumber == 32) {Serial.print(F("33: send0CF00203 = ")); Serial.println(send0CF00203);}
  else if (signalNumber == 33) {Serial.print(F("33: send18F00503 = ")); Serial.println(send18F00503);}

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
      else Serial.println("0: Invalid ID format");
      
      String dataString = commandString.substring(commaIndex1+1);
      
      dataString.toCharArray(dataCharBuffer,17);
      txmsg.len = strlen(dataCharBuffer)/2;
      //Serial.print(" ");
      //Serial.print(txmsg.len,HEX);
      // Serial.print(" ");
      for (uint8_t i = 0; i < txmsg.len*2 ; i++){
        if (isxdigit(dataCharBuffer[i])) goodData = true; 
        else { goodData = false; Serial.println("0 Non Hex Characters or Odd number of nibbles or ID is too long"); break; }
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
        Serial.println(txmsg.len);
      }
      else if (channel == 1){
        Can1.write(txmsg);
        Serial.println(txmsg.len);
      }
      else Serial.println("0: Invalid Channel for SM.");
    }
    
    else
      Serial.println(F("0: Invalid input data for SM. Input should be using hex characters with no spaces in the form SM,channel,ID,data."));
  }
  else
  {
    Serial.println(F("0: Missing or invalid data to send."));
  }
  txmsg.ext = 1; //set default
  txmsg.len = 8;
}



/*                 End Function calls for User input data                             */
/**************************************************************************************/

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
    sprintf(J1939Characters,"%d,%d,%d,%d,%d,",PRIORITY,PGN,DA,SA,DLC);
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
      sendJ1939(0,6,0xFEEB,0xFF,sourceAddress,strlen(componentID),componentID);
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


void printFrame(CAN_message_t &rxmsg, int mailbox)
{
  uint32_t ID = rxmsg.id;
  uint8_t len = rxmsg.len;
  sprintf(displayBuffer," %10d %08X %1d",micros(),ID,len);
  Serial.print(displayBuffer);
  for (uint8_t i = 0; i<len;i++){
    char byteDigits[4];
    sprintf(byteDigits," %02X",rxmsg.buf[i]);
    Serial.print(byteDigits);
  }
  Serial.println();
}


class CAN1Class : public CANListener 
{
public:
   void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something
};

void CAN1Class::gotFrame(CAN_message_t &frame, int mailbox)
{ 
   RXCAN1count++;
   CAN1baudNotDetected = false;
   if (RXCAN1orJ1708timer >=90){
     RXCAN1orJ1708timer = 0;
     if (ignitionCtlState) greenLEDstate = !greenLEDstate;
     else greenLEDstate = false;
     digitalWrite(greenLEDpin, greenLEDstate); 
   }
   if (displayCAN){
     Serial.print("CAN1");
     printFrame(frame, mailbox);
   }
}
CAN1Class CAN1Class;

class CAN0Class : public CANListener 
{
public:
   void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something
};

void CAN0Class::gotFrame(CAN_message_t &frame, int mailbox)
{  
   RXCAN0count++;
   CAN0baudNotDetected = false;
   if (RXCAN0timer >=90){
    RXCAN0timer = 0;
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);  
   }
   if (displayCAN){
     Serial.print("CAN0");
     printFrame(frame, mailbox);
   }
   parseJ1939(frame);
     
}
CAN0Class CAN0Class;


  
void setup() {
  Serial.begin(115200);
  Serial1.begin(19200);
  
  delay(800);
  Serial.println("Welcome to the Smart Sensor Simulator 2.");
  inputString.reserve(200);
  
  //getEEPROMdata();
  getTerminationSwitches();
  getConnectionSwitches();

 
  
  pinMode(greenLEDpin,       OUTPUT);
  pinMode(redLEDpin,         OUTPUT);
  pinMode(CSdispPin,         OUTPUT);
  pinMode(CSpotsPin,         OUTPUT);
  pinMode(CSanalogPin,       OUTPUT);
  pinMode(CShvadjPin,        OUTPUT);
  pinMode(CStermPin,         OUTPUT);
  pinMode(CSVoutPin,         OUTPUT);
  pinMode(CSCANPin,          OUTPUT);
  pinMode(ignitionCtlPin,    OUTPUT);
  pinMode(pwm1,              OUTPUT);
  pinMode(pwm2,              OUTPUT);
  pinMode(pwm3,              OUTPUT);
  pinMode(pwm4,              OUTPUT);
  pinMode(IH1,               OUTPUT);
  pinMode(IH2,               OUTPUT);
  pinMode(IL1,               OUTPUT);
  pinMode(IL2,               OUTPUT);
  pinMode(buttonPin,   INPUT_PULLUP);
  pinMode(LED_BUILTIN,       OUTPUT);

  digitalWrite(greenLEDpin, greenLEDstate);
  digitalWrite(redLEDpin,     redLEDstate);
  digitalWrite(CSdispPin,            HIGH);
  digitalWrite(CSpotsPin,            HIGH);
  digitalWrite(CSanalogPin,          HIGH);
  digitalWrite(CShvadjPin,           HIGH);
  digitalWrite(CStermPin,            HIGH);
  digitalWrite(CSVoutPin,            HIGH);
  digitalWrite(CSCANPin,             HIGH);
  digitalWrite(pwm1,                  LOW);
  digitalWrite(pwm2,                  LOW);
  digitalWrite(pwm3,                  LOW);
  digitalWrite(pwm4,                  LOW);
  digitalWrite(IH1,                   LOW);
  digitalWrite(IH2,                   LOW);
  digitalWrite(IL1,                   LOW);
  digitalWrite(IL2,                   LOW);
  digitalWrite(ignitionCtlPin, ignitionCtlState);

  button.attachClick(myClickFunction);
  button.attachDoubleClick(myDoubleClickFunction);
  button.attachLongPressStart(longPressStart);
  button.attachLongPressStop(longPressStop);
  button.attachDuringLongPress(longPress);
  button.setPressTicks(2000);
  button.setClickTicks(250);
  
  
  SPI1.setSCK(32);
  SPI1.setMOSI(0);
  SPI1.setMISO(1);
  SPI1.begin();

  //Initialize the MCP Extender
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(IOCONA);
  SPI1.transfer(0b10111100);
  digitalWrite(CSpotsPin, HIGH);
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(IODIRA);
  SPI1.transfer(0x00);
  digitalWrite(CSpotsPin, HIGH);
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(IODIRB);
  SPI1.transfer(0x00);
  digitalWrite(CSpotsPin, HIGH);
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(OLATA);
  SPI1.transfer(0xFF);
  digitalWrite(CSpotsPin, HIGH);
  digitalWrite(CSpotsPin, LOW);
  SPI1.transfer(ExtenderOpCode);
  SPI1.transfer(OLATB);
  SPI1.transfer(0xFF);
  digitalWrite(CSpotsPin, HIGH);

  digitalWrite(CSanalogPin, LOW);
  delay(2);
  //Write to Range Register 1 to Select the range for input channels
  SPI1.transfer(0b10111111);
  SPI1.transfer(0b11100000); //Write to ADC Device
  digitalWrite(CSanalogPin, HIGH);
  delay(1);

  digitalWrite(CSanalogPin, LOW);
  delay(2);
  //Write to Range Register 2 to Select the range for input channels
  SPI1.transfer(0b11011111);
  SPI1.transfer(0b11100000); //Write to ADC Device
  digitalWrite(CSanalogPin, HIGH);
  delay(1);

  digitalWrite(CSanalogPin, LOW);
  delay(2);
  //Write to Seq. Register
  SPI1.transfer(0b11111111);
  SPI1.transfer(0b11100000);
  //SPI1.transfer(0xFFFF);
  digitalWrite(CSanalogPin, HIGH);
  delay(1);

  digitalWrite(CSanalogPin, LOW);
  delay(2);
  //Write to control register to select the final channel in the seq. Set Seq1 =1  and Seq2=0
  //Write  RegSel1 RegSel2 ADD2 ADD1 ADD0 Mode1 Mode0 PM1 PM0 Coding Ref Seq1 Seq2 Zero Zero
  SPI1.transfer(0b10011100);
  SPI1.transfer(0b00111000);
  digitalWrite(CSanalogPin, HIGH);
  delay(1);

  //Set up the i2C for the DAC
  Wire.begin();
  Wire.setDefaultTimeout(200000); // 200ms

  initializeDACs(Vout2address);
  initializeDACs(Vout3address);


  listInfo();
  for (int i = 1; i < numSettings; i++) {
    setSetting(i, getSetting(i));
  }
  
  currentSetting = 0;
  setLimits(currentSetting);

  Serial.print("Starting CAN...");
  Can0.attachObj(&CAN0Class);
  Can1.attachObj(&CAN1Class);
  
  Serial.println("Done.");
  
  Serial.print("Setting Baud Rate...");
  BAUDRATE0=250000;
  Can0.begin(BAUDRATE0);
  BAUDRATE1=660000;
  Can1.begin(BAUDRATE1);
  Serial.println("Done.");
  
  
  Serial.print("Setting Filters...");
  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    Can1.setFilter(allPassFilter,filterNum); 
  }
//  for (uint8_t filterNum = 0; filterNum < 16;filterNum++){
//     CANClass0.attachMBHandler(filterNum);
//     CANClass1.attachMBHandler(filterNum);
//  }
  Serial.print("Done.\nAttaching General Handlers...");
  
  CAN0Class.attachGeneralHandler();
  CAN1Class.attachGeneralHandler();
  Serial.println("Done.");
  
  txmsg.ext = 1;
  txmsg.len = 8;

  //ignitionCtlState=true;
  //CAN0baudNotDetected = false;
  //CAN1baudNotDetected = false;
}



void loop() {
  
  /********************************************************************/
  /*            Begin AutoBaud Detection                              */
  /* This runs each loop and sets a value for the BAUDRATE if needed  */
  if (CAN0baudNotDetected && CAN0RXtimer >= CANWaitTimeout){
    CAN0RXtimer = 0;
    BAUDRATE0 = baudRateList[baudRateIndex0];
    baudRateIndex0 += 1;
    if (baudRateIndex0 == 4) baudRateIndex0=0; //reset the index after all elements have been tried.
    Can0.begin(BAUDRATE0);
    for (uint8_t filterNum = 4; filterNum < 16;filterNum++) Can0.setFilter(allPassFilter,filterNum);
   }
  
  if (CAN1baudNotDetected && CAN1RXtimer >= CANWaitTimeout){
    CAN1RXtimer = 0;
    BAUDRATE1 = baudRateList[baudRateIndex1];
    baudRateIndex1 += 1;
    if (baudRateIndex1 == 4) baudRateIndex1=0; //reset the index after all elements have been tried.
    Can1.begin(BAUDRATE1);
    for (uint8_t filterNum = 4; filterNum < 16;filterNum++) Can1.setFilter(allPassFilter,filterNum);
  }
  /*           End AutoBaud Detection                              */
  /********************************************************************/

  /************************************************************************/
  /*            Begin PERIODIC CAN Message Transmission                            */
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
       txmsg.id = 0x18FEF128; //CCVS from SA=40
       //signal 15
       txmsg.id = 0x18FEF128; //CCVS from SA=40
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
       txmsg.id = 0x18E00028; //Cab Message 1 from SA=40
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
       //signal 30
       if (send18DF00F9){
         memcpy(txmsg.buf,DF00F9,8);
         txmsg.id = 0x18DF00F9; //ACM
         Can0.write(txmsg);
       }
       if (send18DFFFF9){
         memcpy(txmsg.buf,DFFFF9,8);
         txmsg.id = 0x18DFFFF9; //ACM
         Can0.write(txmsg);
       }
       
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
    //Serial.println(F("Please put a comma after the two command characters."));
    if      (commandChars == "SM" || commandChars == "sm") sendMessage();
    else if (commandChars == "SS" || commandChars == "ss") changeValue();
    else if (commandChars == "SC" || commandChars == "sc") Serial.println(F("SC - Not implemented yet."));
    else if (commandChars == "SA" || commandChars == "sa") saveEEPROM();
    else if (commandChars == "AO" || commandChars == "ao") turnOnAdjustMode();
    else if (commandChars == "AF" || commandChars == "af") turnOffAdjustMode();
    else if (commandChars == "CS" || commandChars == "cs") changeSetting(); //Select setting to change
    else if (commandChars == "CI" || commandChars == "ci") changeComponentID();
    else if (commandChars == "LS" || commandChars == "ls") listSettings();
    else if (commandChars == "LI" || commandChars == "li") listInfo();
    else if (commandChars == "PF" || commandChars == "pf") setProgrammedFor();
    else if (commandChars == "PB" || commandChars == "pb") setProgrammedBy();
    else if (commandChars == "PD" || commandChars == "pd") setProgramDate();
    else if (commandChars == "PN" || commandChars == "pn") setProgramNotes();
    else if (commandChars == "SW" || commandChars == "sw") getSoftwareVersion();
    else if (commandChars == "VI" || commandChars == "vi") setVIN();
    else if (commandChars == "DS" || commandChars == "ds") setDisplayCAN();
    else if (commandChars == "B0" || commandChars == "b0") autoBaud0();
    else if (commandChars == "B1" || commandChars == "b1") autoBaud1();
    else if (commandChars == "DB" || commandChars == "db") displayBaud();
    else if (commandChars == "CN" || commandChars == "cn") startStopCAN();
    else if (commandChars == "DJ" || commandChars == "dj") displayJ1939 = !displayJ1939;
   
    
    else Serial.println(F("Unrecognized Command Characters. Use a comma after the command.\nKnown commands are CN, B0, B1, DS, VI, SW, PN, PD, PB, PF, LI, LS, CI, CS, AF, AO, SA, SC, SS, or SM."));
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
      setSetting(currentSetting, currentKnob);
    }
    else {
      currentSetting = currentKnob;
      listSetting(currentSetting);
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
  
  /*/Toggle the Builtin LED to check that the program is running. Can't use this if uisng SPI channel 0.*/
  if (toggleTimer >= LEDtoggleTime) {
    toggleTimer = 0;
    LEDstate = !LEDstate;
    digitalWrite(LED_BUILTIN, LEDstate);
  }
  /*             End LED Indicators for messages                        */
  /**********************************************************************/
  
}

