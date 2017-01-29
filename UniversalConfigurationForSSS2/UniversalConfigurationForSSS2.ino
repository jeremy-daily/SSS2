/*
   Smart Sensor Simulator 2
   Controlling the  Quadtrature Knob, Ignition Relay, and Voltage Regulator
   Hardware Revision 2

   Written by Dr. Jeremy S. Daily
   Synercon Technologies, LLC
   
   Proprietary Software - Not for Public Release

   Copyright (c) 2017        Jeremy S. Daily
*/

char softwareVersion[200] = "SSS2*Rev2*0.3*ec644a7e5406124655f49ae9d5e27038f4b450b5";

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
uint8_t potTCONSettings[16] ={7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7};
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
uint8_t HS2state = 0;
const uint16_t LS1Address = 58;
uint8_t LS1state = 0;
const uint16_t LS2Address = 59;
uint8_t LS2state = 0;
const uint16_t DAC3valueAddress = 61;
uint16_t DAC3value[8] {500,1000,1500,2000,2500,3000,3500,4000}; 



const uint16_t componentIDAddress = 1000;
char componentID[200] = "SYNER*SSS2-R02*0000*UNIVERSAL";

const uint16_t programmedForAddress = 1200;
char programmedFor[200] = "UNIVERSAL COMMUNICATIONS";

const uint16_t programmedByAddress = 1400;
char programmedBy[200] = "J. DAILY";

const uint16_t programmeDateAddress = 1600;
char programDate[200] = "01 Jan 2016";

const uint16_t softwareVersionAddress = 1800;

const uint16_t programNotesAddress = 2000;
char programNotes[1000] = "Notes: ";

const uint16_t vehicleIdentificationNumAddress = 3000;
char vehicleIdentificationNum[20] = "A Fake VIN";

const uint16_t number_of_CAN_msgsAddress = 256;
uint8_t number_of_CAN_msgs = 1;

struct CANTXmessage {
  uint8_t  period_index = 2; //in milliseconds
  uint8_t  channel = 0; //0=CAN0, 1=CAN1, or 2=CAN2
  uint8_t DLC = 8;
  uint32_t ID = 0x18FEF117; //assume extended, unless less than 0x7FF
  uint8_t dataField[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};

struct CANTXmessage CANTXmessages[256];


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


/****************************************************************/
/*                    CAN Setup                                 */


//Set up the CAN data structures
static CAN_message_t rxmsg, txmsg;

//set up a counter for each received message
uint32_t RXCAN0count = 0;
uint32_t RXCAN1count = 0;


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
elapsedMillis CANTX_5000ms_timer;
elapsedMillis CANTX_10000ms_timer;
elapsedMillis CANTX_30000ms_timer;


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
boolean CAN0term;
boolean CAN1term;
boolean CAN2term;
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

String inputString = "";

String commandChars = "";
String commandBuffer = "";

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

  "Port 26 (J18-10)",
  "Port 27 (J24-13)",
  "Port 28 (J24-14)",
  "Port 17 (J18- 1)",

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
    analogWrite(pwm1, pwm1value);
  }
  else if (settingNum == 34) {
    pwm2value = uint8_t(settingValue);
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
      potTCONSettings[11] = 0;
      MCP41HVExtender_SetTerminals(11, potTCONSettings[11]);
    }
    digitalWrite(IH2, HS2state);
  }
  else if (settingNum == 47) {
    LS1state = uint8_t(settingValue);
    digitalWrite(IL1, LS1state);
  }
  else if (settingNum == 48) {
    LS2state = uint8_t(settingValue);
    digitalWrite(IL2, LS2state);
  }
  else if (settingNum == 49) {
    HVoutAdjValue = uint8_t(settingValue);
    MCP41HV_SetWiper(CShvadjPin, HVoutAdjValue);
  }
  else if (settingNum >= 50 && settingNum <= 65) {
    if (settingNum == 60 && settingValue == 7){
      HS2state = false;
      digitalWrite(IH2, HS2state);
    }
    
    potTCONSettings[settingNum - 50] = uint8_t(settingValue);
    MCP41HVExtender_SetTerminals(settingNum - 50, potTCONSettings[settingNum - 50]);
    
  }
  else if (settingNum == 66) {
    HS1state = 0;
    digitalWrite(IH1,HS1state);
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
  EEPROM.get(number_of_CAN_msgsAddress, number_of_CAN_msgs);
  EEPROM.get(terminationSettingsAddress, terminationSettings);
  getTerminationSwitches();
  EEPROM.get(connectionSettingsAddress, connectionSettings);
  getConnectionSwitches();
  EEPROM.get(programmedForAddress, programmedFor);
  EEPROM.get(programmedByAddress, programmedBy);
  EEPROM.get(programmeDateAddress, programDate);
  //EEPROM.get(softwareVersionAddress, softwareVersion);
  EEPROM.get(programNotesAddress, programNotes);

  int CANmsgSize = sizeof(CANTXmessage);

  for (int msg = 0; msg < number_of_CAN_msgs; msg++) {
    EEPROM.get(number_of_CAN_msgsAddress + 1 + msg * CANmsgSize, CANTXmessages[msg]);
  }
  
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
  EEPROM.put(number_of_CAN_msgsAddress, number_of_CAN_msgs);
  EEPROM.put(terminationSettingsAddress, terminationSettings);
  EEPROM.put(connectionSettingsAddress, connectionSettings);
  EEPROM.put(programmedForAddress, programmedFor);
  EEPROM.put(programmedByAddress, programmedBy);
  EEPROM.put(programmeDateAddress, programDate);
  EEPROM.put(programNotesAddress, programNotes);
  EEPROM.put(softwareVersionAddress, softwareVersion);
  int CANmsgSize = sizeof(CANTXmessage);

  for (int msg = 0; msg < number_of_CAN_msgs; msg++) {
    EEPROM.put(number_of_CAN_msgsAddress + 1 + msg * CANmsgSize, CANTXmessages[msg] );
  }
  
  Serial.println("Done.");

}


void  adjustError() {
  Serial.println(F("SS - Condition not met. Turn adjust mode on by typing AO, double clicking and/or select a setting with CS"));
}



/********************************************************************************************/
/*                         Begin Function Calls for Knob Buttons                            */

void myClickFunction() {
  if (SAFE_TO_ADJUST) {
    ADJUST_MODE_ON = !ADJUST_MODE_ON;
    if (ADJUST_MODE_ON) turnOnAdjustMode();
    else turnOffAdjustMode();
  }
  else {
    ADJUST_MODE_ON = false;
    Serial.println(F("Double Click to Inititate Adjustment by the knob"));
  }
}
void myDoubleClickFunction() {

  SAFE_TO_ADJUST = !SAFE_TO_ADJUST;
  if (SAFE_TO_ADJUST) {
    Serial.println(F("Adjustments by knob are enabled. Click to toggle modes. Double click to lock."));
  }
  else
  {
    Serial.println(F("Adjustments by knob are locked. Double click to enable."));
    
  }
  ADJUST_MODE_ON = false;
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
  knob.write(currentSetting);
  
}
void longPressStart() {
  ignitionCtlState = !ignitionCtlState;
  digitalWrite(ignitionCtlPin, ignitionCtlState);
  digitalWrite(greenLEDpin, ignitionCtlState);
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
  Serial.println(F("AF - Turned Setting Adjustment mode off. Type AO to turn on or click. Scroll knob to select a setting."));
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
  if (commandBuffer.length() > 0) {
    currentSetting = constrain(commandBuffer.toInt(), 0, numSettings);
    
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
    int adjustmentValue = constrain(commandBuffer.toInt(), knobLowLimit, knobHighLimit);
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
  SPI1.transfer(0x40); //Write to TCON Register
  SPI1.transfer(TCON_Value + 8);
  SPI1.transfer(0x4C); //Read Command
  uint8_t result = SPI1.transfer(0xFF); //Read Terminal Connection (TCON) Register
  digitalWrite(pin, HIGH);
  return result & 0x0F;
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
  SPI1.transfer(0x0C); //Read command
  uint8_t result = SPI1.transfer(0xFF); //Read Wiper Register
  digitalWrite(pin, HIGH);
  delay(1);
  return result;
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
  if (commandBuffer.length() > 0) commandBuffer.toCharArray(programmedFor,200);
  Serial.print("PF - SSS unit is programmed for: ");
  Serial.println(programmedFor);
}

void setProgrammedBy() {
  if (commandBuffer.length() > 0) commandBuffer.toCharArray(programmedBy,200);
  Serial.print("PB - SSS unit was programmed by: ");
  Serial.println(programmedBy);
}

void setProgramDate() {
  if (commandBuffer.length() > 0) commandBuffer.toCharArray(programDate,200);
  Serial.print("PD - Date SSS unit was programmed: ");
  Serial.println(programDate);
}

void getSoftwareVersion() {
  Serial.print("SW - Software/Firmware version: ");
  Serial.println(softwareVersion);
}

void changeComponentID() {
  if (commandBuffer.length() > 5) commandBuffer.toCharArray(componentID,200);
  Serial.print(F("CI - Component Information (Make*Model*Serial*Unit): "));
  Serial.println(componentID);
  if (commandBuffer.length() <= 5 && commandBuffer.length() > 0) Serial.println(F("Please make the component ID longer than 5 characters to change it."));
}

void setProgramNotes(){
  if (commandBuffer.length() > 0) commandBuffer.toCharArray(programNotes,1000);
  Serial.println(F("PN - Programmer Notes: "));
  Serial.println(programNotes);
}

void setVIN(){
  if (commandBuffer.length() > 0) commandBuffer.toCharArray(vehicleIdentificationNum,20);
  Serial.println(F("VI - Vehicle Identification Number (VIN): "));
  Serial.println(vehicleIdentificationNum);
}

void setDisplayCAN(){
  Serial.print(F("DS - Display CAN Messages "));
  displayCAN = !displayCAN;
  if (displayCAN) Serial.println("on.");
  else Serial.println("off.");
}

/*                 End Function calls for User input data                             */
/**************************************************************************************/

void printFrame(CAN_message_t &frame, int mailbox)
{
  uint32_t ID = rxmsg.id;
  uint8_t len = rxmsg.len;
  sprintf(displayBuffer," %10i %08X %1i",micros(),ID,len);
  Serial.print(displayBuffer);
  for (uint8_t i = 0; i<len;i++){
    char byteDigits[4];
    sprintf(byteDigits," %02X",rxmsg.buf[i]);
    Serial.print(byteDigits);
  }
  Serial.println();
}

CAN_filter_t allPassFilter;

class CAN1Class : public CANListener 
{
public:
   void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something
};

void CAN1Class::gotFrame(CAN_message_t &frame, int mailbox)
{ 
   RXCAN1count++;
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

class CAN0Class : public CANListener 
{
public:
   void gotFrame(CAN_message_t &frame, int mailbox); //overrides the parent version so we can actually do something
};

void CAN0Class::gotFrame(CAN_message_t &frame, int mailbox)
{  
   RXCAN0count++;
   if (RXCAN0timer >=90){
    RXCAN0timer = 0;
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);  
   }
   if (displayCAN){
     Serial.print("CAN0");
     printFrame(frame, mailbox);
   }  
}


CAN1Class CAN1Class;
CAN0Class CAN0Class;
  
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("Welcome to the Smart Sensor Simulator 2.");
  inputString.reserve(200);
  
  //getEEPROMdata();
  getTerminationSwitches();
  getConnectionSwitches();
  
  Can0.begin(250000);  
  Can1.begin(250000);
  Can0.attachObj(&CAN0Class);
  Can1.attachObj(&CAN1Class);
  
  
  CAN_filter_t allPassFilter;
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;

  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    Can1.setFilter(allPassFilter,filterNum); 
  }
//  for (uint8_t filterNum = 0; filterNum < 16;filterNum++){
//     CANClass0.attachMBHandler(filterNum);
//     CANClass1.attachMBHandler(filterNum);
//  }
  CAN0Class.attachGeneralHandler();
  CAN1Class.attachGeneralHandler();
  
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
}


void loop() {
  //check for button updates
  button.tick();

  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 140) {
    commandChars = Serial.readStringUntil(',');
    if (Serial.available()) commandBuffer = Serial.readStringUntil('\n');
    else commandBuffer = "";
    //Serial.println(F("Please put a comma after the two command characters."));
    if      (commandChars == "SM" || commandChars == "sm") Serial.println(F("SM not implemented yet."));
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
    
    else Serial.println(F("Unrecognized Command Characters. Use a comma after the command.\nKnown commands are DS, VI, SW, PN, PD, PB, PF, LI, LS, CI, CS, AF, AO, SA, SC, SS, or SM."));
    Serial.flush();
  }
  /*              End Serial Command Processing                   */
  /****************************************************************/

  /****************************************************************/
  /*            Begin Quadrature Knob Processing                  */
  int32_t newKnob = knob.read();
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
  /*           Begin CAN Message Processing                   
  //Read and display a CAN Message
  if (Can0.read(rxmsg)) {
    RXCAN0timer = 0; // reset the receive timer
    uint32_t ID = rxmsg.id;
    uint8_t len = rxmsg.len;
    if (displayCAN){
        sprintf(displayBuffer,"%10i %08X %1i",micros(),ID,len);
        Serial.print(displayBuffer);
        for (uint8_t i = 0; i<len;i++){
          char byteDigits[4];
          sprintf(byteDigits," %02X",rxmsg.buf[i]);
          Serial.print(byteDigits);
        }
        Serial.println();
    }
    
    
  }
 */
  //Reset the greenLED after a timeout in case the last CAN message was on the wrong state
  if (RXCAN0timer >= 200) { 
    RXCAN0timer = 0;
    redLEDstate = true;
    digitalWrite(redLEDpin, redLEDstate); //Use red because it is the power button.
  }
  
  //Reset the greenLED after a timeout in case the last CAN message was on the wrong state
  if (RXCAN1orJ1708timer >= 200) { 
    RXCAN1orJ1708timer = 0;
    if (ignitionCtlState) greenLEDstate = true;
    else greenLEDstate = false;
    digitalWrite(greenLEDpin, greenLEDstate); 
  }
 
  /*             End CAN Message Processing                       */
  /****************************************************************/
  
  //Toggle the Builtin LED to check that the program is running. Can't use this if uisng SPI channel 0.
  if (toggleTimer >= LEDtoggleTime) {
    toggleTimer = 0;
    LEDstate = !LEDstate;
    digitalWrite(LED_BUILTIN, LEDstate);
    txmsg.buf[0]=1;
    //Can0.write(txmsg);
    
  }

}

