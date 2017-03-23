/*************************************************** 
  This is a library for the Smart Sensor Simulator 2
  Revision 3

  Written by Jeremy Daily for Synercon Technologies, LLC.  
  
 ****************************************************/

#include <SPI.h>
#include <i2c_t3.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include "OneButton.h"
#include "Adafruit_MCP23017.h"

Adafruit_MCP23017 ConfigExpander; //U21
Adafruit_MCP23017 PotExpander; //U33


uint8_t potWiperSettings[16] ={21,22,22,56,56,0,10,56,56,0,56,56,56,56,56,255};
uint8_t potTCONSettings[16] ={3,3,3,7,7,3,3,7,7,3,0,0,7,7,7,0};
uint16_t DAC2value[8] = {0,0,0,0,512,512,0,0};
uint8_t pwm1value = 0;
uint8_t pwm2value = 100;
uint8_t pwm3value = 19;
uint8_t pwm4value = 222; //Set this for Bendix
uint8_t HVoutAdjValue = 168;
uint8_t connectionSettings = 0b11001110;
uint8_t HS1state = 0;
uint8_t HS2state = 0;
uint8_t LS1state = 0;
uint8_t LS2state = 1; //Needed for Bendix 
uint16_t DAC3value[8] {0,0,0,0,4095,4095,4095,4095}; 

//Variables that get loaded from contents in the EEPROM
const uint16_t potWiperSettingsAddress = 0;
const uint16_t potTCONSettingsAddress = 16;
const uint16_t DAC2valueAddress = 32;
const uint16_t pwm1valueAddress = 48;
const uint16_t pwm2valueAddress = 49;
const uint16_t pwm3valueAddress = 50;
const uint16_t pwm4valueAddress = 51;
const uint16_t HVoutAdjAddress = 52;
const uint16_t terminationSettingsAddress = 54;
const uint16_t connectionSettingsAddress = 55;
const uint16_t HS1Address = 56;
const uint16_t HS2Address = 57;
const uint16_t LS1Address = 58;
const uint16_t LS2Address = 59;
const uint16_t DAC3valueAddress = 61;
const uint16_t componentIDAddress = 1000;

const uint16_t programmedForAddress = 1200;
char programmedFor[200] = "UNIVERSAL COMMUNICATIONS";

const uint16_t programmedByAddress = 1400;
char programmedBy[200] = "J. DAILY";

const uint16_t programmedDateAddress = 1600;
char programDate[200] = "02 Feb 2017";

const uint16_t softwareVersionAddress = 1800;

const uint16_t programNotesAddress = 2000;
char programNotes[1000] = "Notes: ";

const uint16_t vehicleIdentificationNumAddress = 3000;
char vehicleIdentificationNum[20] = "A Fake VIN";





/****************************************************************/
/*                         Pin Defintions                       */
const uint8_t greenLEDpin       = 2;
const uint8_t redLEDpin         = 5;
const uint8_t CSdispPin         = 9;
const uint8_t CSCANPin          = 15;
const uint8_t PWM1Pin           = 16;
const uint8_t PWM2Pin           = 17;
const uint8_t CSanalogPin       = 20;
const uint8_t CStermPin         = 21;
const uint8_t PWM3Pin           = 22;
const uint8_t PWM4Pin           = 23;
const uint8_t buttonPin         = 24;
const uint8_t CStouchPin        = 26;
const uint8_t IH1Pin            = 35;
const uint8_t IH2Pin            = 36;
const uint8_t IL1Pin            = 37;
const uint8_t IL2Pin            = 38;
const uint8_t ignitionCtlPin    = 39;

void setPinModes(){
    pinMode(greenLEDpin, OUTPUT);
    pinMode(redLEDpin, OUTPUT);
    pinMode(CSdispPin, OUTPUT);
    pinMode(CSCANPin, OUTPUT);
    pinMode(PWM1Pin, OUTPUT);
    pinMode(PWM2Pin, OUTPUT);
    pinMode(CSanalogPin, OUTPUT);
    pinMode(CStermPin, OUTPUT);
    pinMode(PWM3Pin, OUTPUT);
    pinMode(PWM4Pin, OUTPUT);
    pinMode(buttonPin, INPUT);
    pinMode(CStouchPin, OUTPUT);
    pinMode(IH1Pin, OUTPUT);
    pinMode(IH2Pin, OUTPUT);
    pinMode(IL1Pin, OUTPUT);
    pinMode(IL2Pin, OUTPUT);
    pinMode(ignitionCtlPin, OUTPUT);
    
}



//i2C Device Addresses
const uint8_t Vout2address = 0x49;
const uint8_t Vout3address = 0x4B;

/****************************************************************/
/*                         Boolean Variables                    */

bool greenLEDstate      = false;
bool redLEDstate        = true;
bool IH1State           = false;
bool IH2State           = false;
bool IL1State           = false;
bool IL2State           = false;
bool LIN1Switch         = false;
bool LIN2Switch         = true;
bool P10or19Switch      = false;
bool P15or18Switch      = false;
bool U1though8Enable    = true;
bool U9though16Enable   = true;
bool CAN1Switch         = true;
bool CAN2Switch         = false;
bool U1U2P0ASwitch      = true;
bool U3U4P0ASwitch      = true;
bool U5U6P0ASwitch      = true;
bool U7U8P0ASwitch      = true;
bool U9U10P0ASwitch     = true;
bool U11U12P0ASwitch    = true;
bool U13U14P0ASwitch    = true;
bool U15U16P0ASwitch    = true;
bool CAN0term           = true;
bool CAN1term           = true;
bool CAN2term           = true;
bool LINmaster          = false;
bool PWM1Out            = true;
bool PWM2Out            = true;
bool PWM3Out            = true;
bool PWM4Out            = true;
bool ignitionCtlState   = false;


/**********************************************************************/
/*   Definitions for Digital Potentiometer Terminal Connections       */
const uint8_t TCON_B_ONLY      = 1;
const uint8_t TCON_WIPER_ONLY  = 2;
const uint8_t TCON_WIPER_AND_B = 3;
const uint8_t TCON_A_ONLY      = 4;
const uint8_t TCON_A_AND_B     = 5;
const uint8_t TCON_WIPER_AND_A = 6;
const uint8_t TCON_CONNECT_ALL = 7;



/****************************************************************/
/*                    Quadrature Knob Setup                     */

Encoder knob(28, 25);
int32_t currentKnob = 0;

int knobLowLimit = 0;
int knobHighLimit = 255;
int knobJump = 1;





/**********************************************************************/
/*   Utility functions to manipulate Ports                            */

uint8_t setTerminationSwitches() {
  //Set the termination Switches of U29 on Rev 3

  uint8_t terminationSettings =  CAN0term | CAN1term << 1 | CAN2term << 2 |  LINmaster << 3 | 
                                 PWM1Out << 4 | PWM2Out << 5 | PWM3Out << 6 | PWM4Out << 7;
  digitalWrite(CStermPin, LOW);
  SPI.transfer(terminationSettings);
  digitalWrite(CStermPin, HIGH);
  return terminationSettings;
}

void getTerminationSwitches(uint8_t terminationSettings) {
  //Set the termination Switches for U21
  CAN0term  = (terminationSettings & 0b00000001) >> 0;
  CAN1term  = (terminationSettings & 0b00000010) >> 1;
  CAN2term  = (terminationSettings & 0b00000100) >> 2;
  LINmaster = (terminationSettings & 0b00001000) >> 3;
  PWM1Out   = (terminationSettings & 0b00010000) >> 4;
  PWM2Out   = (terminationSettings & 0b00100000) >> 5;
  PWM3Out   = (terminationSettings & 0b01000000) >> 6;
  PWM4Out   = (terminationSettings & 0b10000000) >> 7;
}


uint16_t setConfigSwitches() {
  //Set the termination Switches of U21 on Rev 3 based on the boolean values of the variables representing the GPIO pins of U21

  uint16_t configSwitchSettings =  
              LIN1Switch | LIN2Switch  << 1 | P10or19Switch << 2 |  P15or18Switch << 3 | U1though8Enable << 4 | U9though16Enable << 5 |
              CAN1Switch << 6 | CAN2Switch << 7 | U1U2P0ASwitch << 8 |  U3U4P0ASwitch  << 9 |  U5U6P0ASwitch  << 10 | U7U8P0ASwitch << 11 |
              U9U10P0ASwitch << 12 | U11U12P0ASwitch << 13 | U13U14P0ASwitch << 14 | U15U16P0ASwitch << 15;
  ConfigExpander.writeGPIOAB(configSwitchSettings);
  return configSwitchSettings;
}

void getConfigSwitches(uint16_t configSwitchSettings) {
  //get the termination Switches for U21 from the config switch settings. 
  //Requires a 16 bit number that represents the GPIO Pins on chip U21
  //the Booleans must be previously declared
  
  LIN1Switch        = (configSwitchSettings & 0b0000000000000001) >> 0;
  LIN2Switch        = (configSwitchSettings & 0b0000000000000010) >> 1;
  P10or19Switch     = (configSwitchSettings & 0b0000000000000100) >> 2;
  P15or18Switch     = (configSwitchSettings & 0b0000000000001000) >> 3;
  U1though8Enable   = (configSwitchSettings & 0b0000000000010000) >> 4;
  U9though16Enable  = (configSwitchSettings & 0b0000000000100000) >> 5;
  CAN1Switch        = (configSwitchSettings & 0b0000000001000000) >> 6;
  CAN2Switch        = (configSwitchSettings & 0b0000000010000000) >> 7;
  U1U2P0ASwitch     = (configSwitchSettings & 0b0000000100000000) >> 9;
  U3U4P0ASwitch     = (configSwitchSettings & 0b0000001000000000) >> 9;
  U5U6P0ASwitch     = (configSwitchSettings & 0b0000010000000000) >> 10;
  U7U8P0ASwitch     = (configSwitchSettings & 0b0000100000000000) >> 11;
  U9U10P0ASwitch    = (configSwitchSettings & 0b0001000000000000) >> 12;
  U11U12P0ASwitch   = (configSwitchSettings & 0b0010000000000000) >> 13;
  U13U14P0ASwitch   = (configSwitchSettings & 0b0100000000000000) >> 14;
  U15U16P0ASwitch   = (configSwitchSettings & 0b1000000000000000) >> 15;
}


/********************************************************************/
/*               Begin Define Settings                              */
/*
 * To add another setting, update the numSettings, populate the settingNames and settingPins array and add 
 * new knob limits. The knob limits are used for value checking. These are for numerical Settings.
 */


#define numSettings  80
char settingNames[numSettings][40] = {
  "Nothing Selected", //0
  "Digital Potentiometer  1 Wiper", //1
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
  "Digital Potentiometer 16 Wiper", //16

  "Vout2-A", //17
  "Vout2-B",
  "Vout2-C",
  "Vout2-D",
  "Vout2-E",
  "Vout2-F",
  "Vout2-G",
  "Vout2-H", //24

  "U1 & U2 P0A", //25
  "U3 & U4 P0A", 
  "U5 & U5 P0A", 
  "U7 & U2 P0A", 
  "U9 & U10 P0A", 
  "U11 & U12 P0A", 
  "U13 & U14 P0A", 
  "U15 & U16 P0A", //32
  
  "PWM 1", //33
  "PWM 2",
  "PWM 3",
  "PWM 4", //36

  "Port 10 or 19", //37
  "Port 15 or 18",
  "CAN1 or J1708",
  "Port 31 & 32 or CAN2",
  "CAN0 Termination Resistor",
  "CAN1 Termination Resistor",
  "CAN2 Termination Resistor",
  "LIN Master Pullup Resistor", //44

  "12V Out 1 (H-Bridge)", //45
  "12V Out 2 (H-Bridge)",
  "Ground Out 1 (H-Bridge)",
  "Ground Out 2 (H-Bridge)", //48

  "High Voltage Adjustable Output", //49
  "Ignition Relay",
  
  "Dig. Pot.  1 Terminal Connect", //51
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
  "Dig. Pot. 16 Terminal Connect", //66

  "PWM1 Connect", //67
  "PWM2 Connect",
  "PWM3 Connect",
  "PWM4 Connect",
  "LIN to Shield Connect", //U21 GPA0
  "LIN to Port 16 Connect", //U21 GPA1
  "U28 (U1-U8)  P0A Enable"
  "U31 (U9-U16) P0A Enable" //74
  
  "Digital Potentiometer 28 Wiper", //75
  "Digital Potentiometer 29 Wiper",
  "Digital Potentiometer 30 Wiper",
  
  "Dig. Pot. 28 Terminal Connect",
  "Dig. Pot. 29 Terminal Connect",
  "Dig. Pot. 30 Terminal Connect", //80
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

  "Ports  1 and 2",
  "Ports  3 and 4",
  "Ports  5 and 6",
  "Ports  7 and 8",
  "Ports  9 and 10",
  "Ports 11 and 12",
  "Ports 13 and 14",
  "Ports 15 and 16",

  "Ports 13 (J24-13) and 31 (J18-15)",
  "Ports 14 (J24-14) and 32 (J18-16)",
  "Port 27 (J18-10)",
  "Port 17 (J18-1)",
  
  "(J24-10)",
  "(J24-15)",
  "(J24-17 & J24-18)",
  "(J18-15 and J18-16)",
  "R44",
  "R45",
  "R46",
  "R59",

  "Port 26 (J18-10)",
  "Port 11 (J24-11)",
  "Port 17 (J18- 1)",
  "Port 12 (J24-12)",

  "(J24-19 and J18-11)",
  "(J24-20)",
  
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

  "Port 13 (J24-13)",
  "Port 14 (J24-14)",
  "Port 27 (J18-10)",
  "Port 17 (J18- 1)",
  "(J10- 5)",
  "Port 16 (J24-16)",
  "(J24-1 to J24-8)",
  "(J24-9 to J24-16)",
  
  "Port 28 (J18-12)",
  "Port 29 (J18-13)",
  "Port 30 (J18-14)",
  
  "Port 28 (J18-12)",
  "Port 29 (J18-13)",
  "Port 30 (J18-14)"
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
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum > 32 && settingNum <= 36) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum > 36 && settingNum <= 48) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 49) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum == 50) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
 
  else if (settingNum >= 51 && settingNum <= 66) {
    knobLowLimit = 0;
    knobHighLimit = 7;
    knobJump = 1;
  }
  else if (settingNum >= 67 && settingNum <= 74) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  if (settingNum > 74 && settingNum <= 77) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum >= 77 && settingNum <= 80) {
    knobLowLimit = 0;
    knobHighLimit = 7;
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
    uint8_t CShvadjPin = 57;
    MCP41HV_SetWiper(CShvadjPin, HVoutAdjValue);
  }
  else if (settingNum >= 50 && settingNum <= 65) {
    if (settingNum == 60){
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
    if (LIN16Connect) {
      MCP41HVExtender_SetTerminals(15, potTCONSettings[15]);
    }
    setConnectionSwitches();
    
  }
  
  listSetting(settingNum);

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
      strcpy(displayBuffer, " 1 (TCON_B_ONLY)");
      break;
    case TCON_WIPER_ONLY:
      strcpy(displayBuffer, "2 (TCON_WIPER_ONLY)");
      break;
    case TCON_WIPER_AND_B:
      strcpy(displayBuffer, "3 (TCON_WIPER_AND_B)");
      break;
    case TCON_A_ONLY:
      strcpy(displayBuffer, "4 (TCON_A_ONLY)");
      break;
    case TCON_A_AND_B :
      strcpy(displayBuffer, "5 (TCON_A_AND_B)");
      break;
    case TCON_WIPER_AND_A :
      strcpy(displayBuffer, "6 (TCON_WIPER_AND_A)");
      break;
    case TCON_CONNECT_ALL:
      strcpy(displayBuffer, "7 (TCON_CONNECT_ALL)");
      break;
    default:
      strcpy(displayBuffer, "Nothing connected");
      break;
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
}


/****************************************************************/
/*   Begin Function Calls for Digital Potentiometer             */
  
uint8_t MCP41HVExtender_SetTerminals(uint8_t pin, uint8_t TCON_Value) {
  PotExpander.writeGPIOAB(~(1 << pin));
  delay(1);
  SPI.transfer(0x40); //Write to TCON Register
  SPI.transfer(TCON_Value + 8);
  SPI.transfer(0x4C); //Read Command
  uint8_t result = SPI.transfer(0xFF); //Read Terminal Connection (TCON) Register
  PotExpander.writeGPIOAB(0xFF);
  return result & 0x0F;
}

uint8_t MCP41HVExtender_SetWiper(uint8_t pin, uint8_t potValue)
{
  PotExpander.writeGPIOAB(~(1 << pin));
  delay(1);
  SPI.transfer(0x00); //Write to wiper Register
  SPI.transfer(potValue);
  SPI.transfer(0x0C); //Read command
  uint8_t result = SPI.transfer(0xFF); //Read Wiper Register
  PotExpander.writeGPIOAB(0xFF);
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