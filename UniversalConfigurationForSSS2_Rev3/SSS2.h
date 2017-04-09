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
#define DEBUG_ON  1
#define DEBUG_OFF 0

/**********************************************************************/
/*   Definitions for i2C addresses for the SSS2 Chips                 */

uint8_t pot28TCONSetting = 7;
uint8_t pot29TCONSetting = 7;
uint8_t pot30TCONSetting = 7;
uint8_t pot28WiperSetting = 75;
uint8_t pot29WiperSetting = 150;
uint8_t pot30WiperSetting = 225;
uint8_t potWiperSettings[16] ={21,22,22,56,56,0,10,56,56,0,56,56,56,56,56,255};
uint8_t potTCONSettings[16] ={3,3,3,7,7,3,3,7,7,3,0,0,7,7,7,0};
uint16_t DAC2value[8] = {0,0,0,0,512,512,0,0};
uint16_t pwm1value = 0;
uint16_t pwm2value = 100;
uint16_t pwm3value = 19;
uint16_t pwm4value = 222; //Set this for Bendix
uint8_t HVoutAdjValue = 168;
uint8_t connectionSettings = 0b11001110;
uint16_t pwm1Frequency = 200;
uint16_t pwm2Frequency = 200;
uint16_t pwm3Frequency = 200;
uint16_t pwm4Frequency = 200;

/* uint8_t HS1state = 0;
uint8_t HS2state = 0;
uint8_t LS1state = 0;
uint8_t LS2state = 1; //Needed for Bendix 
 */
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


//set up a display buffer
char displayBuffer[100];


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
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(CStouchPin, OUTPUT);
    pinMode(IH1Pin, OUTPUT);
    pinMode(IH2Pin, OUTPUT);
    pinMode(IL1Pin, OUTPUT);
    pinMode(IL2Pin, OUTPUT);
    pinMode(ignitionCtlPin, OUTPUT);
    pinMode(A21,INPUT);
     
    digitalWrite(CSCANPin, HIGH);
    digitalWrite(CSdispPin, HIGH);
    digitalWrite(CStouchPin, HIGH);
    digitalWrite(CStermPin, HIGH);
    
    
}

/**********************************************************************/
/*   Definitions for i2C addresses for the SSS2 Chips                 */
//i2C Device Addresses
const uint8_t Vout2address = 0x49;
const uint8_t U34addr = 0x3C;
const uint8_t U36addr = 0x3F;
const uint8_t U37addr = 0x3D;
const uint8_t U24addr = 0x3E;



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
bool U1though8Enable    = false;
bool U9though16Enable   = false;
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

uint8_t terminationSettings;


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
      strcpy(displayBuffer, "0 Nothing connected");
      break;
  }
}


/**********************************************************************/
/*   Utility functions to manipulate Ports                            */

uint8_t setTerminationSwitches() {
  //Set the termination Switches of U29 on Rev 3

  uint8_t terminationSettings =  uint8_t( CAN0term | CAN1term << 1 | CAN2term << 2 |  LINmaster << 3 | 
                                 PWM1Out << 4 | PWM2Out << 5 | PWM3Out << 6 | PWM4Out << 7);
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
              LIN1Switch | LIN2Switch  << 1 | P10or19Switch << 2 |  P15or18Switch << 3 | !U1though8Enable << 4 | !U9though16Enable << 5 |
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
  return result & 0x07;
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

uint8_t MCP41HVI2C_SetTerminals(uint8_t addr, uint8_t TCON_Value) {
  
  Wire.beginTransmission(addr); 
  Wire.write(byte(0x40));            // sends instruction byte  
  Wire.write(0xF8 | TCON_Value);             // sends potentiometer value byte  
  Wire.endTransmission();

  Wire.requestFrom(addr,2);    // request 6 bytes from slave device #8
  uint8_t result = Wire.read(); //Read Wiper Register
  result = Wire.read(); //Read Wiper Register
  return result & 0x07;
}

uint8_t MCP41HVI2C_SetWiper(uint8_t addr, uint8_t potValue)
{
  Wire.beginTransmission(addr); 
  Wire.write(byte(0x00));            // sends instruction byte  
  Wire.write(potValue);             // sends potentiometer value byte  
  Wire.endTransmission();

  Wire.requestFrom(addr,2);    // request 6 bytes from slave device #8
  uint8_t result = Wire.read(); //Read Wiper Register, first byt is all 0
  result = Wire.read(); //Read Wiper Register
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

/********************************************************************/
/*               Begin Define Settings                              */
/*
 * To add another setting, update the numSettings, populate the settingNames and settingPins array and add 
 * new knob limits. The knob limits are used for value checking. These are for numerical Settings.
 */


#define numSettings  85
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
  "U28 (U1-U8)  P0A Enable",
  "U31 (U9-U16) P0A Enable", //74
  
  "Digital Potentiometer 28 Wiper", //75
  "Digital Potentiometer 29 Wiper",
  "Digital Potentiometer 30 Wiper",
  
  "Dig. Pot. 28 Terminal Connect",
  "Dig. Pot. 29 Terminal Connect",
  "Dig. Pot. 30 Terminal Connect", //80

  "PWM1 Frequency",
  "PWM2 Frequency",
  "PWM3 Frequency",
  "PWM4 Frequency"
  
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
  "Port 30 (J18-14)",

  "Port 13 (J24-13)",
  "Port 14 (J24-14)",
  "Port 27 (J18-10)",
  "Port 17 (J18-1)"
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
    knobHighLimit = 4096;
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
  else if (settingNum > 74 && settingNum <= 77) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum >= 77 && settingNum <= 80) {
    knobLowLimit = 0;
    knobHighLimit = 7;
    knobJump = 1;
  }
  else if (settingNum >= 81 && settingNum <= 84) {
    knobLowLimit = 0;
    knobHighLimit = 32768;
    knobJump = 1;
  }
  else {
    knobLowLimit = 1;
    knobHighLimit = numSettings-1;
    knobJump = 1;
  }

  // Serial.print("Low Limit: ");
  // Serial.println(knobLowLimit);
  // Serial.print("High Limit: ");
  // Serial.println(knobHighLimit);
}

int16_t setSetting(uint8_t settingNum, int settingValue, bool debugDisplay) {
  
  
  if (debugDisplay){
    Serial.print(settingNum);
    Serial.print(", ");
    Serial.print(settingNames[settingNum]);
    Serial.print(", ");
    Serial.print(settingPins[settingNum]);
    Serial.print(" = ");
  }
  if (settingNum > 0 && settingNum <= 16){
    if (settingValue > -1) potWiperSettings[settingNum - 1] = settingValue; 
    uint8_t position = MCP41HVExtender_SetWiper(settingNum - 1, 
                                                potWiperSettings[settingNum - 1]);
    if (debugDisplay) {
        Serial.print(position);
        Serial.print(", ");
        Serial.println(potWiperSettings[settingNum - 1]);
    }
    return position;
  }
  else if (settingNum > 16 && settingNum <= 24) {
    if (settingValue > -1) DAC2value[settingNum - 17] = settingValue;
    setDAC(DAC2value[settingNum - 17], settingNum - 17, Vout2address);
    if (debugDisplay) Serial.println(DAC2value[settingNum - 17]); 
    return DAC2value[settingNum - 17];
  }
  else if (settingNum == 25){
    if (settingValue > -1) U1U2P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U1U2P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U1U2P0ASwitch;
  }
  else if (settingNum == 26){
    if (settingValue > -1) U3U4P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U3U4P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U3U4P0ASwitch;
  }
  else if (settingNum == 27){
    if (settingValue > -1) U5U6P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U5U6P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U5U6P0ASwitch;
  }
  else if (settingNum == 28){
    if (settingValue > -1) U7U8P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U7U8P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U7U8P0ASwitch;
  } 
  else if (settingNum == 29){
    if (settingValue > -1) U9U10P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U9U10P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U9U10P0ASwitch;
  }  
  else if (settingNum == 30){
    if (settingValue > -1) U11U12P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U11U12P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U11U12P0ASwitch;
  }  
  else if (settingNum == 31){
    if (settingValue > -1) U13U14P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U13U14P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U13U14P0ASwitch;
  }   
  else if (settingNum == 32){
    if (settingValue > -1) U15U16P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U15U16P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U15U16P0ASwitch;
  }
  else if (settingNum == 33){
    if (settingValue > -1) pwm1value = uint16_t(settingValue);
    analogWrite(PWM1Pin,pwm1value);
    if (debugDisplay) Serial.println(pwm1value);
    return pwm1value;
  }
  else if (settingNum == 34){
    if (settingValue > -1) pwm2value = uint16_t(settingValue);
    analogWrite(PWM2Pin,pwm2value);
    if (debugDisplay) Serial.println(pwm2value);
    return pwm1value;
  }
  else if (settingNum == 35){
    if (settingValue > -1) pwm3value = uint16_t(settingValue);
    analogWrite(PWM3Pin,pwm3value);
    if (debugDisplay) Serial.println(pwm3value);
    return pwm3value;
  }
  else if (settingNum == 36){
    if (settingValue > -1) pwm4value = uint16_t(settingValue);
    analogWrite(PWM4Pin,pwm4value);
    if (debugDisplay) Serial.println(pwm4value);
    return pwm4value;
  }
  else if (settingNum == 37){
    if (settingValue > -1) P10or19Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(P10or19Switch);
        Serial.println(displayBuffer);
    }
    return P10or19Switch;
  }
  else if (settingNum == 38){
    if (settingValue > -1) P15or18Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(P15or18Switch);
        Serial.println(displayBuffer);
    }
    return P15or18Switch;
  } 
  else if (settingNum == 39){
    if (settingValue > -1) CAN1Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(CAN1Switch);
        Serial.println(displayBuffer);
    }
    return CAN1Switch;
  }  
  else if (settingNum == 40){
    if (settingValue > -1) CAN2Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(CAN2Switch);
        Serial.println(displayBuffer);
    }
    return CAN2Switch;
  }
  else if (settingNum == 41){
    if (settingValue > -1) CAN0term = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(CAN0term);
        Serial.println(displayBuffer);
    }
    return CAN0term;
  } 
  else if (settingNum == 42) {
    if (settingValue > -1) CAN1term = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(CAN1term);
        Serial.println(displayBuffer);
    }
    return CAN1term;
  } 
  else if (settingNum == 43){
    if (settingValue > -1) CAN2term = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(CAN2term);
        Serial.println(displayBuffer);
    }
    return CAN2term;
  }  
  else if (settingNum == 44) {
    if (settingValue > -1) LINmaster = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(LINmaster);
        Serial.println(displayBuffer);
    }
    return LINmaster;
  }  
  else if (settingNum == 45)  {
    if (settingValue > -1) IH1State = boolean(settingValue);
    if (IH1State) PWM3Out = false;
    else PWM3Out = true;
    
    setTerminationSwitches();
    digitalWrite(IH1Pin,IH1State);
    if (debugDisplay) {
        connectionString(IH1State);
        Serial.println(displayBuffer);
    }
    return IH1State;
  }  
  else if (settingNum == 46) {
    if (settingValue > -1) IH2State = boolean(settingValue);
    if (IH2State) MCP41HVExtender_SetTerminals(11, 0); //Turn off all terminals on Pot 11
    else MCP41HVExtender_SetTerminals(11, potTCONSettings[10]); //Reset all terminals on Pot 11
    digitalWrite(IH2Pin,IH2State);
    if (debugDisplay) {
        connectionString(IH2State);
        Serial.println(displayBuffer);
    }
    return IH2State;
  }
  else if (settingNum == 47){
    if (settingValue > -1) IL1State = boolean(settingValue);
    if (IL1State) PWM4Out = false;
    else PWM4Out = true; 
    setTerminationSwitches(); //Turn off PWM4
    digitalWrite(IL1Pin,IL1State);
    if (debugDisplay) {
        connectionString(IL1State);
        Serial.println(displayBuffer);
    }
    return IL1State;
  }
  else if (settingNum == 48) {
    if (settingValue > -1) IL2State = boolean(settingValue);
    if (IL2State) MCP41HVExtender_SetTerminals(12, 0); //Turn off all terminals on Pot 12
    else MCP41HVExtender_SetTerminals(12, potTCONSettings[11]); //Reset all terminals 
    digitalWrite(IL2Pin,IL2State);
    if (debugDisplay) {
        connectionString(IL2State);
        Serial.println(displayBuffer);
    }
    return IL2State;
  }
  else if (settingNum == 49) {
    if (settingValue > -1) HVoutAdjValue = uint8_t(settingValue);
    uint8_t position = MCP41HVI2C_SetWiper(U24addr,HVoutAdjValue);
    if (debugDisplay) {
        Serial.print(position);
        Serial.print(", ");
        Serial.println(HVoutAdjValue);
    }
    return position;
  }
  else if (settingNum == 50){
    if (settingValue > -1) ignitionCtlState = boolean(settingValue);
    digitalWrite(ignitionCtlPin,ignitionCtlState);
    if (debugDisplay) {
        connectionString(ignitionCtlState);
        Serial.println(displayBuffer);
    }
    return ignitionCtlState;
  } 
  else if (settingNum >  50 && settingNum <= 66) {
    if (settingValue > -1) potTCONSettings[settingNum - 51] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVExtender_SetTerminals(settingNum - 51, potTCONSettings[settingNum - 51]);
    if (debugDisplay) {
        Serial.print(potTCONSettings[settingNum - 51]);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 67){
    if (settingValue > -1) PWM1Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM1Out);
        Serial.println(displayBuffer);
    }
    return PWM1Out;
  } 
  else if (settingNum == 68){
    if (settingValue > -1) PWM2Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM2Out);
        Serial.println(displayBuffer);
    }
    return PWM2Out;
  }  
  else if (settingNum == 69){
    if (settingValue > -1) PWM3Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM3Out);
        Serial.println(displayBuffer);
    }
    return PWM3Out;
  }  
  else if (settingNum == 70) {
    if (settingValue > -1) PWM4Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM4Out);
        Serial.println(displayBuffer);
    }
    return PWM4Out;
  } 
  else if (settingNum == 71) {
    if (settingValue > -1) LIN1Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(LIN1Switch);
        Serial.println(displayBuffer);
    }
    return LIN1Switch;
  }
  else if (settingNum == 72) {
    if (settingValue > -1) LIN2Switch = boolean(settingValue);
    setConfigSwitches();
    if (LIN2Switch) MCP41HVExtender_SetTerminals(15, 0);
    else MCP41HVExtender_SetTerminals(15, potTCONSettings[15]);
    if (debugDisplay) {
        connectionString(LIN2Switch);
        Serial.println(displayBuffer);
    }
    return LIN2Switch;
  }
  else if (settingNum == 73) {
    if (settingValue > -1) U1though8Enable = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U1though8Enable);
        Serial.println(displayBuffer);
    }
    return U1though8Enable;
  } 
  else if (settingNum == 74){
    if (settingValue > -1) U9though16Enable = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U9though16Enable);
        Serial.println(displayBuffer);
    }
    return U9though16Enable;
  } 
  else if (settingNum == 75){
    if (settingValue > -1) pot28WiperSetting = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(U34addr,pot28WiperSetting);
    if (debugDisplay) {
        Serial.print(pot28WiperSetting);
        Serial.print(", ");
        Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 76){
    if (settingValue > -1) pot29WiperSetting = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(U36addr,pot29WiperSetting);
    if (debugDisplay) {
        Serial.print(pot29WiperSetting);
        Serial.print(", ");
        Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 77){
    if (settingValue > -1) pot30WiperSetting = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(U37addr,pot30WiperSetting);
    if (debugDisplay) {
        Serial.print(pot30WiperSetting);
        Serial.print(", ");
        Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 78){
    if (settingValue > -1) pot28TCONSetting = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(U34addr,pot28TCONSetting);
    if (debugDisplay) {
        Serial.print(pot28TCONSetting);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 79){
    if (settingValue > -1) pot29TCONSetting = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(U36addr,pot29TCONSetting);
    if (debugDisplay) {
        Serial.print(pot29TCONSetting);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 80){
    if (settingValue > -1) pot30TCONSetting = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(U37addr,pot30TCONSetting);
    if (debugDisplay) {
        Serial.print(pot30TCONSetting);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }

  else if (settingNum == 81){
    if (settingValue > -1)  pwm1Frequency = uint16_t(settingValue);
    analogWriteFrequency(PWM1Pin, pwm1Frequency);
    analogWrite(PWM1Pin,pwm1value);
    analogWrite(PWM2Pin,pwm2value);
    if (debugDisplay) {
        Serial.println(pwm1Frequency);
    }
    return pwm1Frequency;
  }
  else if (settingNum == 82){
    if (settingValue > -1)  pwm2Frequency = uint16_t(settingValue);
    analogWriteFrequency(PWM2Pin, pwm2Frequency);
    analogWrite(PWM2Pin,pwm2value);
    analogWrite(PWM1Pin,pwm1value);
    if (debugDisplay) {
        Serial.println(pwm2Frequency);
    }
    return pwm2Frequency;
  }
  else if (settingNum == 83){
    if (settingValue > -1)  pwm3Frequency = uint16_t(settingValue);
    analogWriteFrequency(PWM3Pin, pwm3Frequency);
    analogWrite(PWM3Pin,pwm3value);
    analogWrite(PWM4Pin,pwm4value);
    if (debugDisplay) {
        Serial.println(pwm3Frequency);
    }
    return pwm3Frequency;
  }
  else if (settingNum == 84){
    if (settingValue > -1)  pwm4Frequency = uint16_t(settingValue);
    analogWriteFrequency(PWM4Pin, pwm4Frequency);
    analogWrite(PWM4Pin,pwm4value);
    analogWrite(PWM3Pin,pwm3value);
    if (debugDisplay) {
        Serial.println(pwm4Frequency);
    }
    return pwm4Frequency;
  }
  else return -1;
  
  
}



/*               End Define Settings                              */
/******************************************************************/






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
  EEPROM.put(HS1Address, IH1State);
  EEPROM.put(HS2Address, IH2State);
  EEPROM.put(LS1Address, IL1State);
  EEPROM.put(LS2Address, IL2State);
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
  EEPROM.get(HS1Address, IH1State);
  EEPROM.get(HS2Address, IH2State);
  EEPROM.get(LS1Address, IL1State);
  EEPROM.get(LS2Address, IL2State);
  EEPROM.get(HVoutAdjAddress, HVoutAdjValue);
  EEPROM.get(componentIDAddress, componentID);
 // EEPROM.get(number_of_CAN_msgsAddress, number_of_CAN_msgs);
  EEPROM.get(terminationSettingsAddress, terminationSettings);
  getTerminationSwitches(terminationSettings);
  EEPROM.get(connectionSettingsAddress, connectionSettings);
  
  EEPROM.get(programmedForAddress, programmedFor);
  EEPROM.get(programmedByAddress, programmedBy);
  EEPROM.get(programmedDateAddress, programDate);
  //EEPROM.get(softwareVersionAddress, softwareVersion);
  EEPROM.get(programNotesAddress, programNotes);
}


