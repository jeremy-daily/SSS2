/*
 * Smart Sensor Simulator 2
 * 
 * Arduino Sketch to define the board pins and features
 * Uses the Teensy 3.6
 * 
 * Written By Dr. Jeremy S. Daily
 * The University of Tulsa
 * Department of Mechanical Engineering
 * 
 * 06 May 2017
 * 
 * Released under the MIT License
 *
 * Copyright (c) 2017        Jeremy S. Daily
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
*/

#include <SPI.h>
#include <i2c_t3.h>
#include "Adafruit_MCP23017.h" 


#define SSS2_BOARD_REVISION 5
#define J1708 Serial3
#define LIN Serial2
#define linRXpin 4

String make = "SYNER";
String model = "SSS2";
String revision = "05";
String serial_number ="XXXX";
String componentID = make + "*" + model + "-" + revision + "*" + serial_number + "*UNIVERSAL";

Adafruit_MCP23017 ConfigExpander; //U21
Adafruit_MCP23017 PotExpander; //U33
uint8_t terminationSettings;



/****************************************************************/
/*                         Pin Defintions                       */
const int8_t greenLEDpin       = 2;
const int8_t redLEDpin         = 5;
const int8_t CSCANPin          = 31;
const int8_t INTCANPin         = 21;
const int8_t CSconfigAPin      = 26;
const int8_t CSconfigBPin      = 27;
const int8_t buttonPin         = 24;
const int8_t encoderAPin       = 28;
const int8_t encoderBPin       = 25;
const int8_t CStouchPin        = 26;
const int8_t IH1Pin            = 35;
const int8_t IH2Pin            = 36;
const int8_t IL1Pin            = 37;
const int8_t IL2Pin            = 38;
const int8_t ignitionCtlPin    = 39;

const uint8_t numPWMs = 6;
const int8_t PWMPins[numPWMs]     = {16,17,22,23,29,30};
uint16_t pwmValue[numPWMs] = {25,100,19,222,100,100};
uint16_t pwmFrequency[numPWMs] = {200,200,200,200,200,200};

const uint8_t numADCs = 6;
const int8_t analogInPins[numADCs]= {A21,A22,A0,A1,A6,A11};
int analog_display_period = 100;

//int analogPeriod = 100; //milliseconds

/*
 * Begin Default Settings
*/
const uint8_t numSPIpots = 16;
uint8_t  SPIpotWiperSettings[numSPIpots] ={21,22,22,56,56,0,10,56,56,30,56,56,56,56,56,255};
uint8_t  SPIpotTCONSettings[numSPIpots] ={3,3,3,7,7,3,3,7,7,7,0,0,7,7,7,0};

const uint8_t numI2Cpots = 3;
uint8_t  I2CpotWiperSettings[numI2Cpots] = {75,150,225};
uint8_t  I2CpotTCONSettings[numI2Cpots] = {7,7,7};
const uint8_t I2CpotAddr[numI2Cpots] = {0x3C,0x3F,0x3D};

const uint8_t numDACs = 8;
uint16_t DAC2value[numDACs] = {0,0,0,0,512,512,0,0};
uint16_t DAC3value[numDACs] {0,0,0,0,4095,4095,4095,4095}; 



uint8_t HVoutAdjValue = 168;
uint8_t connectionSettings = 0b11001110;
const uint8_t HVoutAdjAddr = 0x3E;

/**********************************************************************/
/*   Definitions for i2C addresses for the SSS2 Chips                 */
//i2C Device Addresses
const uint8_t Vout2address = 0x49;

const uint8_t potExpanderAddr = 7;
const uint8_t configExpanderAddr = 3;

/*
 * End Default Settings
*/

#define DEBUG_ON  1
#define DEBUG_OFF 0

const uint16_t componentIDAddress = 1000;

//set up a display buffer
char displayBuffer[100];

void setPinModes(){
    pinMode(greenLEDpin,     OUTPUT);
    pinMode(redLEDpin,       OUTPUT);
    pinMode(CSCANPin,        OUTPUT);
    pinMode(CSconfigAPin,    OUTPUT);
    pinMode(CSconfigBPin,    OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(IH1Pin,          OUTPUT);
    pinMode(IH2Pin,          OUTPUT);
    pinMode(IL1Pin,          OUTPUT);
    pinMode(IL2Pin,          OUTPUT);
    pinMode(ignitionCtlPin,  OUTPUT);
    pinMode(linRXpin,        INPUT);
     
    digitalWrite(CSconfigAPin, HIGH);
    digitalWrite(CSconfigBPin, HIGH);
    digitalWrite(redLEDpin,    HIGH);
    digitalWrite(greenLEDpin,  LOW);
     digitalWrite(IH2Pin,LOW);
    digitalWrite(IL1Pin,LOW);
    digitalWrite(IL2Pin,LOW);
    digitalWrite(ignitionCtlPin,LOW);
    
    uint8_t i;
    for (i = 0; i < numPWMs; i++) pinMode(PWMPins[i], OUTPUT);
    for (i = 0; i < numADCs; i++) pinMode(analogInPins[i], INPUT);
    
}

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
bool CAN0term1          = false;
bool CAN1term1          = false;
bool CAN2term1          = false;
bool LINmaster          = false;
bool PWM1Out            = true;
bool PWM2Out            = true;
bool PWM3Out            = true;
bool PWM4Out            = true;
bool PWM5Out            = true;
bool PWM6Out            = true;
bool PWM4Out_28         = false;
bool CAN1out            = true;
bool ignitionCtlState   = false;


/**********************************************************************/
/*   Utility functions to manipulate Ports                            */

uint8_t setTerminationSwitches() {
  //Set the termination Switches of U29 on Rev 3

  uint8_t terminationSettings =  uint8_t( CAN0term1 | CAN1term1 << 1 | CAN2term1 << 2 |  PWM4Out_28 << 3 | 
                                 CAN1out << 4 | CAN1out << 5 | PWM5Out << 6 | PWM6Out << 7);
  digitalWrite(CSconfigAPin, LOW);
  SPI.transfer(terminationSettings);
  digitalWrite(CSconfigAPin, HIGH);
  return terminationSettings;
}

void getTerminationSwitches(uint8_t terminationSettings) {
  CAN0term  = (terminationSettings & 0b00000001) >> 0;
  CAN1term  = (terminationSettings & 0b00000010) >> 1;
  CAN2term  = (terminationSettings & 0b00000100) >> 2;
  LINmaster = (terminationSettings & 0b00001000) >> 3;
  PWM1Out   = (terminationSettings & 0b00010000) >> 4;
  PWM2Out   = (terminationSettings & 0b00100000) >> 5;
  PWM3Out   = (terminationSettings & 0b01000000) >> 6;
  PWM4Out   = (terminationSettings & 0b10000000) >> 7;
}


uint8_t setPWMSwitches() {
  uint8_t PWMSettings =  uint8_t( CAN0term | CAN1term << 1 | CAN2term << 2 |  LINmaster << 3 | 
                                 PWM1Out << 4 | PWM2Out << 5 | PWM3Out << 6 | PWM4Out << 7);
  digitalWrite(CSconfigBPin, LOW);
  SPI.transfer(PWMSettings);
  digitalWrite(CSconfigBPin, HIGH);
  return PWMSettings;
}

void getPWMSwitches(uint8_t terminationSettings) {
  //Set the termination Switches for U21
  CAN0term1  = (terminationSettings & 0b00000001) >> 0;
  CAN1term1  = (terminationSettings & 0b00000010) >> 1;
  CAN2term1  = (terminationSettings & 0b00000100) >> 2;
  LINmaster  = (terminationSettings & 0b00001000) >> 3;
  PWM1Out    = (terminationSettings & 0b00010000) >> 4;
  PWM2Out    = (terminationSettings & 0b00100000) >> 5;
  PWM3Out    = (terminationSettings & 0b01000000) >> 6;
  PWM4Out    = (terminationSettings & 0b10000000) >> 7;
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
