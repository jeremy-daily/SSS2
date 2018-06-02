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
 * 22 May 2017
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

#define USE_SPI1
#define SSS2_BOARD_REVISION 5
#define J1708 Serial3
#define LIN Serial2
#define linRXpin 9

String make = "SYNER";
String model = "SSS2";
String revision = "05";
String serial_number ="XXXX";
String componentID = make + "*" + model + "-" + revision + "*" + serial_number + "*UNIVERSAL";


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
uint16_t pwmValue[numPWMs] = {500,1000,1500,2000,2500,3000};
uint16_t pwmFrequency[numPWMs] = {200,210,220,230,240,250};

const uint8_t numADCs = 6;
const int8_t analogInPins[numADCs]= {A21,A1,A0,A6,A11,A22};
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

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];        

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
    pinMode(INTCANPin,        INPUT);
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
bool IL1State           = true;
bool IL2State           = false;
bool LIN1Switch         = false;
bool LIN2Switch         = true;
bool P10or19Switch      = false;
bool P15or18Switch      = false;
bool U1though8Enable    = true;
bool U9though16Enable   = true;
bool CAN1Switch         = true;
bool CAN2Switch         = false;
bool U1U2P0ASwitch      = false;
bool U3U4P0ASwitch      = false;
bool U5U6P0ASwitch      = false;
bool U7U8P0ASwitch      = false;
bool U9U10P0ASwitch     = false;
bool U11U12P0ASwitch    = false;
bool U13U14P0ASwitch    = false;
bool U15U16P0ASwitch    = false;
bool CAN0term           = true;
bool CAN1term           = true;
bool CAN2term           = true;
bool CAN0term1          = false;
bool CAN1term1          = false;
bool CAN2term1          = false;
bool LINmaster          = false;
bool PWM1Out            = false;
bool PWM2Out            = false;
bool PWM3Out            = true;
bool PWM4Out            = false;
bool PWM5Out            = false;
bool PWM6Out            = false;
bool PWM4Out_28         = false;
bool CAN1out            = false;
bool CAN2out            = false;
bool ignitionCtlState   = false;



