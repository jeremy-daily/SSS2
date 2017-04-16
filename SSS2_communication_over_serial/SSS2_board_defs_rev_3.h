#define SSS2_BOARD_REVISION 3
#define J1708 Serial3
#define LIN Serial1

String make = "SYNER";
String model = "SSS2";
String revision = "03";
String serial_number ="XXXX";
String componentID = make + "*" + model + "-" + revision + "*" + serial_number + "*UNIVERSAL";

/****************************************************************/
/*                         Pin Defintions                       */
const int8_t greenLEDpin       = 2;
const int8_t redLEDpin         = 5;
const int8_t CSCANPin          = 15;
const int8_t CSanalogPin       = 20;
const int8_t CStermPin         = 21;
const int8_t CSconfigAPin      = 21;
const int8_t CSconfigBPin      = -1;
const int8_t buttonPin         = 24;
const int8_t encoderAPin       = 28;
const int8_t encoderBPin       = 25;
const int8_t CStouchPin        = 26;
const int8_t IH1Pin            = 35;
const int8_t IH2Pin            = 36;
const int8_t IL1Pin            = 37;
const int8_t IL2Pin            = 38;
const int8_t ignitionCtlPin    = 39;

const uint8_t numPWMs = 4;
const int8_t PWMPins[numPWMs]     = {16,17,22,23};
uint16_t pwmValue[numPWMs] = {25,100,19,222};
uint16_t pwmFrequency[numPWMs] = {200,200,200,200};

const uint8_t numADCs = 1;
const int8_t analogInPins[6]= {A21,A22,A0,A2,A6,A11};
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
const uint8_t Vout3address = -1;

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
    pinMode(CSanalogPin,     OUTPUT);
    pinMode(CStermPin,       OUTPUT);
    pinMode(CSconfigAPin,    OUTPUT);
    pinMode(CSconfigBPin,    OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(CStouchPin,      OUTPUT);
    pinMode(IH1Pin,          OUTPUT);
    pinMode(IH2Pin,          OUTPUT);
    pinMode(IL1Pin,          OUTPUT);
    pinMode(IL2Pin,          OUTPUT);
    pinMode(ignitionCtlPin,  OUTPUT);
     
    if (CSCANPin      > -1) digitalWrite(CSCANPin,     HIGH);
    if (CStouchPin    > -1) digitalWrite(CStouchPin,   HIGH);
    if (CStermPin     > -1) digitalWrite(CStermPin,    HIGH);
    if (CSconfigAPin  > -1) digitalWrite(CSconfigAPin, HIGH);
    if (CSconfigBPin  > -1) digitalWrite(CSconfigBPin, HIGH);
    if (redLEDpin     > -1) digitalWrite(redLEDpin,    HIGH);
    if (greenLEDpin   > -1) digitalWrite(greenLEDpin,  LOW);
    digitalWrite(CSanalogPin,HIGH);
    digitalWrite(CStouchPin,HIGH);
    digitalWrite(IH1Pin,LOW);
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
bool LINmaster          = false;
bool PWM1Out            = true;
bool PWM2Out            = true;
bool PWM3Out            = true;
bool PWM4Out            = true;
bool PWM5Out            = true;
bool PWM6Out            = true;
bool ignitionCtlState   = false;


