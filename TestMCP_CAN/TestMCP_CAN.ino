
// CAN Receive Example
//


#include <i2c_t3.h>
#include "Adafruit_MCP23017.h"

Adafruit_MCP23017 ConfigExpander; //U21
Adafruit_MCP23017 PotExpander; //U33

#include <mcp_can.h>
#include <mcp_can_dfs.h>

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
    
    pinMode(12,INPUT_PULLUP);
}

/**********************************************************************/
/*   Definitions for i2C addresses for the SSS2 Chips                 */
//i2C Device Addresses
const uint8_t Vout2address = 0x49;
const uint8_t U34addr = 0x3C;
const uint8_t U36addr = 0x3F;
const uint8_t U37addr = 0x3D;
const uint8_t U24addr = 0x3E;


long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

#define CAN0_INT 6                             // Set INT to pin 2
MCP_CAN CAN0(CSCANPin);                               // Set CS to pin 10
#define DEBUG_MODE 1;
boolean configed=false;

void setup()
{
  Serial.begin(115200);
  SPI.begin();
   setPinModes();

  
  
 
  PotExpander.begin(7);  //U33
  ConfigExpander.begin(3); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
  }
  pinMode(ignitionCtlPin,OUTPUT);
  digitalWrite(ignitionCtlPin,HIGH);
  PotExpander.writeGPIOAB(0xFF);
  ConfigExpander.writeGPIOAB(0xFF);
  while(!Serial);
  delay(10);
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK){
    Serial.println("MCP2515 Initialized Successfully!");
     configed=true;}
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 Library Receive Example...");
  configed=true;
  SPI.endTransaction();
  
}

void loop()
{
  if(configed)                         // If CAN0_INT pin is low, read receive buffer
  {
     SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
     CAN0.readMsgBuf(&rxId, &len, rxBuf);   // Read data: len = data length, buf = data byte(s)
     SPI.endTransaction();
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }
  delay(100);
}

//
//mcp2515_write_register(0x2A, 0x00);      // Write to CNF1
//  mcp2515_write_register(0x29, 0x99);     // Write to CNF2
//  mcp2515_write_register(0x28, 0x02);     // Write to CNF3
//  
//  mcp2515_write_register(0x2B, 0x00);     // Write to CANINTE and clear it
//    
//  mcp2515_write_register(0x60, 0x60);     // Write to RXB0CTRL and set to turn masks off
//  mcp2515_write_register(0x70, 0x60);     // Write to RXB1CTRL and set to turn masks off  
//  
//  mcp2515_write_register(0x20, 0x00);     // Write to RXM0SIDH and clear it 
//  mcp2515_write_register(0x21, 0x00);     // Write to RXM0SIDL and clear it 
//  mcp2515_write_register(0x22, 0x00);     // Write to RXM0EID8 and clear it 
//  mcp2515_write_register(0x23, 0x00);     // Write to RXM0EID0 and clear it 
//  
//  mcp2515_write_register(0x24, 0x00);     // Write to RXM1SIDH and clear it
//  mcp2515_write_register(0x25, 0x00);     // Write to RXM1SIDL and clear it 
//  mcp2515_write_register(0x26, 0x00);     // Write to RXM1EID8 and clear it 
//  mcp2515_write_register(0x27, 0x00);     // Write to RXM1EID0 and clear it
//  
//  mcp2515_bit_modify(0x0C, 0x05, (1<<2));   // Set RXM0BF as a digital output
//  
//  mcp2515_write_register(0x0D, 0x00);     // Set TXnRTS pins as inputs
//  
//  mcp2515_bit_modify(MCP_CANCTRL, 0xE0, 0x00);
//  
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
