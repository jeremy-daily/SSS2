
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

const uint8_t linRXpin = 0;

elapsedMicros LINtimer;
elapsedMicros LINsyncTimer;
bool LINbreak;
int LINtransitionCount;
uint32_t LINbitTime=52;
uint16_t LINsyncPause = 13*LINbitTime; //microseconds
bool readyForLINsync;
bool readyForLINid;
bool readyForLINdata;
bool LINfinished;
bool readyForLINchecksum;
bool readyToTransmitShifter;
uint8_t LINbuffer[8];
uint8_t LINchecksum;
uint8_t LINaddress;
uint8_t LINlength;
uint8_t LINindex;
uint8_t LIN_ID;
uint8_t outByte[256]; 
uint8_t k;

void resetLINtimer(){
  LINtimer = 0;
  attachInterrupt(linRXpin,determineSync,RISING);
}

void determineSync(){
  if(LINtimer > LINsyncPause) {
    Serial.println("Break");
    detachInterrupt(linRXpin);
    Serial1.begin(19200,SERIAL_8N1);
    readyForLINsync=true;
  }
}
 
void setup()
{
 
   SPI.begin();
   setPinModes();

   pinMode(linRXpin,INPUT);
  
 
  PotExpander.begin(7);  //U33
  ConfigExpander.begin(3); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
  }
  pinMode(ignitionCtlPin,OUTPUT);
  PotExpander.writeGPIOAB(0xFF);
  ConfigExpander.writeGPIOAB(0xBF);
  
  
  Serial1.begin(19200,SERIAL_8N1);
  
  while(!Serial);
  attachInterrupt(linRXpin,determineSync,FALLING);

  Serial.println("Welcome to the Testbench!"); 
  delay(10);
  digitalWrite(ignitionCtlPin,HIGH);
  LINfinished = true;
   
}

void loop()
{

  if( Serial1.available() ){
    if (readyForLINsync){
      readyForLINsync=false;
      uint8_t LIN_Sync = Serial1.read();
      if (LIN_Sync == 0x55){
        readyForLINid = true;
        Serial.printf("LIN Sync: %02X\n",LIN_Sync);
      }
      else{
        Serial.printf("ERROR: LIN Sync: %02X\n",LIN_Sync);
        LINfinished = true;
      }
    }
    else if (readyForLINid ){
      LIN_ID = Serial1.read();
      readyForLINid = false;
      LINindex = 0;
      LINtimer = 0;
      
      bool ID0 = (LIN_ID & 0b00000001) >> 0;
      bool ID1 = (LIN_ID & 0b00000010) >> 1; 
      bool ID2 = (LIN_ID & 0b00000100) >> 2;
      bool ID3 = (LIN_ID & 0b00001000) >> 3; 
      bool ID4 = (LIN_ID & 0b00010000) >> 4; 
      bool ID5 = (LIN_ID & 0b00100000) >> 5; 
      bool LINparity0 = (LIN_ID & 0b01000000) >> 6;
      bool LINparity1 = (LIN_ID & 0b10000000) >> 7;
      LINaddress = (LIN_ID & 0x0F) >> 0;

     
//      bool ID0   = bool((LIN_ID & 0b10000000) >> 7);
//      bool ID1   = bool((LIN_ID & 0b01000000) >> 6); 
//      bool ID2   = bool((LIN_ID & 0b00100000) >> 5);
//      bool ID3   = bool((LIN_ID & 0b00010000) >> 4); 
//      bool ID4   = bool((LIN_ID & 0b00001000) >> 3); 
//      bool ID5   = bool((LIN_ID & 0b00000100) >> 2); 
//      bool LINparity0 = (LIN_ID & 0b00000010) >> 1;
//      bool LINparity1 = (LIN_ID & 0b00000001) >> 0;
//      LINaddress = (LIN_ID & 0xF0) >> 4;

      if (ID4 && ID5) LINlength = 8;
      else if (!ID4 && ID5) LINlength = 4;
      else if (ID4 && !ID5) LINlength = 2;
      else if (!ID4 && !ID5) LINlength = 2;
      Serial.printf("%02X, Address: %d, Length: %d, P0: %d=%d, ~P1: %d=%d\n",LIN_ID,LINaddress,LINlength,LINparity0,(ID0 | ID1 | ID2 | ID4),LINparity1,!(ID1 | ID3 | ID4 | ID5) );
      
      if (LIN_ID == 0x20){
        readyToTransmitShifter = true; 
        outByte[0] = 0x14;
        outByte[1] = (k << 4) + 0x01;
        k+=1;
        outByte[2] = 0x02;
        outByte[3] = 0x0D;
        int calculatedChecksum = 0;
        calculatedChecksum += 0x20;
        for (int i = 0; i<4;i++){
          calculatedChecksum +=outByte[i];
        }
        outByte[3] = uint8_t(~(calculatedChecksum % 255) );
          
      }
      else if (LINlength > 0){
        readyForLINdata = true;
      }
      else {
        LINfinished = true;
      }
      
     
      
    }
    else if (readyForLINdata){
      LINbuffer[LINindex] = Serial1.read();
      Serial.printf("%02X ",LINbuffer[LINindex]);
      LINindex++;
      if (LINindex == LINlength){
        readyForLINdata = false;
        readyForLINchecksum = true;
      }
    }
    else if (readyForLINchecksum ){
      LINchecksum = Serial1.read();
      int calculatedChecksum = 0;
      calculatedChecksum += LIN_ID & 0x3F;
      //calculatedChecksum += LINchecksum;
      //the inverted module-256 checksum
      for (LINindex = 0; LINindex < LINlength; LINindex++){
        calculatedChecksum += LINbuffer[LINindex];
      }
      uint8_t checksumValue = uint8_t(~(calculatedChecksum % 255) );
      Serial.printf("Checksum: %02X = %02X\n",LINchecksum,checksumValue);

      readyForLINchecksum = false;
      LINfinished = true;
    }
    else //Serial.printf("%02X\n",Serial1.read());
    Serial1.read();
  }
   
  if (LINfinished){
      Serial1.clear();
      Serial1.flush();     
      pinMode(linRXpin,INPUT);
      attachInterrupt(linRXpin,resetLINtimer,FALLING);
      LINfinished = false;
      
    }
    
  if (readyToTransmitShifter){
      if (LINtimer >= LINbitTime*LINindex){
        Serial1.write(outByte[LINindex]);
        LINindex++;
        if (LINindex > 5) {
          readyToTransmitShifter = false;
          LINfinished = true;
          LINtimer = 0;
          LINindex = 0;
       }
     }
   }
      

}
