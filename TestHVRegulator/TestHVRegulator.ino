/*
   Smart Sensor Simulator 2
   Controlling the i2C Potentiometers
   Hardware Revision 3

   Written by Dr. Jeremy S. Daily
   Synercon Technologies, LLC
   
   Proprietary Software - Not for Public Release

   Copyright (c) 2017        Jeremy S. Daily

   C:\Users\jeremy-daily\Documents\Arduino\libraries\MCP23017 
*/
#include <i2c_t3.h>

/**********************************************************************/
/*   Definitions for i2C addresses for the SSS2 Chips                 */
const uint8_t U34addr = 0x3C;
const uint8_t U36addr = 0x3F;
const uint8_t U37addr = 0x3D;
const uint8_t U24addr = 0x3E;

/**********************************************************************/
/*   Definitions for Digital Potentiometer Terminal Connections       */
const uint8_t TCON_B_ONLY      = 1;
const uint8_t TCON_WIPER_ONLY  = 2;
const uint8_t TCON_WIPER_AND_B = 3;
const uint8_t TCON_A_ONLY      = 4;
const uint8_t TCON_A_AND_B     = 5;
const uint8_t TCON_WIPER_AND_A = 6;
const uint8_t TCON_CONNECT_ALL = 7;

uint8_t nRetVal;
uint8_t result;

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
  result = Wire.read(); //Read Wiper Register, first byt is all 0
  result = Wire.read(); //Read Wiper Register
  return result;
} 

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while(!Serial); //wait for the serial port to open
  Serial.println("Setting Terminals.");
  nRetVal = MCP41HVI2C_SetTerminals(U24addr, 7);
  Serial.println(nRetVal);
  nRetVal = MCP41HVI2C_SetTerminals(U34addr, 7);
  Serial.println(nRetVal);
  nRetVal = MCP41HVI2C_SetTerminals(U36addr, 7);
  Serial.println(nRetVal);
  nRetVal = MCP41HVI2C_SetTerminals(U37addr, 7);
  Serial.println(nRetVal);
  
  nRetVal = MCP41HVI2C_SetWiper(U24addr,0);
  Serial.println(nRetVal);
  
  
}


void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  nRetVal = MCP41HVI2C_SetWiper(U24addr,0);
  Serial.println(nRetVal);
  delay(1000);
  nRetVal = MCP41HVI2C_SetWiper(U24addr,127);
  Serial.println(nRetVal); 
  delay(1000);
  nRetVal = MCP41HVI2C_SetWiper(U24addr,255);
  Serial.println(nRetVal); 
  
}
