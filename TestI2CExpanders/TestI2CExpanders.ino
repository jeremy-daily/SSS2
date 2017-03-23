/*
   Smart Sensor Simulator 2
   Controlling the  Quadtrature Knob, Ignition Relay, and Voltage Regulator
   Hardware Revision 3

   Written by Dr. Jeremy S. Daily
   Synercon Technologies, LLC
   
   Proprietary Software - Not for Public Release

   Copyright (c) 2017        Jeremy S. Daily

   C:\Users\jeremy-daily\Documents\Arduino\libraries\MCP23017 
*/
#include <SPI.h>
#include <i2c_t3.h>
#include <EEPROM.h>
#include <FlexCAN.h>
#include "OneButton.h"
#include "Adafruit_MCP23017.h"

Adafruit_MCP23017 ConfigExpander; //U21
Adafruit_MCP23017 PotExpander; //U33

uint16_t ConfigExpanderValue = 0;
uint16_t PotExpanderValue = 0;

void setup() {
  // put your setup code here, to run once:
  PotExpander.begin(7);  //U33
  ConfigExpander.begin(3); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
    PotExpander.digitalWrite(i,LOW);
    ConfigExpander.digitalWrite(i,LOW);
  }
  pinMode(LED_BUILTIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  digitalWrite(LED_BUILTIN,HIGH);
  PotExpander.writeGPIOAB(0xFFFF);
  for (uint8_t i = 0; i<16; i++){
    ConfigExpander.digitalWrite(i,HIGH);
  }
  delay(2000);
  digitalWrite(LED_BUILTIN,LOW);
  PotExpander.writeGPIOAB(0);
  for (uint8_t i = 0; i<16; i++){
    ConfigExpander.digitalWrite(i,LOW);
  }
}
