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

const uint8_t PWM1Pin = 16;
const uint8_t PWM2Pin = 17;
const uint8_t PWM3Pin = 22;
const uint8_t PWM4Pin = 23;
const uint8_t CStermPin = 21;
const uint8_t IL1Pin = 37;

Adafruit_MCP23017 ConfigExpander; //U21
Adafruit_MCP23017 PotExpander; //U33


boolean IL1State = false;

uint16_t configSwitchSettings;
boolean LIN1Switch = false;
boolean LIN2Switch = true;
boolean P10or19Switch = false;
boolean P15or18Switch = false;
boolean U1though8Enable = true;
boolean U9though16Enable = true;
boolean CAN1Switch = true;
boolean CAN2Switch = false;
boolean U1U2P0ASwitch = true;
boolean U3U4P0ASwitch = true;
boolean U5U6P0ASwitch = true;
boolean U7U8P0ASwitch = true;
boolean U9U10P0ASwitch = true;
boolean U11U12P0ASwitch = true;
boolean U13U14P0ASwitch = true;
boolean U15U16P0ASwitch = true;



uint8_t terminationSettings;
boolean CAN0term = true;
boolean CAN1term = true;
boolean CAN2term = true;
boolean LINmaster = false;
boolean PWM1Out = true;
boolean PWM2Out = true;
boolean PWM3Out = true;
boolean PWM4Out = true;

uint8_t setTerminationSwitches() {
  //Set the termination Switches of U29 on Rev 3

  uint8_t terminationSettings =  CAN0term | CAN1term << 1 | CAN2term << 2 |  LINmaster << 3 | PWM1Out << 4 | PWM2Out << 5 | PWM3Out << 6 | PWM4Out << 7;
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

void setup() {
  SPI.begin();
  Serial.begin(9600);
  while(!Serial);
  // put your setup code here, to run once:
  PotExpander.begin(7);  //U33
  ConfigExpander.begin(3); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
  }
  Serial.println("Finished Setting Up MCP23017 extender chips.");
  PotExpander.writeGPIOAB(0xFFFF);
  ConfigExpander.writeGPIOAB(0xFFFF);
  
  pinMode(PWM1Pin,OUTPUT);
  pinMode(PWM2Pin,OUTPUT);
  pinMode(PWM3Pin,OUTPUT);
  pinMode(PWM4Pin,OUTPUT);
  pinMode(CStermPin,OUTPUT);
  pinMode(IL1Pin,OUTPUT);

  digitalWrite(IL1Pin,IL1State);
  
  analogWrite(PWM1Pin,220);
  analogWrite(PWM2Pin,168);
  analogWrite(PWM3Pin,200);
  analogWrite(PWM4Pin,127);

  
  pinMode(CStermPin,OUTPUT);

  Serial.print("Termination Switches (U29): ");
  terminationSettings = setTerminationSwitches();
  Serial.println(terminationSettings,BIN);

  Serial.print("Configration Switches (U21): ");
  configSwitchSettings = setConfigSwitches();
  Serial.println(configSwitchSettings,BIN);
}


void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  PWM1Out = true;
  PWM2Out = true;
  PWM3Out = true;
  PWM4Out = true;
  Serial.println(setTerminationSwitches(),BIN);
  
  delay(2000);
  PWM1Out = false;
  PWM2Out = false;
  PWM3Out = false;
  PWM4Out = false;
  Serial.println(setTerminationSwitches(),BIN);
}
