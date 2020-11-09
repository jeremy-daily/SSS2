/*
 * Example file to test the autobaud function of the Teensy 3.x 
 * uing FlexCAN. The list of baud rates can be modified in the defines of the FlexCAN.h
 * 
 * The last known good value of the CAN bitrate is stored in EEPROM. The address is also 
 * in the defines section of the FlexCAN.h. The following defines are available to addjust 
 * autobaud settings.
 *   AUTOBAUD_TIMEOUT 
 *   EEPROM_BIT_RATE_INDEX_ADDR
 *   NUM_BAUD_RATES 
 *   BAUD_RATE_LIST 
 *   
 * Passing a value of 0 into the CanX.begin() function invokes the autobaud routine.
 * Autobaud is done in listen only mode, so it should not transmit error on the bus when
 * looking for a proper baud rate.
 * 
 * Example written by Jeremy Daily
 * November 2018
 * 
 */
#include <FlexCAN.h>

//Create a counter to keep track of message traffic
uint32_t RXCount0 = 0;
uint32_t RXCount1 = 0;

//Define message structure from FlexCAN library
static CAN_message_t rxmsg;

boolean LED_state;

//A generic CAN Frame print function for the Serial terminal
void printFrame(CAN_message_t rxmsg, uint8_t channel, uint32_t RXCount)
{
  char CANdataDisplay[50];
  sprintf(CANdataDisplay, "%d %12lu %12lu %08X %d %d", channel, RXCount, micros(), rxmsg.id, rxmsg.ext, rxmsg.len);
  Serial.print(CANdataDisplay);
  for (uint8_t i = 0; i < rxmsg.len; i++) {
    char CANBytes[4];
    sprintf(CANBytes, " %02X", rxmsg.buf[i]);
    Serial.print(CANBytes);
  }
  Serial.println();
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  LED_state = true;
  digitalWrite(LED_BUILTIN, LED_state);
  
  while(!Serial);
  Serial.println("Starting CAN Autobaud Test.");
  
  //Initialize the CAN channels with autobaud setting
  Can0.begin(0);
  #if defined(__MK66FX1M0__)
  Can1.begin(0);
  #endif
}

void loop()
{
  while (Can0.read(rxmsg)) {
    printFrame(rxmsg,0,RXCount0++);
    LED_state = !LED_state;
    digitalWrite(LED_BUILTIN, LED_state);
  }
  #if defined(__MK66FX1M0__)
  while (Can1.read(rxmsg)) {
    printFrame(rxmsg,1,RXCount1++);
    LED_state = !LED_state;
    digitalWrite(LED_BUILTIN, LED_state);
   }
  #endif
}
