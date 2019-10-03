/*
 * Smart Sensor Simulator 2
 * 
 * Arduino Sketch to enable the functions for the SSS2
 * 
 * Written By Dr. Jeremy S. Daily
 * The University of Tulsa
 * Department of Mechanical Engineering
 * 
 * v 1.0: 22 May 2017
 * v 1.1: 19 May 2018
 * v 1.2: 17 July 2019
 * Released under the MIT License
 *
 * Copyright (c) 2017,2019        Jeremy S. Daily
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
 * 
 * 
 * Uses Arduino 1.8.5 and Teensyduino 1.41
*/
#include "SSS2_defines.h"
#include "SSS2_board_defs_rev_5.h"
#include "SSS2_functions.h"
#include "SSS2_LIN_functions.h"
#include "version.h"

char serial_buffer[64];

//softwareVersion
String softwareVersion = "SSS2*REV" + revision + "*" + VERSION;

void listSoftware(){
  Serial.print("FIRMWARE ");
  Serial.println(softwareVersion);
}

elapsedMillis usb_tx_timer;
int8_t ret_val;
uint8_t usb_buffer[65];
uint8_t usb_hid_rx_buffer[65];
uint8_t timeout = 0;

void setup() {
  SPI.begin();
  SPI1.begin();
  //while(!Serial); //Uncomment for testing
  status_buffer_1[0] = 1;
  status_buffer_2[0] = 2;
  status_buffer_3[0] = 3;
    
  LIN.begin(19200);
   
  commandString.reserve(256);
  commandPrefix.reserve(20);
  
  kinetisUID(uid);
  print_uid();
  
  analogWriteResolution(12);
  
  setSyncProvider(getTeensy3Time);
  setSyncInterval(1);
  
  setPinModes();
  
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(2000); // 2ms

  //Configure the MCP23017
  const uint8_t BANK   = 0x80;
  const uint8_t MIRROR = 0x00;
  const uint8_t SEQOP  = 0x20;
  const uint8_t DISSLW = 0x10; 
  const uint8_t HAEN   = 0x08; 
  const uint8_t ODR    = 0x00; 
  const uint8_t INTPOL = 0x02;
  
//  Wire.beginTransmission(configExpanderAddr); 
//  Wire.write(uint8_t(MCP23017_IOCON)); // sends instruction byte  
//  Wire.write(uint8_t(BANK | MIRROR | SEQOP | DISSLW | HAEN | ODR | INTPOL));     
//  Wire.endTransmission();
  
  Wire.beginTransmission(potExpanderAddr); 
  Wire.write(uint8_t(MCP23017_IODIRA)); // sends instruction byte  
  Wire.write(uint8_t(0));     
  Wire.endTransmission();

  Wire.beginTransmission(potExpanderAddr); 
  Wire.write(uint8_t(MCP23017_IODIRB)); // sends instruction byte  
  Wire.write(uint8_t(0));     
  Wire.endTransmission();

  Wire.beginTransmission(configExpanderAddr); 
  Wire.write(uint8_t(MCP23017_IODIRA)); // sends instruction byte  
  Wire.write(uint8_t(0));     
  Wire.endTransmission();

  Wire.beginTransmission(configExpanderAddr); 
  Wire.write(uint8_t(MCP23017_IODIRB)); // sends instruction byte  
  Wire.write(uint8_t(0));     
  Wire.endTransmission();

 
  setTerminationSwitches();
  setPWMSwitches();
  setConfigSwitches();
  
  button.attachClick(myClickFunction);
  button.attachDoubleClick(myDoubleClickFunction);
  button.attachLongPressStart(longPressStart);
  button.attachLongPressStop(longPressStop);
  button.attachDuringLongPress(longPress);
  button.setPressTicks(2000);
  button.setClickTicks(250);
 
  initializeDACs(Vout2address);

  load_settings();
  for (int i = 1; i < numSettings; i++) {
    currentSetting = setSetting(i, -1,DEBUG_OFF);
    setSetting(i, currentSetting, DEBUG_OFF);
  }
  delayMicroseconds(100000);
  listInfo();

 
  if(MCPCAN.begin(MCP_ANY, getBAUD(BAUDRATE_MCP), MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  MCPCAN.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  status_buffer_2[CAN2_BAUD_LOC] = getBAUD(BAUDRATE_MCP);
  
  
  //Start FlexCAN with Auto Baud
  Can0.begin(0);
  Can1.begin(0);
  
  status_buffer_2[CAN0_BAUD_LOC] = getBAUD(Can0.baud_rate);
  status_buffer_2[CAN1_BAUD_LOC] = getBAUD(Can1.baud_rate);
  
  Can0.startStats();
  Can1.startStats();

  txmsg.ext = 1;
  txmsg.len = 8;
 

    
  LIN.begin(19200,SERIAL_8N2);

  J1708.begin(9600);
  J1708.clear();
  
  CANTimer.begin(runCANthreads, 500); // Run can threads on an interrupt. Produces very little jitter.

  currentSetting = 1;
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;

  LINfinished = true;
  getCompIdEEPROMdata();
 
  commandString="1";
  setEnableComponentInfo();
  reloadCAN();
  listSettings();
}

void loop() {
  //Check CAN messages
  if (Can0.available()) {
    Can0.read(rxmsg);
    //parseJ1939(rxmsg);
    RXCount0++;
    memcpy(&status_buffer_2[CAN0_RX_COUNT_LOC],&RXCount0,4);
    RXCAN0timer = 0;
    if (displayCAN0) printFrame(rxmsg, 0, RXCount0);
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);
  }
  if (Can1.available()) {
    Can1.read(rxmsg);
    RXCount1++;
    memcpy(&status_buffer_2[CAN1_RX_COUNT_LOC],&RXCount1,4);
    RXCAN1orJ1708timer = 0;
    if (displayCAN1) printFrame(rxmsg, 1, RXCount1);
    if (ignitionCtlState){
      greenLEDstate = !greenLEDstate;
      digitalWrite(greenLEDpin, greenLEDstate);
    }
  }
  //!digitalRead(INTCANPin) &&
  if(MCPCAN.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){      // Read data: len = data length, buf = data byte(s)
      RXCount2++;
      memcpy(&status_buffer_2[CAN2_RX_COUNT_LOC],&RXCount2,4);  
      if(displayCAN2)  // If low, read receive buffer
      {
        rxmsg.id = (rxId & 0x1FFFFFFF);
        rxmsg.len = len;
        for(byte i = 0; i<len; i++) rxmsg.buf[i] = rxBuf[i];
        printFrame(rxmsg, 2, RXCount2);
      }
  }
  
  //Check J1708
  if (J1708.available()){
    J1708RXbuffer[J1708_index] = J1708.read();
    J1708RXtimer = 0;
    newJ1708Char = true;
    J1708_index++;
    if (J1708_index > sizeof(J1708RXbuffer)) J1708_index = 0;
  }
  if (newJ1708Char && J1708RXtimer > 1150) { //At least 11 bit times must pass
    //Check to see if this is the first displayed message. If so, discard and start showing subsequent messages.
    if (firstJ1708) firstJ1708 = false; 
    else{
      uint8_t j1708_checksum = 0;
       if (showJ1708) Serial.printf("J1708 %10lu.%06lu ",now(),uint32_t(microsecondsPerSecond));
      for (int i = 0; i<J1708_index;i++){
        j1708_checksum += J1708RXbuffer[i];
         if (showJ1708) Serial.printf("%02X ", J1708RXbuffer[i]);
      }
      if (j1708_checksum == 0){
        J1708RXCount++;
        memcpy(&status_buffer_2[J1708_RX_COUNT_LOC],&J1708RXCount,2);
        if (showJ1708) Serial.println("OK");
      }
      else {
         if (showJ1708) Serial.println("Checksum Failed.");
      }
    }
    J1708_index = 0;
    newJ1708Char = false;
  }
    
  if (analog_tx_timer >= analog_display_period ){
    analog_tx_timer=0;
    for (uint8_t j = 0; j < numADCs; j++){
      uint16_t temp_reading = analogRead(analogInPins[j]);
      memcpy(&status_buffer_3[ANALOG_OUT_START_LOC + 2*j],&temp_reading,2);
    }
  }


//  if (enableSendComponentInfo){
//    if (canComponentIDtimer > 5000){
//      canComponentIDtimer = 0;
//      can_messages[comp_id_index]->enabled = true;
//      can_messages[comp_id_index]->transmit_number = 0;
//      can_messages[comp_id_index]->ok_to_send = true;
//      can_messages[comp_id_index]->loop_cycles = 0; 
//      can_messages[comp_id_index]->cycle_count = 0;
//      can_messages[comp_id_index]->message_index = 0;
//    }
//  }

  sendLINResponse();

  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  uint8_t n;
  if (Serial.available() >= 2) {
    // This section makes the SSS2 backwards compatible. 
    memset(serial_buffer,0,64);
    serial_buffer[0]=COMMAND_TYPE;
    n=1;
    while (n < 64){
      char c = Serial.read();
      if (c == '\n'){
        n=64;
      }
      else{
        serial_buffer[n] = c;
        n++;
      }
      Serial.write(c);
    }
    memset(usb_hid_rx_buffer,0,64);
    memcpy(&usb_hid_rx_buffer,&serial_buffer,64);
    uint16_t checksum = CRC16.ccitt(usb_hid_rx_buffer, 62);
    memcpy(&usb_hid_rx_buffer[62], &checksum, 2);
  }
  else{
    n = RawHID.recv(usb_hid_rx_buffer,0);
  }
  if (n > 0 ){
    // Extract the CRC from the message
    uint16_t crc_message = (usb_hid_rx_buffer[63] << 8) + usb_hid_rx_buffer[62];
    uint16_t crc = CRC16.ccitt(usb_hid_rx_buffer, 62);
//    Serial.print("CRC: ");
//    Serial.print(crc_message);
//    Serial.print(" ?=? ");
//    Serial.println(crc);
//    for (uint8_t i = 0; i < n; i++) Serial.write(usb_hid_rx_buffer[i]);
//    Serial.println();
    if ((usb_hid_rx_buffer[USB_FRAME_TYPE_LOC] & USB_FRAME_TYPE_MASK) == COMMAND_TYPE
         && crc_message == crc){
      uint8_t myByteArray[61];
      memcpy(&myByteArray,&usb_hid_rx_buffer[1],61);
      char * pch;
      pch = strtok((char *)myByteArray,", .");
      while (pch != NULL)
      {
        commandPrefix = String(pch);
        //Serial.print("commandPrefix: ");
        //Serial.println(commandPrefix);
        pch = strtok(NULL,", .");
        commandString =  String(pch);   
        //Serial.print("commandString: ");
        //Serial.println(commandString);
        pch = strtok(NULL,", .");
        if      (commandPrefix.toInt() > 0)                   fastSetSetting();  
        else if (commandPrefix.equalsIgnoreCase("AI"))        displayVoltage();
        else if (commandPrefix.equalsIgnoreCase("B0"))        autoBaud0();
        else if (commandPrefix.equalsIgnoreCase("B1"))        autoBaud1();
        else if (commandPrefix.equalsIgnoreCase("BMCP"))      autoBaudMCP();
        else if (commandPrefix.equalsIgnoreCase("DB"))        displayBaud();
        else if (commandPrefix.equalsIgnoreCase("CANCOMP"))   setEnableComponentInfo();
        else if (commandPrefix.equalsIgnoreCase("ID"))        print_uid();
        else if (commandPrefix.equalsIgnoreCase("C0"))        startStopCAN0Streaming();
        else if (commandPrefix.equalsIgnoreCase("C1"))        startStopCAN1Streaming();
        else if (commandPrefix.equalsIgnoreCase("C2"))        startStopCAN2Streaming();
        else if (commandPrefix.equalsIgnoreCase("GO"))        startCAN();
        else if (commandPrefix.equalsIgnoreCase("SP"))        set_shortest_period();
        else if (commandPrefix.equalsIgnoreCase("STOPCAN"))   stopCAN();
        else if (commandPrefix.equalsIgnoreCase("STARTCAN"))  goCAN();
        else if (commandPrefix.equalsIgnoreCase("CLEARCAN"))  clearCAN();
        else if (commandPrefix.equalsIgnoreCase("STATS"))     displayStats();
        else if (commandPrefix.equalsIgnoreCase("CLEARSTATS"))clearStats();
        else if (commandPrefix.equalsIgnoreCase("CI"))        changeComponentID();
        else if (commandPrefix.equalsIgnoreCase("LS"))        listSettings();
        else if (commandPrefix.equalsIgnoreCase("OK"))        checkAgainstUID();
        else if (commandPrefix.equalsIgnoreCase("CANNAME"))   getThreadName();
        else if (commandPrefix.equalsIgnoreCase("CANSIZE"))   getThreadSize();
        else if (commandPrefix.equalsIgnoreCase("THREADS"))   getAllThreadNames();
        else if (commandPrefix.equalsIgnoreCase("SOFT"))      listSoftware();
        else if (commandPrefix.equalsIgnoreCase("J1708"))     displayJ1708();
        else if (commandPrefix.equalsIgnoreCase("SM"))        setupPeriodicCANMessage();
        else if (commandPrefix.equalsIgnoreCase("CANSEND"))   sendMessage();
        else if (commandPrefix.equalsIgnoreCase("RELOAD"))    reloadCAN();
        else if (commandPrefix.equalsIgnoreCase("TIME"))      displayTime();
        else if (commandPrefix.equalsIgnoreCase("GETTIME"))   getTeensyTime();
        else if (commandPrefix.equalsIgnoreCase("LIN"))       displayLIN();
        else if (commandPrefix.equalsIgnoreCase("SENDLIN"))   sendLINselect(); 
        else if (commandPrefix.equalsIgnoreCase("LOAD"))      load_settings();
        else if (commandPrefix.equalsIgnoreCase("SAVE"))      save_settings();
        else {
          Serial.println(("ERROR Unrecognized Command Characters."));
        }
      }
    }
  }
  /*              End Serial Command Processing                   */
  /****************************************************************/

  /****************************************************************/
  /*            Begin Quadrature Knob Processing                  */
  button.tick() ; //check for presses
  int32_t newKnob = knob.read(); //check for turns
  if (newKnob != currentKnob) {
    if (newKnob >= knobHighLimit) {  //note: knob limits are for each input parameter
      knob.write(knobHighLimit);
      currentKnob = knobHighLimit;
    }
    else if (newKnob <= knobLowLimit) {
      knob.write(knobLowLimit);
      currentKnob = knobLowLimit;
    }
    else
    {
      currentKnob = newKnob;
    }
    //Place function calls to execute when the knob turns.
    if (ADJUST_MODE_ON) {
      setSetting(currentSetting, currentKnob,DEBUG_ON);
    }
    else {
      currentSetting = currentKnob;
      setSetting(currentSetting,-1,DEBUG_ON);
    }
  }
  /*            End Quadrature Knob Processing                    */
  /****************************************************************/


  /****************************************************************/
  /*           Begin LED Indicators for messages                  */
  /*
  /*Reset the redLED after a timeout in case the last CAN message was on the wrong state*/
  if (RXCAN0timer >= 200) { 
    RXCAN0timer = 0;
    redLEDstate = true;
    digitalWrite(redLEDpin, redLEDstate); //Use red because it is the power button.
  }
  
  /*Reset the greenLED after a timeout in case the last CAN message was on the wrong state*/
  if (RXCAN1orJ1708timer >= 200) { 
    RXCAN1orJ1708timer = 0;
    if (ignitionCtlState) greenLEDstate = true;
    else greenLEDstate = false;
    digitalWrite(greenLEDpin, greenLEDstate); 
  }
  /*             End LED Indicators for messages                        */
  /**********************************************************************/

  /*
   * Send USB RawHID Status
   */

  if (usb_tx_timer >= 200){
    usb_tx_timer = 0;
    status_buffer_1[61]++;
    uint16_t checksum1 = CRC16.ccitt(status_buffer_1, 62);
    memcpy(&status_buffer_1[62], &checksum1, 2);
    ret_val = RawHID.send(status_buffer_1, timeout);
    
    status_buffer_2[61]++;
    uint16_t checksum2 = CRC16.ccitt(status_buffer_2, 62);
    memcpy(&status_buffer_2[62], &checksum2, 2);
    ret_val = RawHID.send(status_buffer_2, timeout);

    status_buffer_3[61]++;
    uint16_t checksum3 = CRC16.ccitt(status_buffer_3, 62);
    memcpy(&status_buffer_3[62], &checksum3, 2);
    ret_val = RawHID.send(status_buffer_3, timeout);
  }
}
