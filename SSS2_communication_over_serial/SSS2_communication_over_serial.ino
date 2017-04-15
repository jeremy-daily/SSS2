

/*
   Smart Sensor Simulator 2
   Controlling the  Quadtrature Knob, Ignition Relay, and Voltage Regulator
   Hardware Revision 3

   Written by Dr. Jeremy S. Daily
   Synercon Technologies, LLC
   
   Proprietary Software - Not for Public Release

   Copyright (c) 2017        Jeremy S. Daily
*/


#include "SSS2_board_defs_rev_3.h"
#include "SSS2_functions.h"


//softwareVersion
String softwareVersion = "SSS2*REV" + revision + "*0.5*CAN-Fixes*d05f5f9df9027442f473471a37743c64a8cdbd42"; //Hash of the previous git commit










/**************************************************************************************/
/*               Begin Function calls for User input data                             */


void listInfo() {
  Serial.print("INFO SSS2 Component ID (Make*Model*Serial*Unit): ");
  Serial.println(componentID);
}

void getSoftwareVersion() {
  Serial.print("INFO SSS2 Firmware version: ");
  Serial.println(softwareVersion);
}

void changeComponentID() {
  if (commandString.length() > 5) componentID = commandString;
  Serial.print(F("SET SSS2 Component ID: "));
  Serial.println(componentID);
  if (commandString.length() <= 5 && commandString.length() > 0) Serial.println(F("Please make the component ID longer than 5 characters to change it."));
}


void autoBaud0(){
  Serial.println(F("B0 - Set the baudrate for CAN 0 or select AutoBaud"));
  char baudstring[9];
  if (commandString.length() > 0){
    commandString.toCharArray(baudstring,9);
    BAUDRATE0 = strtoul(baudstring,0,10);
    Serial.print("SET CAN0 baudrate set to ");
    Serial.println(BAUDRATE0);
    for (uint8_t baudRateIndex = 0; baudRateIndex<sizeof(baudRateList); baudRateIndex++){
      if (BAUDRATE0 == baudRateList[baudRateIndex]){
        Can0.begin(BAUDRATE0);
        for (uint8_t filterNum = 4; filterNum < 16;filterNum++) Can0.setFilter(allPassFilter,filterNum);
        CAN0baudNotDetected = false;
        break; 
      }
      else{
        CAN0baudNotDetected = true;
      }
      
    }
  }
  else {
    BAUDRATE0 = 0;
  } 
  if (BAUDRATE0 == 0){
    CAN0baudNotDetected = true;
    Serial.println("INFO CAN0 set to automatically set baudrate.");
  }
}

void autoBaud1(){
  Serial.println(F("INFO B1 - Set the baudrate for CAN 1 or select AutoBaud"));
  char baudstring[9];
  if (commandString.length() > 0){
    commandString.toCharArray(baudstring,9);
    BAUDRATE1 = strtoul(baudstring,0,10);
    Serial.print("INFO CAN1 baudrate set to ");
    Serial.println(BAUDRATE1);
    for (uint8_t baudRateIndex = 0; baudRateIndex<sizeof(baudRateList); baudRateIndex++){
      if (BAUDRATE1 == baudRateList[baudRateIndex]){
          Can1.begin(BAUDRATE1);
          for (uint8_t filterNum = 4; filterNum < 16;filterNum++) Can1.setFilter(allPassFilter,filterNum);
          CAN1baudNotDetected = false;
          break;
        }
        else{
         CAN1baudNotDetected = true;
        }
      }
  }
  else {
    BAUDRATE1 = 0;
  } 
  if (BAUDRATE1 == 0){
    CAN1baudNotDetected = true;
    Serial.println(F("INFO CAN1 set to automatically set baudrate."));
  }
}

void displayBaud(){
  Serial.print("SET CAN0 Baudrate");
  Serial.println(BAUDRATE0);
  Serial.print("SET CAN1 Baudrate");
  Serial.println(BAUDRATE1);  
}

void startStopCAN0Streaming(){
  if (commandString.toInt() > 0) displayCAN0 = true;
  else  displayCAN0 = false;
}

void startStopCAN1Streaming(){
  if (commandString.toInt() > 0) displayCAN1 = true;
  else  displayCAN1 = false;
}

void startStopCAN2Streaming(){
  if (commandString.toInt() > 0) displayCAN2 = true;
  else  displayCAN2 = false;
}

void setEnableComponentInfo(){
  if (commandString.toInt() > 0){
    enableSendComponentInfo = true;
    Serial.print(F("SET Enable CAN transmission of Component ID"));
  }
  else{
    enableSendComponentInfo = false;
    Serial.print(F("SET Disable CAN transmission of Component ID"));  
  }
  
}

void checkAgainstUID(){
  String secret = kinetisUID();
  if(commandString==secret) Serial.println("OK:Authenticated");
  else Serial.println("OK:Denied");
}



void sendMessage(){
  /* Sends a CAN message from the following string fields:
   *  channel,id,data
   *  channel is 0 for CAN0 and 1 for CAN1
   *  id is the CAN ID. if the ID is less than 11 bits and the first bit is set, then it will be transmitted as an extended id
   *  data length code is determined by the length of the data
   *  data is the hex representation in ascii with no spaces. there will be 16 hex characters for 8 bytes. Data longer than 8 bytes will be ignored. 
   */
  boolean goodID = false;
  boolean goodData = false;
  
  //Serial.println(F("SM - Send Message."));
  //Serial.println(commandString);
  char commandCharBuffer[100];
  char IdCharBuffer[9];
  char dataCharBuffer[17];
  char *endptr;
  if (commandString.length() > 0) {
    commandString.toCharArray(commandCharBuffer,commandString.length());
    int channel;
    if (commandCharBuffer[0]=='1') channel = 1;
    else if (commandCharBuffer[0]=='0') channel = 0;
    else channel = -1;
    
    //Serial.print(channel);  
    
    int commaIndex0 = commandString.indexOf(',');
    int commaIndex1 = commandString.indexOf(',',commaIndex0+1);
    if (commaIndex0 > 0 && commaIndex1 > 1){
      String idString = commandString.substring(commaIndex0+1,commaIndex1);
      //Serial.print("idString = ");
      //Serial.println(idString);
      idString.toCharArray(IdCharBuffer,9);
      for (uint8_t i = 0; i < strlen(IdCharBuffer) ; i++){
        //Serial.print(IdCharBuffer[i]);
        if (isxdigit(IdCharBuffer[i])) goodID = true; 
        else { goodID = false; break; }
      }
      if (goodID){
        uint32_t tempID = strtoul(IdCharBuffer,&endptr,16);
        //Serial.print(" ");
        //Serial.print(tempID,HEX);
        if (  (tempID >> 11) > 0){
          txmsg.ext = 1;
          txmsg.id = (0x3FFFFFFF & tempID); //29 bit ID
        }
        else {
          txmsg.ext = 0;
          txmsg.id = (0x7FF & tempID); //11 bit ID
        }
        //Serial.print(" ");
        //Serial.print(txmsg.id,HEX);
      }
      else Serial.println("ERROR Invalid ID format");
      
      String dataString = commandString.substring(commaIndex1+1);
      
      dataString.toCharArray(dataCharBuffer,17);
      txmsg.len = strlen(dataCharBuffer)/2;
      //Serial.print(" ");
      //Serial.print(txmsg.len,HEX);
      // Serial.print(" ");
      for (uint8_t i = 0; i < txmsg.len*2 ; i++){
        if (isxdigit(dataCharBuffer[i])) goodData = true; 
        else { goodData = false; Serial.println("ERROR Non Hex Characters or Odd number of nibbles or ID is too long"); break; }
      } 
      if (goodData){
        for (int i = 0; i <  txmsg.len ; i++){
          char byteStringChars[3] = {dataCharBuffer[2*i],dataCharBuffer[2*i+1],0x00};
          txmsg.buf[i] = strtol(byteStringChars,&endptr,16);
          //Serial.print(txmsg.buf[i],HEX);
          //Serial.print(" ");
        }
      }
    }
    
    if (goodData && goodID ){
      if (channel == 0) {
        Can0.write(txmsg);
      }
      else if (channel == 1){
        Can1.write(txmsg);
      }
      else Serial.println("ERROR Invalid Channel for SM.");
    }
    
    else
      Serial.println(F("ERROR Invalid input data for SM. Input should be using hex characters with no spaces in the form SM,channel,ID,data/n"));
  }
  else
  {
    Serial.println(F("ERROR Missing or invalid data to send."));
  }
  txmsg.ext = 1; //set default
  txmsg.len = 8;
}



/*                 End Function calls for User input data                             */
/**************************************************************************************/

void sendComponentInfo()
{
  if (enableSendComponentInfo){
       char id[29];
       componentID.toCharArray(id,29);
       
       Serial.print(F("INFO Received Request for Component ID. Sending  "));
       for (int i = 0; i<28;i++) Serial.print(id[i]);
       Serial.println();
       
       byte transport0[8] = {32,28,0,4,0xFF,0xEB,0xFE,0};
       byte transport1[8] = {1,id[0],id[1],id[2],id[3],id[4],id[5],id[6]};
       byte transport2[8] = {2,id[7],id[8],id[9],id[10],id[11],id[12],id[13]};
       byte transport3[8] = {3,id[14],id[15],id[16],id[17],id[18],id[19],id[20]};
       byte transport4[8] = {4,id[21],id[22],id[23],id[24],id[25],id[26],id[27]};
       txmsg.id = 0x1CECFFFA;
       txmsg.len = 8;
       memcpy(txmsg.buf,transport0,8);
       Can0.write(txmsg);
       delay(3);
       txmsg.id = 0x1CEBFFFA;
       memcpy(txmsg.buf,transport1,8);
       Can0.write(txmsg);
       memcpy(txmsg.buf,transport2,8);
       Can0.write(txmsg);
       memcpy(txmsg.buf,transport3,8);
       Can0.write(txmsg);
       memcpy(txmsg.buf,transport4,8);
       Can0.write(txmsg);
  }     
}       
void parseJ1939(CAN_message_t &rxmsg ){
  uint32_t ID = rxmsg.id;
  uint8_t DLC = rxmsg.len;
  uint8_t SA = (ID & 0xFF);
  uint8_t PF = (ID & 0x03FF0000) >> 16;
  uint8_t PRIORITY = (ID & 0x3C000000) >> 26;
  uint8_t DA = 0xFF;
  uint32_t PGN;
  if (PF >= 240){
     PGN = (ID & 0x03FFFF00) >> 8;
  }
  else{
    //pdu1 format
    PGN = (ID & 0x03FF0000) >> 8;
    DA = (ID & 0x00000FF00);
  }
  
  if (PGN == 0xEA00){
    //request message
    if (rxmsg.buf[0] == 0xEB && rxmsg.buf[1] == 0xFE){
      //Component ID 
      sendComponentInfo();
      //sendJ1939(0,6,0xFEEB,0xFF,sourceAddress,strlen(componentID),componentID);
    }
  }
  else if (PGN == 0xEB00){
    //Transport Protocol - Data
    
  }
  else if (PGN == 0xEC00){
    //Transport Protocol - Connection Management
    uint8_t controlByte = rxmsg.buf[0];
    if (controlByte == 16){
      //Connection Mode - Request to Send
      uint16_t totalMessageSize = rxmsg.buf[1] + rxmsg.buf[2]*256;
      uint8_t totalPackets = rxmsg.buf[3];
      uint8_t numPacketsToBeSent = rxmsg.buf[4];
      uint32_t requestedPGN = rxmsg.buf[5] + (rxmsg.buf[6] << 8) + (rxmsg.buf[7] << 16);
      //TODO: Set up a response
    }
    else if (controlByte == 17){
      //Connection Mode - Clear to Send 
      uint8_t numPacketsToBeSent = rxmsg.buf[1];
    }
    else {
      uint16_t totalMessageSize = rxmsg.buf[1] + rxmsg.buf[2]*256;
      uint8_t totalPackets = rxmsg.buf[3];
      uint8_t numPacketsToBeSent = rxmsg.buf[4];
      uint8_t maxNumOfPackets =rxmsg.buf[5];
      uint8_t nextPacketToBeSent =rxmsg.buf[6];
      uint8_t sequenceNumber =rxmsg.buf[7];
    }


    
    
    
  }
}

void sendJ1939(uint8_t channel, uint8_t priority, uint32_t pgn, uint8_t DA, uint8_t SA, int numBytes, char J1939Buffer[]){
  if (numBytes <= 8){
    if((pgn & 0xFF00) >= 240) txmsg.id = (priority << 26) + (pgn << 8) + SA;
    else txmsg.id = (priority << 26) + (pgn << 8) + (DA << 8) + SA;
    txmsg.len = numBytes;
    for (uint8_t i = 0; i<numBytes;i++) txmsg.buf[i]=J1939Buffer[i];
    if (channel == 0) Can0.write(txmsg);
    else if(channel == 1) Can1.write(txmsg);
    else Serial.println("0: J1939 Message not sent.");
  }
  else {
    //transport
    uint8_t numFrames = numBytes;
    transportTimer = 0;
    
  }
}


//A generic CAN Frame print function for the Serial terminal
char outMessage[27] ={};
void printFrame(CAN_message_t rxmsg, int mailbox, uint8_t channel, uint32_t RXCount)
{ 
//  uint32_t currentMicros = micros();
//  uint8_t *idPointer = (uint8_t *)&rxmsg.id;
//  uint8_t *RXCountPointer = (uint8_t *)&RXCount;
//  uint8_t *microsPointer = (uint8_t *)&currentMicros;
//
//  outMessage[0]='C';
//  outMessage[1]='A';
//  outMessage[2]='N';
//  outMessage[3]=channel;
//  outMessage[4]=RXCountPointer[0];
//  outMessage[5]=RXCountPointer[1];
//  outMessage[6]=RXCountPointer[2];
//  outMessage[7]=RXCountPointer[3];
//  outMessage[8]=microsPointer[0];
//  outMessage[9]=microsPointer[1];
//  outMessage[10]=microsPointer[2];
//  outMessage[11]=microsPointer[3];
//  outMessage[12]=idPointer[0];
//  outMessage[13]=idPointer[1];
//  outMessage[14]=idPointer[2];
//  outMessage[15]=idPointer[3];
//  outMessage[16]=rxmsg.len;
//  outMessage[17]=rxmsg.buf[0];
//  outMessage[18]=rxmsg.buf[1];
//  outMessage[19]=rxmsg.buf[2];
//  outMessage[20]=rxmsg.buf[3];
//  outMessage[21]=rxmsg.buf[4];
//  outMessage[22]=rxmsg.buf[5];
//  outMessage[23]=rxmsg.buf[6];
//  outMessage[24]=rxmsg.buf[7];
//  outMessage[25]=0x0A;
//  Serial.write(outMessage,26);
  

  Serial.printf("CAN%d %10lu %10lu %08X %d %d %02X %02X %02X %02X %02X %02X %02X %02X\n",
          channel,RXCount,micros(),rxmsg.id,rxmsg.ext,rxmsg.len,
          rxmsg.buf[0],rxmsg.buf[1],rxmsg.buf[2],rxmsg.buf[3],
          rxmsg.buf[4],rxmsg.buf[5],rxmsg.buf[6],rxmsg.buf[7]);

}


time_t getTeensy3Time(){
  microsecondsPerSecond = 0;
  return Teensy3Clock.get();
}


void print_uid()  {  
  Serial.printf("ID: %s\n", kinetisUID());
}

void displayStats(){
  CAN_stats_t currentStats = Can0.getStats();
  printStats(currentStats,0);
  currentStats = Can1.getStats();
  printStats(currentStats,1);
}
 
void printStats(CAN_stats_t currentStats,int channel){
  Serial.printf("STATS for CAN%d: Enabled:%d, RingRXHighWater: %ul, ringRXFramesLost: %lu, ringTXHighWater: %lu, mailbox use count:[%lu",
                            channel,currentStats.enabled,currentStats.ringRxHighWater,currentStats.ringRxFramesLost,currentStats.ringTxHighWater,currentStats.mb[0].refCount);
  for (int i=1;i<16;i++){
    Serial.printf(", %lu",currentStats.mb[i].refCount); 
  }
  Serial.printf("], overrunCount: [%lu",currentStats.mb[0].overrunCount);
  for (int i=1; i<16; i++){
    Serial.printf(", %lu",currentStats.mb[i].overrunCount); 
  }
  Serial.println("]");
}

void clearStats(){
  Can0.clearStats();
  Can1.clearStats();
  
  Can0.startStats();
  Can1.startStats();
  
  Serial.println("INFO Cleared CAN Statisitics");
}
    
void setup() {
  Serial.begin(9600);
  Serial1.begin(19200);
  
  commandString.reserve(200);
  commandPrefix.reserve(20);
  
  kinetisUID(uid);
  
  analogWriteResolution(12);
  
  setSyncProvider(getTeensy3Time);
  setSyncInterval(1);
  
  setPinModes();

  Wire.begin();
  Wire.setDefaultTimeout(200000); // 200ms
  
  PotExpander.begin(potExpanderAddr);  //U33
  ConfigExpander.begin(configExpanderAddr); //U21
  for (uint8_t i = 0; i<16; i++){
    PotExpander.pinMode(i,OUTPUT);
    ConfigExpander.pinMode(i,OUTPUT);
  }
  PotExpander.writeGPIOAB(0xFFFF);
  ConfigExpander.writeGPIOAB(0xFFFF);
  
  SPI.begin();
  terminationSettings = setTerminationSwitches();
  
  Serial.print("Configration Switches (U21): ");
  uint16_t configSwitchSettings = setConfigSwitches();
  Serial.println(configSwitchSettings,BIN);
  
  button.attachClick(myClickFunction);
  button.attachDoubleClick(myDoubleClickFunction);
  button.attachLongPressStart(longPressStart);
  button.attachLongPressStop(longPressStop);
  button.attachDuringLongPress(longPress);
  button.setPressTicks(2000);
  button.setClickTicks(250);

  initializeDACs(Vout2address);

  listInfo();
  for (int i = 1; i < numSettings; i++) {
    currentSetting = setSetting(i, -1,DEBUG_OFF);
    setSetting(i, currentSetting ,DEBUG_ON);
  }
  
  currentSetting = 1;
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
  
  Can0.begin(BAUDRATE0);
  Can1.begin(BAUDRATE1);
  
  Can0.startStats();
  Can1.startStats();

  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    Can1.setFilter(allPassFilter,filterNum); 
  }

  txmsg.ext = 1;
  txmsg.len = 8;

  setConfigSwitches();
    
  LIN.begin(19200);
  LIN.flush();
  LIN.clear();


  J1708.begin(9600);
  J1708.flush();
  J1708.clear();
  
}

void loop() {
  //Always do this 
  can_thread_controller.run();
  
  //Check CAN messages
  while (Can0.available()) {
    Can0.read(rxmsg);
    RXCount0++;
    RXCAN0timer = 0;
    if (displayCAN0) printFrame(rxmsg, -1, 0, RXCount0);
    redLEDstate = !redLEDstate;
    digitalWrite(redLEDpin, redLEDstate);
  }
  while (Can1.available()) {
    Can1.read(rxmsg);
    RXCount1++;
    RXCAN1orJ1708timer = 0;
    if (displayCAN1) printFrame(rxmsg, -1, 1, RXCount1);
    if (ignitionCtlState){
      greenLEDstate = !greenLEDstate;
      digitalWrite(greenLEDpin, greenLEDstate);
    }
  }

  //Check J1708
  while (J1708.available()){
    J1708RXbuffer[J1708_index] = J1708.read();
    J1708RXtimer = 0;
    newJ1708Char = true;
    
    J1708_index++;
    if (J1708_index > sizeof(J1708RXbuffer)) J1708_index = 0;
  }
  if (showJ1708 && newJ1708Char && J1708RXtimer > 1150) { //At least 11 bit times must pass
    //Check to see if this is the first displayed message. If so, discard and start showing subsequent messages.
    if (firstJ1708) firstJ1708 = false; 
    else{
      uint8_t j1708_checksum = 0;
      Serial.print("J1708 ");
      for (int i = 0; i<J1708_index;i++){
        j1708_checksum += J1708RXbuffer[i];
        Serial.printf("%02X ", J1708RXbuffer[i]);
      }
      if (j1708_checksum == 0) Serial.println("OK");
      else Serial.println("Checksum Failed.");
    }
    J1708_index = 0;
    newJ1708Char = false;
  }

  
  /************************************************************************/
  /*            Begin PERIODIC CAN Message Transmission                            */
  if (A21TX_Timer >=100){
    A21TX_Timer=0;
    if (sendA21voltage) displayVoltage();
  }

   
  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 140) {
    commandPrefix = Serial.readStringUntil(',');
 
    if (Serial.available()) commandString = Serial.readStringUntil('\n');
    else commandString = "";
 
    if      (commandPrefix.toInt() > 0) fastSetSetting();  
    else if (commandPrefix.equalsIgnoreCase("SM")) {
      int i=0;
      while (i < 25){
          temp_txmsg.id = i;
          temp_txmsg.buf[0] = 2*i;
          setupPeriodicCANMessage(i);
          delay(1);
          can_thread_controller.run();
          i++;

        }
    }
    else if (commandPrefix.equalsIgnoreCase("J1708")) displayJ1708();
    else if (commandPrefix.startsWith("SS") || commandPrefix.startsWith("ss")) changeValue();
    else if (commandPrefix.startsWith("SC") || commandPrefix.startsWith("sc")) Serial.println(F("SC - Not implemented yet."));
    else if (commandPrefix.startsWith("SA") || commandPrefix.startsWith("sa")) saveEEPROM();
    else if (commandPrefix.startsWith("AO") || commandPrefix.startsWith("ao")) turnOnAdjustMode();
    else if (commandPrefix.startsWith("AF") || commandPrefix.startsWith("af")) turnOffAdjustMode();
    else if (commandPrefix.startsWith("CS") || commandPrefix.startsWith("cs")) changeSetting(); //Select setting to change
    else if (commandPrefix.startsWith("CI") || commandPrefix.startsWith("ci")) changeComponentID();
    else if (commandPrefix.startsWith("LS") || commandPrefix.startsWith("ls")) listSettings();
    else if (commandPrefix.startsWith("LI") || commandPrefix.startsWith("li")) listInfo();
    else if (commandPrefix.startsWith("B0") || commandPrefix.startsWith("b0")) autoBaud0();
    else if (commandPrefix.startsWith("B1") || commandPrefix.startsWith("b1")) autoBaud1();
    else if (commandPrefix.startsWith("DB") || commandPrefix.startsWith("db")) displayBaud();
    else if (commandPrefix.startsWith("C0") || commandPrefix.startsWith("c0")) startStopCAN0Streaming();
    else if (commandPrefix.startsWith("C1") || commandPrefix.startsWith("c1")) startStopCAN1Streaming();
    else if (commandPrefix.startsWith("C2") || commandPrefix.startsWith("c2")) startStopCAN2Streaming();
    else if (commandPrefix.startsWith("AI") || commandPrefix.startsWith("ai")) displayVoltage();
    else if (commandPrefix.startsWith("MK") || commandPrefix.startsWith("mk")) setEnableComponentInfo();
    else if (commandPrefix.startsWith("ID") || commandPrefix.startsWith("id")) print_uid();
    else if (commandPrefix.startsWith("SV") || commandPrefix.startsWith("sv")) streamVoltage();
    else if (commandPrefix.startsWith("OK") || commandPrefix.startsWith("ok")) checkAgainstUID();
    else if (commandPrefix.startsWith("ST") || commandPrefix.startsWith("st")) displayStats();
    else if (commandPrefix.startsWith("CL") || commandPrefix.startsWith("cl")) clearStats();

   
    
    else Serial.println(F("ERROR Unrecognized Command Characters. Use a comma after the command.\nERROR Known commands are CN, B0, B1, DS, VI, SW, PN, PD, PB, PF, LI, LS, CI, CS, AF, AO, SA, SC, SS, or SM."));
  
  }
  //Serial.clear();
  //Serial.flush();
  /*              End Serial Command Processing                   */
  /****************************************************************/

  /****************************************************************/
  /*            Begin Quadrature Knob Processing                  */
  button.tick(); //check for presses
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
      //setLimits(currentSetting);
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
  /*Reset the greenLED after a timeout in case the last CAN message was on the wrong state*/
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
}

/*

void loop(){
 can_thread_controller.run();

/****************************************************************/
  /*            Begin Serial Command Processing                  
  if (Serial.available() >= 2 && Serial.available() < 140) {
    commandPrefix = Serial.readStringUntil(',');
    if (Serial.available()) commandString = Serial.readStringUntil('\n');
    
    if (commandPrefix.startsWith("SM") || commandPrefix.startsWith("sm")) {
        temp_txmsg.id =  commandString.toInt();
       
        
        while (i < 25){
          temp_txmsg.id = i;
          temp_txmsg.buf[0] = 2*i;
          sendMessage();
          delay(1);
          can_thread_controller.run();
          i++;

        }
      }
    else if (commandPrefix.toLowerCase().startsWith("stop")){
      int index = commandString.toInt();
      Serial.printf("Stop,%d\n",index);
      can_messages[index]->enabled = false;
      can_messages[index]->txmsg.buf[1] = index;
      
      can_thread_controller.remove(index);
      //can_thread_controller.clear();
    }
    else if (commandPrefix.toLowerCase().startsWith("go")){
      int index = commandString.toInt();
      Serial.printf("Go,%d\n",index);
      can_messages[index]->enabled = true;
      //can_thread_controller.remove(index);
      //can_thread_controller.clear();
    }
    else if (commandPrefix.toLowerCase().startsWith("list")){
      for (int j = 0; j < can_thread_controller.size();j++){
         Serial.printf("ID: %X, enabled: %d \n", can_messages[j] -> txmsg.id,can_messages[j] -> enabled);
         can_thread_controller.run();
      }
    }
  }
   */
