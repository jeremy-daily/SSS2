

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
String softwareVersion = "SSS2*REV" + revision + "*0.91b*master*18887b06f7f46973e403baf2cbbdc876b832adba"; //Hash of the previous git commit


void listSoftware(){
  Serial.print("FIRMWARE ");
  Serial.println(softwareVersion);
}







/**************************************************************************************/
/*               Begin Function calls for User input data                             */


void listInfo() {
  Serial.print("INFO SSS2 Component ID (Make*Model*Serial*Unit): ");
  Serial.println(componentID);
  
}

void changeComponentID() {
  if (commandString.length() < 12) {
    Serial.print(F("INFO SSS2 Component ID: "));
    Serial.println(componentID);
  }
  else{
    componentID = commandString;
    setCompIdEEPROMdata();
    Serial.print(F("SET SSS2 Component ID: "));
    Serial.println(componentID);
    setupComponentInfo();
  } 
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
    setupComponentInfo();
    enableSendComponentInfo = true;
    Serial.println(F("SET Enable CAN transmission of Component ID"));
    if (canComponentIDtimer >5000){
      canComponentIDtimer = 0;
      can_messages[comp_id_index]->enabled = true;
      can_messages[comp_id_index]->transmit_number = 0;
      can_messages[comp_id_index]->ok_to_send = true;
      can_messages[comp_id_index]->loop_cycles = 0; 
      can_messages[comp_id_index]->cycle_count = 0;
      can_messages[comp_id_index]->message_index = 0;
    }
  
  }
  else{
    enableSendComponentInfo = false;
    Serial.println(F("SET Disable CAN transmission of Component ID"));  
    can_messages[comp_id_index]->enabled = false;
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
  
  //Serial.println(F("CANSEND - Send Message."));
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
      else Serial.println("ERROR Invalid Channel for CANSEND.");
    }
    
    else
      Serial.println(F("ERROR Invalid input data for CANSEND. Input should be using hex characters with no spaces in the form SM,channel,ID,data/n"));
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

void setupComponentInfo(){
   char byteEntry[4];
   uint8_t old_shortest_period = shortest_period;
   shortest_period = 1;
   uint16_t id_length = constrain(componentID.length(),0,7*256-1);
   char id[7*256];
   componentID.toCharArray(id,id_length+1);
   uint8_t num_frames = id_length/7;
   if (id_length % 7 > 0) num_frames++;
   char bytes_to_send[4]; 
   sprintf(bytes_to_send,"%02X",id_length);
   char frames_to_send[3];
   sprintf(frames_to_send,"%02X",num_frames);
   commandString = "CI from SSS2,0,1,0,0,1,0,1,1,18ECFF";
   sprintf(byteEntry,"%02X,",source_address);
   commandString += byteEntry; 
   commandString += "8,20,";
   commandString += bytes_to_send;
   commandString += ",0,";
   commandString += frames_to_send;
   commandString += ",FF,EB,FE,00";
   Serial.println(commandString);
   
   setupPeriodicCANMessage();
   
   
   for (int i = 0; i < num_frames; i++){
     commandString = "CI from SSS2,0,";
     sprintf(byteEntry,"%d,",num_frames+1);
     commandString += byteEntry; 
     sprintf(byteEntry,"%d,",i+1);
     commandString += byteEntry; 
     commandString += "0,1,0,";
     sprintf(byteEntry,"%d,",num_frames+1);
     commandString += byteEntry; 
     commandString += "1,18EBFF";
     sprintf(byteEntry,"%02X,",source_address);
     commandString += byteEntry; 
     commandString += "8,";
     sprintf(byteEntry,"%02X,",i+1);
     commandString += byteEntry; 
     for (int j = 7*i; j < 7*i+7;j++){
       if (j < id_length) sprintf(byteEntry,"%02X,",id[j]);
       else sprintf(byteEntry,"%02X,",0xFF);
       commandString += byteEntry;
     }
     Serial.println(commandString);
   
     comp_id_index = setupPeriodicCANMessage();
   }
   shortest_period = old_shortest_period;
  
    
}       

void reloadCAN(){
  setupComponentInfo();
        
  for (int i = 0; i < num_default_messages; i++){
    commandString = default_messages[i];
    setupPeriodicCANMessage();
    delay(1);
  }
  commandString = "1";
  goCAN();
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
  
  if (PGN == 0xEA00 && DA == 0xFF){
    //request message
    if (rxmsg.buf[0] == 0xEB && rxmsg.buf[1] == 0xFE){
      if (enableSendComponentInfo){
        if (canComponentIDtimer >5000){
          canComponentIDtimer = 0;
          can_messages[comp_id_index]->enabled = true;
          can_messages[comp_id_index]->transmit_number = 0;
          can_messages[comp_id_index]->ok_to_send = true;
          can_messages[comp_id_index]->loop_cycles = 0; 
          can_messages[comp_id_index]->cycle_count = 0;
          can_messages[comp_id_index]->message_index = 0;
        }
  
      }
    }
  }
}



//A generic CAN Frame print function for the Serial terminal
void printFrame(CAN_message_t rxmsg, int mailbox, uint8_t channel, uint32_t RXCount)
{ 

 Serial.printf("CAN%d %10lu.%06lu %08X %d %d %02X %02X %02X %02X %02X %02X %02X %02X\n",
          channel,now(),uint32_t(microsecondsPerSecond),rxmsg.id,rxmsg.ext,rxmsg.len,
          rxmsg.buf[0],rxmsg.buf[1],rxmsg.buf[2],rxmsg.buf[3],
          rxmsg.buf[4],rxmsg.buf[5],rxmsg.buf[6],rxmsg.buf[7]);
}


time_t getTeensy3Time(){
  microsecondsPerSecond = 0;
  return Teensy3Clock.get();
}

const uint32_t DEFAULT_TIME = 10; 
char timeStamp[45];

uint32_t processSyncMessage() {
  char timeChar[15];
  memset(timeChar,0,15);
  commandString.toCharArray(timeChar,15);
  time_t pctime = atol(timeChar);
  if (pctime > DEFAULT_TIME){
    return pctime;
  }
  else if (pctime == 0) {
    Serial.printf("INFO Time is %04d-%02d-%02d %02d:%02d:%02d\n",year(),month(),day(),hour(),minute(),second());
    return 0L;
  }
  else{
    Serial.println("ERROR Time value is invalid.");
    return 0L;
  }
}

void displayTime() {
  time_t t = processSyncMessage();
  if (t != 0) {
    Teensy3Clock.set(t); // set the RTC
    setTime(t);
    Serial.printf("SET Time to %04d-%02d-%02d %02d:%02d:%02d\n",year(),month(),day(),hour(),minute(),second());
  }
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
  LIN.begin(19200);
  
  commandString.reserve(256);
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
    
  LIN.begin(19200,SERIAL_8N2);
  //LIN.flush();
  //LIN.clear();


  J1708.begin(9600);
  J1708.clear();

  
  CANTimer.begin(runCANthreads, 1500); // Run can threads on an interrupt. Produces very little jitter.

  currentSetting = 1;
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;

  LINfinished = true;
  getCompIdEEPROMdata();
 
  commandString="1";
  setEnableComponentInfo();
  reloadCAN();
 
}

void loop() {
  //Always do this 
  //can_thread_controller.run();
  
  //Check CAN messages
  while (Can0.available()) {
    Can0.read(rxmsg);
    //parseJ1939(rxmsg);
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
      Serial.printf("J1708 %10lu.%06lu ",now(),uint32_t(microsecondsPerSecond));
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
  if (analog_tx_timer >= analog_display_period ){
    analog_tx_timer=0;
    if (send_voltage){
      Serial.print("ANALOG");
      for (uint8_t j = 0; j < numADCs; j++){
        Serial.printf(" %lu:%d",uint32_t(analogMillis),analogRead(analogInPins[j]));
      }
      Serial.print("\n");
    }
  }

  if (enableSendComponentInfo){
    if (canComponentIDtimer >5000){
      canComponentIDtimer = 0;
      can_messages[comp_id_index]->enabled = true;
      can_messages[comp_id_index]->transmit_number = 0;
      can_messages[comp_id_index]->ok_to_send = true;
      can_messages[comp_id_index]->loop_cycles = 0; 
      can_messages[comp_id_index]->cycle_count = 0;
      can_messages[comp_id_index]->message_index = 0;
    }
  }

  sendLINResponse();

  /****************************************************************/
  /*            Begin Serial Command Processing                   */
  if (Serial.available() >= 2 && Serial.available() < 256) {
    commandPrefix = Serial.readStringUntil(',');
    commandString = Serial.readStringUntil('\n');
    //commandString.trim();
    
    if      (commandPrefix.toInt() > 0)                   fastSetSetting();  
    else if (commandPrefix.equalsIgnoreCase("AI"))        displayVoltage();
    else if (commandPrefix.equalsIgnoreCase("B0"))        autoBaud0();
    else if (commandPrefix.equalsIgnoreCase("B1"))        autoBaud1();
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
    else if (commandPrefix.equalsIgnoreCase("GETTIME"))   Serial.printf("INFO Timestamp: %D\n",now());
    else if (commandPrefix.equalsIgnoreCase("LIN"))       displayLIN();
    else if (commandPrefix.equalsIgnoreCase("SENDLIN"))   sendLINselect();
    
    
    else {
      Serial.println(F("ERROR Unrecognized Command Characters. Use a comma after the command."));
      Serial.clear();
      //Serial.println(F("INFO Known commands are setting numbers, GO, SP, J1708, STOPCAN, STARTCAN, B0, B1, C0, C1, C2, DS, SW, OK, ID, STATS, CLEAR, MK, LI, LS, CI, CS, SA, SS, or SM."));
    }
  }
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
