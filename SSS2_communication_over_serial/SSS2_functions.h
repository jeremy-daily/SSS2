/*************************************************** 
  This is a library for the Smart Sensor Simulator 2
  Revision 3

  Written by Jeremy Daily for Synercon Technologies, LLC.  
  
 ****************************************************/
#define ENCODER_OPTIMIZE_INTERRUPTS

#include <SPI.h>
#include <i2c_t3.h>
#include <Encoder.h>
#include "OneButton.h"
#include "Adafruit_MCP23017.h"
#include <EEPROM.h>
#include <FlexCAN.h>
#include <TimeLib.h>
#include <TeensyID.h>
#include "Thread.h"
#include "ThreadController.h"
#include "FlexCAN.h"


Adafruit_MCP23017 ConfigExpander; //U21
Adafruit_MCP23017 PotExpander; //U33

IntervalTimer CANTimer;

boolean LIN_slave = true;
uint8_t LIN_address = 0xda;
elapsedMicros LINtimer;

//The Unique ID variable that comes from the chip
uint32_t uid[4];

uint8_t terminationSettings;


String commandPrefix;
String commandString;

#define numSettings  85
uint8_t source_address = 0xFA; 

int comp_id_index = 0;

/****************************************************************/
/*              Setup millisecond timers and intervals          */
//Declare a millisecond timer to execute the switching of the LEDs on a set time
elapsedMillis RXCAN0timer;
elapsedMillis RXCAN1orJ1708timer;
elapsedMillis analog_tx_timer;

elapsedMicros J1708RXtimer;
elapsedMicros microsecondsPerSecond;



/****************************************************************/
/*                 Binary Control Variables                       */

boolean ADJUST_MODE_ON  = 0;
boolean SAFE_TO_ADJUST = 0;
boolean displayCAN0 = 0;
boolean displayCAN1 = 0;
boolean displayCAN2 = 0;
boolean CAN0baudNotDetected = true;
boolean CAN1baudNotDetected = true;
boolean CAN2baudNotDetected = true;
boolean TXCAN = true;
boolean enableSendComponentInfo = true;
boolean sendA21voltage = false;
boolean showJ1708 = false;
boolean firstJ1708 = true;
boolean send_voltage = false;
uint16_t currentSetting = 0;
bool newJ1708Char = false;

uint8_t J1708RXbuffer[256];
uint8_t J1708_index=0;

uint32_t shortest_period = 10;

const int allFFs[8] = {255,255,255,255,255,255,255,255};
/****************************************************************/
/*                    Quadrature Knob Setup                     */

Encoder knob(encoderAPin,encoderBPin);
int32_t currentKnob = 0;

int knobLowLimit = 0;
int knobHighLimit = 255;
int knobJump = 1;

int LINbaud = 19200;
uint8_t LINIndex = 0;
boolean firstLINtime;
uint32_t serialSend0Micros;
uint32_t serialSend1Micros;
uint32_t serialSend2Micros;
uint32_t serialSend3Micros;
uint32_t serialSend4Micros;
uint8_t outByte[4];
boolean LIN0send;
boolean LIN1send;
boolean LIN2send;
boolean LIN3send;
boolean LIN4send;

void sendLINResponse(){
  uint32_t currentMicros = micros();
  if (LIN.available()>=3) 
  {
    byte firstChar = LIN.read();
    byte secondChar = LIN.read() ;
    byte thirdChar = LIN.read() ;
    if (firstChar == 0x00 && secondChar == 0xF0){
      //Serial.println("LIN Start");
      LIN.write(0xF0);
    }
    else if (firstChar == 0x00 && secondChar == 0x55 && thirdChar == 0x20 && firstLINtime){
        firstLINtime =false;
         LIN.write(0xFF);
         LIN.write(0xF);
         LIN.write(0xFF);
         LIN.write(0x3F);
         LIN.write(0x91);
         //Serial.println("first LIN Pass");
    }
    else if (firstChar == 0x00 && secondChar == 0x55 && thirdChar == 0x20 && !firstLINtime)//Serial.print(micros());
    {
      outByte[0] = 0x14;
      outByte[1] = (LINIndex << 4) + 0x01;
      LINIndex++;
      outByte[2] = 0x02;
      outByte[3] = 0x0D;

      //A bad way to calculate checksums
      if      (outByte[1]==0x01) outByte[4] =0xBB;
      else if (outByte[1]==0x11) outByte[4] =0xAB;
      else if (outByte[1]==0x21) outByte[4] =0x9B;
      else if (outByte[1]==0x31) outByte[4] =0x8B;
      else if (outByte[1]==0x41) outByte[4] =0x7B;
      else if (outByte[1]==0x51) outByte[4] =0x6B;
      else if (outByte[1]==0x61) outByte[4] =0x5B;
      else if (outByte[1]==0x71) outByte[4] =0x4B;
      else if (outByte[1]==0x81) outByte[4] =0x3B;
      else if (outByte[1]==0x91) outByte[4] =0x2B;
      else if (outByte[1]==0xA1) outByte[4] =0x1B;
      else if (outByte[1]==0xB1) outByte[4] =0x0B;
      else if (outByte[1]==0xC1) outByte[4] =0xFA;
      else if (outByte[1]==0xD1) outByte[4] =0xEA;
      else if (outByte[1]==0xE1) outByte[4] =0xDA;
      else if (outByte[1]==0xF1) outByte[4] =0xCA;

      serialSend0Micros = currentMicros+500;
      LIN0send = true;
      serialSend1Micros = currentMicros+1000;
      LIN1send = true;
      serialSend2Micros = currentMicros+1500;
      LIN2send = true;
      serialSend3Micros = currentMicros+2000;
      LIN3send = true;
      serialSend4Micros = currentMicros+2500;
      LIN4send = true;
      

    }
  }    

  if (currentMicros - serialSend0Micros >=0 && LIN0send) {
    LIN.write(outByte[0]);
    LIN0send=false;
  }
  if (currentMicros - serialSend1Micros >=0 && LIN1send) {
    LIN.write(outByte[1]);
    LIN1send=false;
  }
  if (currentMicros - serialSend2Micros >=0 && LIN2send) {
    LIN.write(outByte[2]);
    LIN2send=false;
  }
  if (currentMicros - serialSend3Micros >=0 && LIN3send){
    LIN.write(outByte[3]);
    LIN3send=false;
  }
  if (currentMicros - serialSend4Micros >=0 && LIN4send) {
    LIN.write(outByte[4]);
    LIN4send=false;
  }
}

class SensorThread: public Thread
{
public:
  int reading;
  int pin;
   
  
  bool shouldRun(unsigned long time){
    return Thread::shouldRun(time);
  }
  
  void run(){
   
    reading = analogRead(pin);
    Serial.printf("A%d,%d\n",pin,reading);
    runned(); 
  }
};

SensorThread analog1 = SensorThread();

void analogTimerCallback(){
  analog1.run();
}



void displayVoltage(){
  
  if (commandString.toInt() > 0){
    send_voltage = true;
    Serial.println("SET Stream analog in data on."); 
  }
  else {
    Serial.println("SET Stream analog In data off.");
    send_voltage = false;
  }
}


/****************************************************************/
/*                    CAN Setup                                 */
//Setup messages
#define num_default_messages  25
String default_messages[num_default_messages] = {
 "DDEC MCM 01,                   1,1,0,1,  10,   0,0,1, 8FF0001,8, 0, 0, 0, 0, 0, 0, 0, 0", //DDEC 13 MCM message, CAN1
 "DDEC TCM 01,                   2,1,0,1,  10,   0,0,1, CF00203,8, 0, 0, 0, 0, 0, 0, 0, 0", //DDEC13 Transmission controler message, CAN1
 "DDEC TCM 02,                   3,1,0,1,  10,   0,0,1, 8FF0303,8, 0, 0, 0, 0, 0, 0, 0, 0", //DDEC 13 TCM message, CAN1
 "DDEC TCM 03,                   4,1,0,1, 100,   0,0,1,18F00503,8, 0, 0, 0, 0, 0, 0, 0, 0", //Transmission on DDEC 13
 "HRW from Brake Controller,     5,1,0,0,  20,   0,0,1, CFE6E0B,8, 0, 0, 0, 0, 0, 0, 0, 0", //High Resolution wheel speed message from SA=11 (brake controller)
 "EBC1 from Cab Controller,      6,1,0,0, 100,   0,0,1,18F00131,8, 0, 0, 0, 0, 0, 0, 0, 0", // Electronic Brake Controller from SA=49
 "EBC1 from Brake Controller,    7,1,0,0, 100,   0,0,1,18F0010B,8, 0, 0, 0, 0, 0, 0, 0, 0", // Electronic Brake Controller from SA=11
 "CCVS1 from Instrument Cluster, 8,1,0,0, 100,   0,0,1,18FEF117,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise Control/Vehicle Speed from SA=23
 "CCVS1 from Cab Display 1,      9,1,0,0, 100,   0,0,1,18FEF128,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise Control/Vehicle Speed from SA=40
 "CCVS1 from Body Controller,   10,1,0,0, 100,   0,0,1,18FEF121,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise Control/Vehicle Speed from SA=33
 "CCVS1 from Cab Controller,    11,1,0,0, 100,   0,0,1,18FEF131,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise Control/Vehicle Speed from SA=49
 "CM1 from Instrument Cluster,  12,1,0,0, 100,   0,0,1,18E00017,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cab Message 1 from SA=23
 "CM1 from Climate Control 1,   13,1,0,0, 100,   0,0,1,18E00019,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cab Message 1 from SA=25
 "CM1 from Body Controller,     14,1,0,0, 100,   0,0,1,18E00021,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cab Message 1 from SA=33
 "CM1 from Cab Display,         15,1,0,0, 100,   0,0,1,18E00028,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cab Message 1 from SA=40
 "CM1 from Cab Controller,      16,1,0,0, 100,   0,0,1,18E00031,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cab Message 1 from SA=49
 "PTO from Instrument Cluster,  17,1,0,0, 100,   0,0,1,18FEF017,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise and PTO setup from SA=23
 "PTO from Body Controller,     18,1,0,0, 100,   0,0,1,18FEF021,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise and PTO setup from SA=33
 "PTO from Cab Display,         19,1,0,0, 100,   0,0,1,18FEF028,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise and PTO setup from SA=40
 "PTO from Cab Controller,      20,1,0,0, 100,   0,0,1,18FEF031,8, 0, 0, 0, 0, 0, 0, 0, 0", //Cruise and PTO setup from SA=49
 "DDEC Fault Codes from MCM,    21,2,0,1,   5,1000,0,1,10ECFF01,8,20,0E,00,01,FF,CA,FE,00", //TP.CM Session Control Message for DDEC
 "DDEC Fault Codes from MCM,    21,2,1,1,   5,1000,0,1,10EBFF01,8,01, 0, 0, 0, 0, 0, 0, 0", //TP.DT
 "DDEC Fault Codes from ACM,    22,2,0,1,   5,1000,0,1,10ECFF3D,8,20,0E,00,01,FF,CA,FE,00", //TP.CM Session Control Message for DDEC
 "DDEC Fault Codes from ACM,    22,2,1,1,   5,1000,0,1,10EBFF3D,8,01, 0, 0, 0, 0, 0, 0, 0", //TP.DT
 "AMB from Body Controller,     23,1,0,1,1000,   0,0,1,18FEF521,8, 0, 0, 0, 0, 0, 0, 0, 0" //Ambient Conditions
};




//Set up the CAN data structures
static CAN_message_t rxmsg;
static CAN_message_t txmsg;
static CAN_message_t temp_txmsg;

//set up a counter for each received message
uint32_t RXCount0 = 0;
uint32_t RXCount1 = 0;

int CANWaitTimeout = 200;
uint32_t BAUDRATE0 = 250000;
uint32_t BAUDRATE1 = 250000;
const uint32_t baudRateList[5] = {250000,500000,666666,125000,1000000};
uint8_t baudRateIndex0 = 0;
uint8_t baudRateIndex1 = 0;


CAN_filter_t allPassFilter;

// Create a thread controller class
ThreadController can_thread_controller = ThreadController();

void runCANthreads(){
  can_thread_controller.run();
}


class CanThread: public Thread
{
public:
  uint32_t stop_after_count;
  uint32_t transmit_number = 0;
  boolean ok_to_send = true;
  uint32_t loop_cycles = 0; 
  uint32_t cycle_count = 0;
  
  uint8_t channel = 0;
 
  CAN_message_t txmsg;
    
  uint8_t num_messages = 1; 
  uint8_t message_index = 0;
  uint8_t message_list[256][8] = {};
  uint32_t id_list[256]={};
 
  
  
  bool shouldRun(unsigned long time){
    if (stop_after_count > 0){
      if (transmit_number >= stop_after_count) enabled = false;
    }
    return Thread::shouldRun(time);
  }
  
  void run(){
    //Set the CAN message data to the next one in the list.
    txmsg.id = id_list[message_index];
    memcpy(txmsg.buf,message_list[message_index],8);

    //Write the data to the CAN bus.
    if (ok_to_send && ignitionCtlState){
      if      (channel == 0) Can0.write(txmsg);
      else if (channel == 1) Can1.write(txmsg);
      
      transmit_number++;
      message_index++;
    }
    cycle_count++;
    if (message_index >= num_messages ) {
      message_index = 0; 
    }
    if (cycle_count >= num_messages) {
      ok_to_send = false;
    }
    if (cycle_count*interval >= loop_cycles){
      cycle_count = 0;
      ok_to_send = true;
    }
  
    
    return Thread::run();
  }
};

//Setup a container to keep track of the individual CAN threads
CanThread* can_messages[MAX_THREADS] ={};

void set_shortest_period(){
  if (commandString.length() > 0){
    shortest_period = commandString.toInt();
    Serial.printf("SET Shortest CAN Broadcast Period to %lu milliseconds.\n",shortest_period);
  }
  else
     Serial.printf("INFO Shortest CAN Broadcast Period is %lu milliseconds.\n",shortest_period);
}

void getThreadName(){
  int index = commandString.toInt();
  Serial.printf("NAME of CAN Thread %d: ",index); 
  Serial.println(can_messages[index]->ThreadName);
}
void getAllThreadNames(){
  int threadSize =  can_thread_controller.size(false);
  for(int i = 0; i<threadSize; i++){
    Serial.printf("NAME of CAN Thread %d: ",i); 
    Serial.println(can_messages[i]->ThreadName);
  }
  
}


void getThreadSize(){
  int threadSize =  can_thread_controller.size(false);
  Serial.printf("INFO Size of CAN Thread = %d\n",threadSize); 
}

int setupPeriodicCANMessage(){
  CANTimer.end();
  CanThread* can_message;
  int index;
  int sub_index;
  int channel;
  uint32_t tx_period;
  uint32_t tx_delay;
  uint8_t num_messages=1;
  uint32_t stop_after_count;
   
  int threadSize =  can_thread_controller.size(false);
  char commandBytes[256];
  commandString.toCharArray(commandBytes,256);
  char delimiter[] = ",";
  char* commandValues;

  commandValues = strtok(commandBytes, delimiter);
  String threadName = commandValues;
  
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    index = constrain(atoi(commandValues),0,threadSize);
  }
  else {
    Serial.println(F("ERROR SM command is missing arguments.")); 
    return -1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    num_messages = constrain(atoi(commandValues),1,255);
  }
  else {
    Serial.println(F("ERROR SM command not able to determine the number of sub messages.")); 
    return -2;
  }


  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
     //constrain the sub_index to be one larger than the 
     sub_index = constrain(atoi(commandValues),0,num_messages-1);
  }
  else {
    Serial.println(F("ERROR SM command missing sub_index.")); 
    return -3;
  }
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    channel = constrain(atoi(commandValues),0,1);
  }
  else {
    Serial.println(F("ERROR SM command not able to set CAN Channel.")); 
    return -4;
  }
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    tx_period = constrain(strtoul(commandValues,NULL,10),shortest_period,0xFFFFFFFF);
  }
  else {
    Serial.println(F("ERROR SM command not able to set period information.")); 
    return -5;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    tx_delay = strtoul(commandValues,NULL,10);
  }
  else {
    Serial.println(F("ERROR SM command not able to set delay information.")); 
    return -6;
  }
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    stop_after_count = strtoul(commandValues,NULL,10);
  }
  else {
    Serial.println(F("ERROR SM command not able to set the total number count.")); 
    return -7;
  }
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    temp_txmsg.ext = constrain(atoi(commandValues),0,1);
  }
  else {
    Serial.println(F("ERROR SM command not able to set extended ID flag.")); 
    temp_txmsg.ext = 1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    temp_txmsg.id = strtol(commandValues,NULL,16);
  }
  else {
    Serial.println(F("WARNING SM command not able to set CAN ID information."));
    temp_txmsg.id = 0x3FFFFFFF ;
  }
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    temp_txmsg.len = constrain(atoi(commandValues),0,8);
  }
  else {
    Serial.println(F("WARNING SM command not able to set CAN data length code."));
    temp_txmsg.len = 8;
  }
 
  //memset(temp_txmsg.buf,allFFs,8);
  for (int i = 0; i < temp_txmsg.len; i++){
    commandValues = strtok(NULL, delimiter);
    if (commandValues != NULL) {
      temp_txmsg.buf[i] = constrain(strtol(commandValues,NULL,16),0,255);
    }
    else {
      temp_txmsg.buf[i] = 0xFF;
      Serial.printf("WARNING SM command not able to set CAN data byte in position %d.\n",i);
    }
  }
  char threadNameChars[256]; 
  threadName.toCharArray(threadNameChars,threadName.length()+1);
  
  Serial.printf("SET CAN name=%s, i=%d, n=%d, j=%d, c=%d, p=%d, d=%d, t=%d, e=%d, ID=%08X, DLC=%d, DATA=[",
                 threadNameChars,index, num_messages, sub_index, channel, tx_period, tx_delay, stop_after_count, temp_txmsg.ext, temp_txmsg.id,temp_txmsg.len);
  for (int i = 0; i < temp_txmsg.len-1; i++){
    Serial.printf("%02X, ",temp_txmsg.buf[i]);
  }
  Serial.printf("%02X]\n",temp_txmsg.buf[temp_txmsg.len-1]);
  
  if (index == threadSize) { //Create a new entry
    Serial.printf("THREAD %lu, %s (NEW)\n",index,threadNameChars); 
    CanThread* can_message = new CanThread(); 
    can_thread_controller.add(can_message);
    can_messages[index] = can_message;
    can_messages[index]->enabled = false;
  }
  else{
   Serial.printf("THREAD %lu, %s (EXISTING)\n",index,threadNameChars); 
  }

  can_messages[index]->channel = channel;
  can_messages[index]->txmsg.ext = temp_txmsg.ext;
  can_messages[index]->txmsg.len = temp_txmsg.len;
  can_messages[index]->id_list[sub_index] = temp_txmsg.id;
  for (int i = 0; i < temp_txmsg.len; i++) {
    can_messages[index]->message_list[sub_index][i] = temp_txmsg.buf[i];
  }
  can_messages[index]->stop_after_count = stop_after_count;
  can_messages[index]->transmit_number = 0;
  can_messages[index]->cycle_count = 0;
  can_messages[index]->message_index = 0;
  can_messages[index]->num_messages = num_messages;  
  can_messages[index]->setInterval(tx_period);
  can_messages[index]->loop_cycles =  tx_delay ;
  can_messages[index]->ThreadName = threadName;
  
  CANTimer.begin(runCANthreads, 500);
  return index;
}

void stopCAN(){
  int threadSize =  can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++) {
    can_messages[i]->enabled = false;
    }
  Serial.println(F("INFO Stopped all CAN transmission."));
}

void clearCAN(){
  int threadSize =  can_thread_controller.size(false);
  for (int i = 0; i < sizeof(can_messages); i++) {
    delete can_messages[i] ;
    }
  can_thread_controller.clear();
  Serial.println(F("INFO Cleared the CAN transmission thread. All messages must be reloaded."));
}

void goCAN(){
  int threadSize =  can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++) {
    can_messages[i]->enabled = true;
    can_messages[i]->transmit_number = 0;
    can_messages[i]->message_index = 0;
    can_messages[i]->cycle_count = 0;
  }
  Serial.println(F("INFO Started all CAN transmissions."));
}

void startCAN (){
  int threadSize =  can_thread_controller.size(false);
  if (threadSize > 0){
    char commandBytes[10];
    commandString.toCharArray(commandBytes,10);
    char delimiter[] = ",";
    char* commandValues;
    commandValues = strtok(commandBytes, delimiter);
    int index = constrain(atoi(commandValues),0,threadSize-1);
    
    commandValues = strtok(NULL, delimiter);
    int setting = atoi(commandValues);
    
    if (setting > 0) {
      can_messages[index]->enabled = true;
      can_messages[index]->transmit_number = 0;
      can_messages[index]->message_index = 0;
      can_messages[index]->cycle_count = 0;
      Serial.printf("SET CAN message %d with ID 0x%08X on.\n",index,can_messages[index]->id_list[0]); 
    }
    else  {
      can_messages[index]->enabled = false;
      Serial.printf("SET CAN message %d with ID 0x%08X off.\n",index,can_messages[index]->id_list[0]); 
    }
  }
  else
  {
     Serial.println("ERROR No CAN Messages Setup to turn on.");
  }
}

class settingsData {
public:
  uint32_t knobLowLimit = 0;
  uint32_t knobHighLimit = 255;
  uint32_t knobJump = 1;
  String settingName = "";
  String settingPort = "";
  
};



void connectionString(boolean switchState) {
  //memset(displayBuffer,0,sizeof(displayBuffer));
  Serial.print(switchState);
  if (switchState) strcpy(displayBuffer, " Connected");
  else strcpy(displayBuffer, " Open");
}

void terminalString(uint8_t setting) {
  switch (setting) {
    case TCON_B_ONLY:
      strcpy(displayBuffer, " 1 (TCON_B_ONLY)");
      break;
    case TCON_WIPER_ONLY:
      strcpy(displayBuffer, "2 (TCON_WIPER_ONLY)");
      break;
    case TCON_WIPER_AND_B:
      strcpy(displayBuffer, "3 (TCON_WIPER_AND_B)");
      break;
    case TCON_A_ONLY:
      strcpy(displayBuffer, "4 (TCON_A_ONLY)");
      break;
    case TCON_A_AND_B :
      strcpy(displayBuffer, "5 (TCON_A_AND_B)");
      break;
    case TCON_WIPER_AND_A :
      strcpy(displayBuffer, "6 (TCON_WIPER_AND_A)");
      break;
    case TCON_CONNECT_ALL:
      strcpy(displayBuffer, "7 (TCON_CONNECT_ALL)");
      break;
    default:
      strcpy(displayBuffer, "0 Nothing connected");
      break;
  }
}


/**********************************************************************/
/*   Utility functions to manipulate Ports                            */

uint8_t setTerminationSwitches() {
  //Set the termination Switches of U29 on Rev 3

  uint8_t terminationSettings =  uint8_t( CAN0term | CAN1term << 1 | CAN2term << 2 |  LINmaster << 3 | 
                                 PWM1Out << 4 | PWM2Out << 5 | PWM3Out << 6 | PWM4Out << 7);
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
              LIN1Switch | LIN2Switch  << 1 | P10or19Switch << 2 |  P15or18Switch << 3 | !U1though8Enable << 4 | !U9though16Enable << 5 |
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


/****************************************************************/
/*   Begin Function Calls for Digital Potentiometer             */
  
uint8_t MCP41HVExtender_SetTerminals(uint8_t pin, uint8_t TCON_Value) {
  PotExpander.writeGPIOAB(~(1 << pin));
  delay(1);
  SPI.transfer(0x40); //Write to TCON Register
  SPI.transfer(TCON_Value + 8);
  SPI.transfer(0x4C); //Read Command
  uint8_t result = SPI.transfer(0xFF); //Read Terminal Connection (TCON) Register
  PotExpander.writeGPIOAB(0xFF);
  return result & 0x07;
}



uint8_t MCP41HVExtender_SetWiper(uint8_t pin, uint8_t potValue)
{
  PotExpander.writeGPIOAB(~(1 << pin));
  delay(1);
  SPI.transfer(0x00); //Write to wiper Register
  SPI.transfer(potValue);
  SPI.transfer(0x0C); //Read command
  uint8_t result = SPI.transfer(0xFF); //Read Wiper Register
  PotExpander.writeGPIOAB(0xFF);
  return result;
} 

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
  uint8_t result = Wire.read(); //Read Wiper Register, first byt is all 0
  result = Wire.read(); //Read Wiper Register
  return result;
} 

/*   End Function Calls for Digital Potentiometer               */
/****************************************************************/

/****************************************************************/
/*   Begin Function Calls for DAC                               */

void initializeDACs(uint8_t address) {
  Serial.print("Setting DAC Internal Reference register with address of 0x");
  Serial.println(address, HEX);
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b10000000);
  Wire.write(0x00);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b10000000);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  char highC = Wire.read();
  char lowC = Wire.read();
  Serial.print("Internal Reference Register: ");
  Serial.println(lowC);         // print the character

  Serial.println("Setting LDAC register.");
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01100000);
  Wire.write(0xFF);
  Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01100000);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  highC = Wire.read();
  lowC = Wire.read();
  Serial.print("LDAC Register: ");
  Serial.println(lowC, HEX);        // print the character

  Serial.println("Setting DAC Power Down register to On.");
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01000000);
  Wire.write(0b00011111);
  Wire.write(0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b01000000);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  highC = Wire.read();
  lowC = Wire.read();
  Serial.print("Power Register: ");
  Serial.println(lowC, BIN);        // print the character

  Serial.print(F("Done with DAC at address 0x"));
  Serial.println(address, HEX);
}

uint16_t setDAC(uint16_t setting, uint8_t DACchannel, uint8_t address) {
  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b00100000 + DACchannel);
  Wire.write((setting & 0x0FF0) >> 4);
  Wire.write((setting & 0x000F) << 4);
  Wire.endTransmission();

  Wire.beginTransmission(address);   // Slave address
  Wire.write(0b00010000 + DACchannel);
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  uint8_t highC = Wire.read();
  uint8_t lowC =  Wire.read();
  uint16_t result = (highC << 4) + (lowC >> 4);
  return result;
}

/*   End Function Calls for DAC                              */
/****************************************************************/

/********************************************************************/
/*               Begin Define Settings                              */
/*
 * To add another setting, update the numSettings, populate the settingNames and settingPins array and add 
 * new knob limits. The knob limits are used for value checking. These are for numerical Settings.
 */


char settingNames[numSettings][40] = {
  "Nothing Selected", //0
  "Digital Potentiometer  1 Wiper", //1
  "Digital Potentiometer  2 Wiper",
  "Digital Potentiometer  3 Wiper",
  "Digital Potentiometer  4 Wiper",
  "Digital Potentiometer  5 Wiper",
  "Digital Potentiometer  6 Wiper",
  "Digital Potentiometer  7 Wiper",
  "Digital Potentiometer  8 Wiper",
  "Digital Potentiometer  9 Wiper",
  "Digital Potentiometer 10 Wiper",
  "Digital Potentiometer 11 Wiper",
  "Digital Potentiometer 12 Wiper",
  "Digital Potentiometer 13 Wiper",
  "Digital Potentiometer 14 Wiper",
  "Digital Potentiometer 15 Wiper",
  "Digital Potentiometer 16 Wiper", //16

  "Vout2-A", //17
  "Vout2-B",
  "Vout2-C",
  "Vout2-D",
  "Vout2-E",
  "Vout2-F",
  "Vout2-G",
  "Vout2-H", //24

  "U1 & U2 P0A", //25
  "U3 & U4 P0A", 
  "U5 & U5 P0A", 
  "U7 & U2 P0A", 
  "U9 & U10 P0A", 
  "U11 & U12 P0A", 
  "U13 & U14 P0A", 
  "U15 & U16 P0A", //32
  
  "PWM 1", //33
  "PWM 2",
  "PWM 3",
  "PWM 4", //36

  "Port 10 or 19", //37
  "Port 15 or 18",
  "CAN1 or J1708",
  "Port 31 & 32 or CAN2",
  "CAN0 Termination Resistor",
  "CAN1 Termination Resistor",
  "CAN2 Termination Resistor",
  "LIN Master Pullup Resistor", //44

  "12V Out 1 (H-Bridge)", //45
  "12V Out 2 (H-Bridge)",
  "Ground Out 1 (H-Bridge)",
  "Ground Out 2 (H-Bridge)", //48

  "High Voltage Adjustable Output", //49
  "Ignition Relay",
  
  "Dig. Pot.  1 Terminal Connect", //51
  "Dig. Pot.  2 Terminal Connect",
  "Dig. Pot.  3 Terminal Connect",
  "Dig. Pot.  4 Terminal Connect",
  "Dig. Pot.  5 Terminal Connect",
  "Dig. Pot.  6 Terminal Connect",
  "Dig. Pot.  7 Terminal Connect",
  "Dig. Pot.  8 Terminal Connect",
  "Dig. Pot.  9 Terminal Connect",
  "Dig. Pot. 10 Terminal Connect",
  "Dig. Pot. 11 Terminal Connect",
  "Dig. Pot. 12 Terminal Connect",
  "Dig. Pot. 13 Terminal Connect",
  "Dig. Pot. 14 Terminal Connect",
  "Dig. Pot. 15 Terminal Connect",
  "Dig. Pot. 16 Terminal Connect", //66

  "PWM1 Connect", //67
  "PWM2 Connect",
  "PWM3 Connect",
  "PWM4 Connect",
  "LIN to Shield Connect", //U21 GPA0
  "LIN to Port 16 Connect", //U21 GPA1
  "U28 (U1-U8)  P0A Enable",
  "U31 (U9-U16) P0A Enable", //74
  
  "Digital Potentiometer 28 Wiper", //75
  "Digital Potentiometer 29 Wiper",
  "Digital Potentiometer 30 Wiper",
  
  "Dig. Pot. 28 Terminal Connect",
  "Dig. Pot. 29 Terminal Connect",
  "Dig. Pot. 30 Terminal Connect", //80

  "PWM1 Frequency",
  "PWM2 Frequency",
  "PWM3 Frequency",
  "PWM4 Frequency"
  
};

char settingPins[numSettings][40] = {
  "\n",
  "Port  1 (J24- 1)",
  "Port  2 (J24- 2)",
  "Port  3 (J24- 3)",
  "Port  4 (J24- 4)",
  "Port  5 (J24- 5)",
  "Port  6 (J24- 6)",
  "Port  7 (J24- 7)",
  "Port  8 (J24- 8)",
  "Port  9 (J24- 9)",
  "Port 10 (J24-10)",
  "Port 11 (J24-11)",
  "Port 12 (J24-12)",
  "Port 13 (J18-11)",
  "Port 14 (J18-12)",
  "Port 15 (J24-15)",
  "Port 16 (J24-16)",

  "Port 18 (J18- 2)",
  "Port 19 (J18- 3)",
  "Port 20 (J18- 4)",
  "Port 21 (J18- 5)",
  "Port 22 (J18- 6)",
  "Port 23 (J18- 7)",
  "Port 24 (J18- 8)",
  "Port 25 (J18- 9)",

  "Ports  1 and 2",
  "Ports  3 and 4",
  "Ports  5 and 6",
  "Ports  7 and 8",
  "Ports  9 and 10",
  "Ports 11 and 12",
  "Ports 13 and 14",
  "Ports 15 and 16",

  "Ports 13 (J24-13) and 31 (J18-15)",
  "Ports 14 (J24-14) and 32 (J18-16)",
  "Port 27 (J18-10)",
  "Port 17 (J18-1)",
  
  "(J24-10)",
  "(J24-15)",
  "(J24-17 & J24-18)",
  "(J18-15 and J18-16)",
  "R44",
  "R45",
  "R46",
  "R59",

  "Port 26 (J18-10)",
  "Port 11 (J24-11)",
  "Port 17 (J18- 1)",
  "Port 12 (J24-12)",

  "(J24-19 and J18-11)",
  "(J24-20)",
  
  "Port  1 (J24- 1)",
  "Port  2 (J24- 2)",
  "Port  3 (J24- 3)",
  "Port  4 (J24- 4)",
  "Port  5 (J24- 5)",
  "Port  6 (J24- 6)",
  "Port  7 (J24- 7)",
  "Port  8 (J24- 8)",
  "Port  9 (J24- 9)",
  "Port 10 (J24-10)",
  "Port 11 (J24-11)",
  "Port 12 (J24-12)",
  "Port 13 (J18-11)",
  "Port 14 (J18-12)",
  "Port 15 (J24-15)",
  "Port 16 (J24-16)",

  "Port 13 (J24-13)",
  "Port 14 (J24-14)",
  "Port 27 (J18-10)",
  "Port 17 (J18- 1)",
  "(J10- 5)",
  "Port 16 (J24-16)",
  "(J24-1 to J24-8)",
  "(J24-9 to J24-16)",
  
  "Port 28 (J18-12)",
  "Port 29 (J18-13)",
  "Port 30 (J18-14)",
  
  "Port 28 (J18-12)",
  "Port 29 (J18-13)",
  "Port 30 (J18-14)",

  "Port 13 (J24-13)",
  "Port 14 (J24-14)",
  "Port 27 (J18-10)",
  "Port 17 (J18-1)"
};


void setLimits(uint8_t settingNum) {
  if (settingNum > 0 && settingNum <= 16) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum > 16 && settingNum <= 24) {
    knobLowLimit = 0;
    knobHighLimit = 4095;
    knobJump = 20;
  }
  else if (settingNum > 24 && settingNum <= 32) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum > 32 && settingNum <= 36) {
    knobLowLimit = 0;
    knobHighLimit = 4096;
    knobJump = 1;
  }
  else if (settingNum > 36 && settingNum <= 48) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 49) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum == 50) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
 
  else if (settingNum >= 51 && settingNum <= 66) {
    knobLowLimit = 0;
    knobHighLimit = 7;
    knobJump = 1;
  }
  else if (settingNum >= 67 && settingNum <= 74) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum > 74 && settingNum <= 77) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum >= 77 && settingNum <= 80) {
    knobLowLimit = 0;
    knobHighLimit = 7;
    knobJump = 1;
  }
  else if (settingNum >= 81 && settingNum <= 84) {
    knobLowLimit = 0;
    knobHighLimit = 32768;
    knobJump = 1;
  }
  else {
    knobLowLimit = 1;
    knobHighLimit = numSettings-1;
    knobJump = 1;
  }

  // Serial.print("Low Limit: ");
  // Serial.println(knobLowLimit);
  // Serial.print("High Limit: ");
  // Serial.println(knobHighLimit);
}

int16_t setSetting(uint8_t settingNum, int settingValue, bool debugDisplay) {
  
  
  if (debugDisplay){
    Serial.print(settingNum);
    Serial.print(", ");
    Serial.print(settingNames[settingNum]);
    Serial.print(", ");
    Serial.print(settingPins[settingNum]);
    Serial.print(" = ");
  }
  if (settingNum > 0 && settingNum <= 16){
    if (settingValue > -1) SPIpotWiperSettings[settingNum - 1] = settingValue; 
    uint8_t w_position = MCP41HVExtender_SetWiper(settingNum - 1, 
                                                SPIpotWiperSettings[settingNum - 1]);
    if (debugDisplay) {
        Serial.print(w_position);
        Serial.print(", ");
        Serial.println(SPIpotWiperSettings[settingNum - 1]);
    }
    return w_position;
  }
  else if (settingNum > 16 && settingNum <= 24) {
    if (settingValue > -1) DAC2value[settingNum - 17] = settingValue;
    setDAC(DAC2value[settingNum - 17], settingNum - 17, Vout2address);
    if (debugDisplay) Serial.println(DAC2value[settingNum - 17]); 
    return DAC2value[settingNum - 17];
  }
  else if (settingNum == 25){
    if (settingValue > -1) U1U2P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U1U2P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U1U2P0ASwitch;
  }
  else if (settingNum == 26){
    if (settingValue > -1) U3U4P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U3U4P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U3U4P0ASwitch;
  }
  else if (settingNum == 27){
    if (settingValue > -1) U5U6P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U5U6P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U5U6P0ASwitch;
  }
  else if (settingNum == 28){
    if (settingValue > -1) U7U8P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U7U8P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U7U8P0ASwitch;
  } 
  else if (settingNum == 29){
    if (settingValue > -1) U9U10P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U9U10P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U9U10P0ASwitch;
  }  
  else if (settingNum == 30){
    if (settingValue > -1) U11U12P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U11U12P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U11U12P0ASwitch;
  }  
  else if (settingNum == 31){
    if (settingValue > -1) U13U14P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U13U14P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U13U14P0ASwitch;
  }   
  else if (settingNum == 32){
    if (settingValue > -1) U15U16P0ASwitch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U15U16P0ASwitch);
        Serial.println(displayBuffer);
    }
    return U15U16P0ASwitch;
  }
  else if (settingNum >= 33 && settingNum <= 36 ){
    if (settingValue > -1) pwmValue[settingNum - 33] = uint16_t(settingValue);
    analogWrite(PWMPins[settingNum - 33],pwmValue[settingNum - 33]);
    if (debugDisplay) Serial.println(pwmValue[settingNum - 33]);
    return pwmValue[settingNum - 33];
  }
  else if (settingNum == 37){
    if (settingValue > -1) P10or19Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(P10or19Switch);
        Serial.println(displayBuffer);
    }
    return P10or19Switch;
  }
  else if (settingNum == 38){
    if (settingValue > -1) P15or18Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(P15or18Switch);
        Serial.println(displayBuffer);
    }
    return P15or18Switch;
  } 
  else if (settingNum == 39){
    if (settingValue > -1) CAN1Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(CAN1Switch);
        Serial.println(displayBuffer);
    }
    return CAN1Switch;
  }  
  else if (settingNum == 40){
    if (settingValue > -1) CAN2Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(CAN2Switch);
        Serial.println(displayBuffer);
    }
    return CAN2Switch;
  }
  else if (settingNum == 41){
    if (settingValue > -1) CAN0term = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(CAN0term);
        Serial.println(displayBuffer);
    }
    return CAN0term;
  } 
  else if (settingNum == 42) {
    if (settingValue > -1) CAN1term = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(CAN1term);
        Serial.println(displayBuffer);
    }
    return CAN1term;
  } 
  else if (settingNum == 43){
    if (settingValue > -1) CAN2term = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(CAN2term);
        Serial.println(displayBuffer);
    }
    return CAN2term;
  }  
  else if (settingNum == 44) {
    if (settingValue > -1) LINmaster = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(LINmaster);
        Serial.println(displayBuffer);
    }
    return LINmaster;
  }  
  else if (settingNum == 45)  {
    if (settingValue > -1) IH1State = boolean(settingValue);
    if (IH1State) PWM3Out = false;
    else PWM3Out = true;
    
    setTerminationSwitches();
    digitalWrite(IH1Pin,IH1State);
    if (debugDisplay) {
        connectionString(IH1State);
        Serial.println(displayBuffer);
    }
    return IH1State;
  }  
  else if (settingNum == 46) {
    if (settingValue > -1) IH2State = boolean(settingValue);
    if (IH2State) MCP41HVExtender_SetTerminals(11, 0); //Turn off all terminals on Pot 11
    else MCP41HVExtender_SetTerminals(11, SPIpotTCONSettings[10]); //Reset all terminals on Pot 11
    digitalWrite(IH2Pin,IH2State);
    if (debugDisplay) {
        connectionString(IH2State);
        Serial.println(displayBuffer);
    }
    return IH2State;
  }
  else if (settingNum == 47){
    if (settingValue > -1) IL1State = boolean(settingValue);
    if (IL1State) PWM4Out = false;
    else PWM4Out = true; 
    setTerminationSwitches(); //Turn off PWM4
    digitalWrite(IL1Pin,IL1State);
    if (debugDisplay) {
        connectionString(IL1State);
        Serial.println(displayBuffer);
    }
    return IL1State;
  }
  else if (settingNum == 48) {
    if (settingValue > -1) IL2State = boolean(settingValue);
    if (IL2State) MCP41HVExtender_SetTerminals(12, 0); //Turn off all terminals on Pot 12
    else MCP41HVExtender_SetTerminals(12, SPIpotTCONSettings[11]); //Reset all terminals 
    digitalWrite(IL2Pin,IL2State);
    if (debugDisplay) {
        connectionString(IL2State);
        Serial.println(displayBuffer);
    }
    return IL2State;
  }
  else if (settingNum == 49) {
    if (settingValue > -1) HVoutAdjValue = uint8_t(settingValue);
    uint8_t position = MCP41HVI2C_SetWiper(HVoutAdjAddr,HVoutAdjValue);
    if (debugDisplay) {
        Serial.print(position);
        Serial.print(", ");
        Serial.println(HVoutAdjValue);
    }
    return position;
  }
  else if (settingNum == 50){
    if (settingValue > -1) ignitionCtlState = boolean(settingValue);
    digitalWrite(ignitionCtlPin,ignitionCtlState);
    if (debugDisplay) {
        connectionString(ignitionCtlState);
        Serial.println(displayBuffer);
    }
    return ignitionCtlState;
  } 
  else if (settingNum >  50 && settingNum <= 66) {
    if (settingValue > -1) SPIpotTCONSettings[settingNum - 51] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVExtender_SetTerminals(settingNum - 51, SPIpotTCONSettings[settingNum - 51]);
    if (debugDisplay) {
        Serial.print(SPIpotTCONSettings[settingNum - 51]);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 67){
    if (settingValue > -1) PWM1Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM1Out);
        Serial.println(displayBuffer);
    }
    return PWM1Out;
  } 
  else if (settingNum == 68){
    if (settingValue > -1) PWM2Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM2Out);
        Serial.println(displayBuffer);
    }
    return PWM2Out;
  }  
  else if (settingNum == 69){
    if (settingValue > -1) PWM3Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM3Out);
        Serial.println(displayBuffer);
    }
    return PWM3Out;
  }  
  else if (settingNum == 70) {
    if (settingValue > -1) PWM4Out = boolean(settingValue);
    setTerminationSwitches();
    if (debugDisplay) {
        connectionString(PWM4Out);
        Serial.println(displayBuffer);
    }
    return PWM4Out;
  } 
  else if (settingNum == 71) {
    if (settingValue > -1) LIN1Switch = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(LIN1Switch);
        Serial.println(displayBuffer);
    }
    return LIN1Switch;
  }
  else if (settingNum == 72) {
    if (settingValue > -1) LIN2Switch = boolean(settingValue);
    setConfigSwitches();
    if (LIN2Switch) MCP41HVExtender_SetTerminals(15, 0);
    else MCP41HVExtender_SetTerminals(15, SPIpotTCONSettings[15]);
    if (debugDisplay) {
        connectionString(LIN2Switch);
        Serial.println(displayBuffer);
    }
    return LIN2Switch;
  }
  else if (settingNum == 73) {
    if (settingValue > -1) U1though8Enable = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U1though8Enable);
        Serial.println(displayBuffer);
    }
    return U1though8Enable;
  } 
  else if (settingNum == 74){
    if (settingValue > -1) U9though16Enable = boolean(settingValue);
    setConfigSwitches();
    if (debugDisplay) {
        connectionString(U9though16Enable);
        Serial.println(displayBuffer);
    }
    return U9though16Enable;
  } 
  else if (settingNum == 75){
    if (settingValue > -1) I2CpotWiperSettings[0] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(I2CpotAddr[0],I2CpotWiperSettings[0]);
    if (debugDisplay) {
        Serial.print(I2CpotWiperSettings[0]);
        Serial.print(", ");
        Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 76){
    if (settingValue > -1) I2CpotWiperSettings[1] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(I2CpotAddr[1],I2CpotWiperSettings[1]);
    if (debugDisplay) {
        Serial.print(I2CpotWiperSettings[1]);
        Serial.print(", ");
        Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 77){
    if (settingValue > -1) I2CpotWiperSettings[2] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(I2CpotAddr[2],I2CpotWiperSettings[2]);
    if (debugDisplay) {
        Serial.print(I2CpotWiperSettings[2]);
        Serial.print(", ");
        Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 78){
    if (settingValue > -1) I2CpotTCONSettings[0] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(I2CpotAddr[0],I2CpotTCONSettings[0]);
    if (debugDisplay) {
        Serial.print(I2CpotTCONSettings[0]);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 79){
    if (settingValue > -1) I2CpotTCONSettings[1] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(I2CpotAddr[1],I2CpotTCONSettings[1]);
    if (debugDisplay) {
        Serial.print(I2CpotTCONSettings[1]);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 80){
    if (settingValue > -1) I2CpotTCONSettings[2] = uint8_t(settingValue);
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(I2CpotAddr[2],I2CpotTCONSettings[2]);
    if (debugDisplay) {
        Serial.print(I2CpotTCONSettings[2]);
        Serial.print(", ");
        terminalString(terminalConnection);
        Serial.println(displayBuffer);
    }
    return terminalConnection;
  }

  else if (settingNum >= 81 && settingNum <= 84 ){
    if (settingValue > -1)  pwmFrequency[settingNum-81] = uint16_t(settingValue);
    analogWriteFrequency(PWMPins[settingNum-81], pwmFrequency[settingNum-81]);
    for (uint8_t i = 0; i<numPWMs; i++) analogWrite(PWMPins[i],pwmValue[i]);
    if (debugDisplay) {
        Serial.println(pwmFrequency[settingNum-81]);
    }
    return pwmFrequency[settingNum-81];
  }

  else return -1;
  
  
}



/*               End Define Settings                              */
/******************************************************************/






void setCompIdEEPROMdata () {
  char id[256];
  componentID.toCharArray(id,255);
  EEPROM.put(componentIDAddress,id);
  Serial.print("SAVED ");
  Serial.println(componentID);
}


void getCompIdEEPROMdata () {
  char id[256];
  EEPROM.get(componentIDAddress, id);
  componentID = String(id);
  Serial.print("LOADED ");
  Serial.println(componentID);
}



void displayJ1708(){
  showJ1708 = bool(commandString.toInt());
  //clear the RX buffer on start. 
  J1708.clear();
  J1708_index = 0;
  newJ1708Char = false;
  firstJ1708 = true;
}

void  adjustError() {
  Serial.println(F("INFO SS - Condition not met. Turn adjust mode on by typing AO, then select a setting with CS"));
}



/********************************************************************************************/
/*                       Begin Function Calls for Serial Commands                           */

void turnOnAdjustMode() {
  ADJUST_MODE_ON = 1;
  Serial.println(F("INFO AO - Turned Adjustment mode on. Type AF or click to turn off. Type SS,XXXX  or scroll knob and click to set settings."));
  Serial.print(F("INFO Current Setting for Adjustement is "));
  Serial.print(currentSetting);
  Serial.print(" - ");
  Serial.println(settingNames[currentSetting]);
  knob.write(setSetting(currentSetting,-1,DEBUG_OFF));
  setLimits(currentSetting);

}

void turnOffAdjustMode() {
  ADJUST_MODE_ON = 0;
  Serial.println(F("INFO AF - Turned Setting Adjustment mode off. Type AO to turn on. Scroll knob to select a setting."));
  knob.write(currentSetting);
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
}

void fastSetSetting(){
  int returnval;
  currentSetting = commandPrefix.toInt();
  if (currentSetting > 0 && currentSetting < numSettings){
    setLimits(currentSetting);
    if (commandString.length() > 0){ 
      long settingValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
      returnval = setSetting(currentSetting, settingValue,DEBUG_OFF);
    }
    else{
      returnval = setSetting(currentSetting, -1, DEBUG_OFF);
    }
    Serial.print("SET ");
    Serial.print(currentSetting);
    Serial.print(",");
    Serial.println(returnval);  
  }
  else Serial.println(F("ERROR in setting value."));
  
}

void changeSetting() {
  Serial.println(F("INFO CS - Change or Select Setting."));
  if (commandString.length() > 0) {
    currentSetting = constrain(commandString.toInt(), 0, numSettings);
    
  }
  //listSetting(currentSetting);
  if (ADJUST_MODE_ON){
    setLimits(currentSetting);
    knob.write(setSetting(currentSetting,-1,DEBUG_OFF));
  }
  else{
    if (knob.read() == currentSetting){
      Serial.print("INFO ");
      setSetting(currentSetting,-1,DEBUG_ON);
    }
    else knob.write(currentSetting); //automatic listSetting if knob changes
    knobLowLimit = 1;
    knobHighLimit = numSettings - 1;
  }
}

void listSettings(){
  Serial.println(F("INFO LS - List Settings. "));
  for (int i = 1; i < numSettings; i++) {
    Serial.print("INFO ");
    setSetting(i,-1,DEBUG_ON);
  }
}

void changeValue(){
  //Set value from Serial commands
  if (ADJUST_MODE_ON && currentSetting != 0) {
    Serial.println(F("INFO SS - Set Setting."));
    int adjustmentValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
    currentKnob = setSetting(currentSetting, adjustmentValue,DEBUG_ON);
    knob.write(currentKnob);
  }
  else
  {
    adjustError();
  }
}

//void saveEEPROM(){
//  //Save settings to EEPROM
//  Serial.println(F("INFO SA - Saving Settings to EEPROM."));
//  setDefaultEEPROMdata();
//}



/*                End Function Calls for Serial and Knob Commands                           */
/********************************************************************************************/





/********************************************************************************************/
/*                         Begin Function Calls for Knob Buttons                            */

OneButton button(buttonPin, true);

void longPressStart() {
  ignitionCtlState = !ignitionCtlState;
  commandPrefix = "50";
  if (ignitionCtlState) commandString = "1";
  else commandString = "0";
  fastSetSetting();
}

void myClickFunction() {
  turnOffAdjustMode();
  }
void myDoubleClickFunction() {}
void longPress() {} //Do nothing at this time.
void longPressStop() {} 
/*                         End Function Calls for Knob Buttons                            */
/********************************************************************************************/


