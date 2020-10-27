/*
   Smart Sensor Simulator 2

   Arduino Sketch to define the Smart Sensor Simulator 2 Functions
   Uses the Teensy 3.6
   This file needs to be included in the main program.

   Written By Dr. Jeremy S. Daily
   The University of Tulsa
   Department of Mechanical Engineering

   22 May 2017

   Released under the MIT License

   Copyright (c) 2017        Jeremy S. Daily

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   All libraried and packages are available by installing the following:
   Arduino v1.8 https://www.arduino.cc/en/Main/Software
   Teensyduino 1.35 https://www.pjrc.com/teensy/td_download.html

   Libraries not included in the Arduino and Teensyduino environment include:
   OneButton - https://github.com/mathertel/OneButton
   FlexCAN - https://github.com/collin80/FlexCAN_Library
   Thread - https://github.com/ivanseidel/ArduinoThread
   TeensyID - https://github.com/sstaub/TeensyID
*/

#define ENCODER_OPTIMIZE_INTERRUPTS

#define USE_SPI1
#include <mcp_can.h>
#include <SPI.h>
#include <Encoder.h>
#include "OneButton.h"
#include <i2c_t3.h>
#include <EEPROM.h>
#include "FlexCAN.h"
#include <TimeLib.h>
#include <TeensyID.h>
#include "FastCRC.h"
#include "Thread.h"
#include "ThreadController.h"
#include "base64.hpp"


FastCRC16 CRC16;

uint8_t status_buffer_1[64];
uint8_t status_buffer_2[64];
uint8_t status_buffer_3[64];
uint8_t status_buffer_4[64];
uint16_t buffer4_message_index;
uint16_t buffer4_index = 1;

#define STATUS_BUFFER_1_ADDR 64
#define STATUS_BUFFER_2_ADDR 128
#define STATUS_BUFFER_3_ADDR 192
#define STATUS_BUFFER_4_ADDR 256 //CAN Threads

long unsigned int rxId;
unsigned char len = 0;
unsigned char extended = 1;
unsigned char rxBuf[8];
char msgString[128];
unsigned char base64[64];

uint8_t can_buffer[62];
elapsedMillis can_send_time;



uint16_t configSwitchSettings;

//set up a display buffer
char displayBuffer[100];

IntervalTimer CANTimer;

MCP_CAN MCPCAN(CSCANPin);

//The Unique ID variable that comes from the chip
uint32_t uid[4];

String commandPrefix;
String commandString;

#define numSettings  96
uint8_t source_address = 0xFA;

int comp_id_index = 0;

/****************************************************************/
/*              Setup millisecond timers and intervals          */
//Declare a millisecond timer to execute the switching of the LEDs on a set time
elapsedMillis RXCAN0timer;
elapsedMillis RXCAN1orJ1708timer;
elapsedMillis analog_tx_timer;
elapsedMillis analogMillis;
elapsedMicros J1708RXtimer;
elapsedMicros microsecondsPerSecond;
elapsedMillis canComponentIDtimer;
elapsedMillis CAN_send_timer;

/****************************************************************/
/*                 Binary Control Variables                       */

boolean ADJUST_MODE_ON  = 0;
boolean SAFE_TO_ADJUST = 0;
boolean displayCAN0 = 0;
boolean displayCAN1 = 0;
boolean displayCAN2 = 0;
boolean TXCAN = true;
boolean enableSendComponentInfo = false;
boolean sendA21voltage = false;
boolean showJ1708 = false;
boolean firstJ1708 = true;
boolean send_voltage = false;
uint16_t currentSetting = 0;
bool newJ1708Char = false;

uint8_t J1708RXbuffer[256];
uint8_t J1708_index = 0;

uint32_t shortest_period = 10;

const int allFFs[8] = {255, 255, 255, 255, 255, 255, 255, 255};

void save_settings() {
  //Calculate CheckSums
  uint16_t checksum1 = CRC16.ccitt(status_buffer_1, 62);
  memcpy(&status_buffer_1[62], &checksum1, 2);
  uint16_t checksum2 = CRC16.ccitt(status_buffer_2, 62);
  memcpy(&status_buffer_2[62], &checksum2, 2);
  uint16_t checksum3 = CRC16.ccitt(status_buffer_3, 62);
  memcpy(&status_buffer_3[62], &checksum3, 2);

  EEPROM.put(STATUS_BUFFER_1_ADDR, status_buffer_1);
  EEPROM.put(STATUS_BUFFER_2_ADDR, status_buffer_2);
  EEPROM.put(STATUS_BUFFER_3_ADDR, status_buffer_3);

  Serial.println("Saved Settings.");
}

bool load_settings() {
  //Calculate CheckSums

  EEPROM.get(STATUS_BUFFER_1_ADDR, status_buffer_1);
  EEPROM.get(STATUS_BUFFER_2_ADDR, status_buffer_2);
  EEPROM.get(STATUS_BUFFER_3_ADDR, status_buffer_3);

  bool checksumOK = true;
  uint16_t checksum1 = CRC16.ccitt(status_buffer_1, 62);
  if (status_buffer_1[62] != (checksum1 & 0xFF00) >> 8) checksumOK = false;
  if (status_buffer_1[63] != (checksum1 & 0x00FF) >> 0) checksumOK = false;

  uint16_t checksum2 = CRC16.ccitt(status_buffer_2, 62);
  if (status_buffer_2[62] != (checksum2 & 0xFF00) >> 8) checksumOK = false;
  if (status_buffer_2[63] != (checksum2 & 0x00FF) >> 0) checksumOK = false;

  uint16_t checksum3 = CRC16.ccitt(status_buffer_3, 62);
  if (status_buffer_3[62] != (checksum3 & 0xFF00) >> 8) checksumOK = false;
  if (status_buffer_3[63] != (checksum3 & 0x00FF) >> 0) checksumOK = false;

  if (checksumOK) {
    Serial.println("Successfully loaded and verified settings from EEPROM.");
  }
  else {
    Serial.println("The checksum failed when loading settings from EEPROM.");
  }
  return checksumOK;
}


/****************************************************************/
/*                    Quadrature Knob Setup                     */

Encoder knob(encoderAPin, encoderBPin);
int32_t currentKnob = 0;

int knobLowLimit = 0;
int knobHighLimit = 255;
int knobJump = 1;


void displayVoltage() {
  analogMillis = 0;
  if (commandString.toInt() > 0) {
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
#define num_default_messages  21
String default_messages[num_default_messages] = {
  F("DDEC MCM 01;                   1;1;0;2;  10;   0;0;1; 8FF0001;8;01;02;03;04;05;06;07;08\n"), //DDEC 13 MCM message; CAN1
  F("DDEC TCM 01;                   2;1;0;2;  10;   0;0;1; CF00203;8; 9; a; b; c; d; e; f; 0\n"), //DDEC13 Transmission controler message; CAN1
  F("DDEC TCM 02;                   3;1;0;2;  10;   0;0;1; 8FF0303;8; 0; 0; 0; 0; 0; 0; 0; 0"), //DDEC 13 TCM message; CAN1
  F("DDEC TCM 03;                   4;1;0;2; 100;   0;0;1;18F00503;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Transmission on DDEC 13
  F("HRW from Brake Controller;     5;1;0;0;  20;   0;0;1; CFE6E0B;8; 0; 0; 0; 0; 0; 0; 0; 0"), //High Resolution wheel speed message from SA=11 (brake controller)
  F("EBC1 from Cab Controller;      6;1;0;0; 100;   0;0;1;18F00131;8; 0; 0; 0; 0; 0; 0; 0; 0"), // Electronic Brake Controller from SA=49
  F("EBC1 from Brake Controller;    7;1;0;0; 100;   0;0;1;18F0010B;8; 0; 0; 0; 0; 0; 0; 0; 0"), // Electronic Brake Controller from SA=11
  F("CCVS1 from Instrument Cluster; 8;1;0;0; 100;   0;0;1;18FEF117;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise Control/Vehicle Speed from SA=23
  F("CCVS1 from Cab Display 1;      9;1;0;0; 100;   0;0;1;18FEF128;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise Control/Vehicle Speed from SA=40
  F("CCVS1 from Body Controller;   10;1;0;0; 100;   0;0;1;18FEF121;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise Control/Vehicle Speed from SA=33
  F("CCVS1 from Cab Controller;    11;1;0;0; 100;   0;0;1;18FEF131;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise Control/Vehicle Speed from SA=49
  F("CM1 from Instrument Cluster;  12;1;0;0; 100;   0;0;1;18E00017;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cab Message 1 from SA=23
  F("CM1 from Climate Control 1;   13;1;0;0; 100;   0;0;1;18E00019;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cab Message 1 from SA=25
  F("CM1 from Body Controller;     14;1;0;0; 100;   0;0;1;18E00021;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cab Message 1 from SA=33
  F("CM1 from Cab Display;         15;1;0;0; 100;   0;0;1;18E00028;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cab Message 1 from SA=40
  F("CM1 from Cab Controller;      16;1;0;0; 100;   0;0;1;18E00031;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cab Message 1 from SA=49
  F("PTO from Instrument Cluster;  17;1;0;0; 100;   0;0;1;18FEF017;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise and PTO setup from SA=23
  F("PTO from Body Controller;     18;1;0;0; 100;   0;0;1;18FEF021;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise and PTO setup from SA=33
  F("PTO from Cab Display;         19;1;0;0; 100;   0;0;1;18FEF028;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise and PTO setup from SA=40
  F("PTO from Cab Controller;      20;1;0;0; 100;   0;0;1;18FEF031;8; 0; 0; 0; 0; 0; 0; 0; 0"), //Cruise and PTO setup from SA=49
  F("AMB from Body Controller;     21;1;0;0;1000;   0;0;1;18FEF521;8; 0; 0; 0; 0; 0; 0; 0; 0")  //Ambient Conditions
};

//Set up the CAN data structures
static CAN_message_t rxmsg;
static CAN_message_t txmsg;
static CAN_message_t temp_txmsg;

//set up a counter for each received message
uint32_t RXCount0 = 0;
uint32_t RXCount1 = 0;
uint32_t RXCount2 = 0;
uint16_t J1708RXCount = 0;

//set up a counter for each received message
uint32_t TXCount0 = 0;
uint32_t TXCount1 = 0;
uint32_t TXCount2 = 0;
uint16_t J1708TXCount = 0;


int CANWaitTimeout = 200;
uint32_t BAUDRATE0 = 250000;
uint32_t BAUDRATE1 = 250000;
uint32_t BAUDRATE_MCP = 250000;
const uint8_t baudRateListLength = 15;
const uint32_t baudRateList[baudRateListLength] = {250000, 500000, 666666, 125000, 1000000, 5000, 10000, 20000, 31520, 333333, 40000, 50000, 80000, 100000, 200000};
uint8_t baudRateIndex0 = 0;
uint8_t baudRateIndex1 = 0;
uint8_t baudRateIndex2 = 0;

void send_can_messages(CAN_message_t txmsg, uint8_t can_channel)
{
  if (can_channel == 0) {
    Can0.write(txmsg);
    TXCount0++;
    memcpy(&status_buffer_2[CAN0_TX_COUNT_LOC], &TXCount0, 4);
  }
  else if (can_channel == 1) {
    Can1.write(txmsg);
    TXCount1++;
    memcpy(&status_buffer_2[CAN1_TX_COUNT_LOC], &TXCount1, 4);
  }
  else if (can_channel == 2) {
    MCPCAN.sendMsgBuf(txmsg.id, txmsg.ext, txmsg.len, txmsg.buf);
    TXCount2++;
    memcpy(&status_buffer_2[CAN2_TX_COUNT_LOC], &TXCount2, 4);
  }
}

// Create a thread controller class
ThreadController can_thread_controller = ThreadController();

void runCANthreads() {
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
    uint32_t tx_period = 0;
    uint8_t channel = 0;

    CAN_message_t txmsg;

    uint8_t num_messages = 1;
    uint8_t message_index = 0;
    uint8_t len_list[256] = {};
    uint8_t message_list[256][8] = {};
    uint32_t id_list[256] = {};


    bool shouldRun(unsigned long time) {
      if (stop_after_count > 0) {
        if (transmit_number >= stop_after_count) enabled = false;
      }
      return Thread::shouldRun(time);
    }

    void run() {
      //Set the CAN message data to the next one in the list.
      txmsg.id = id_list[message_index];
      txmsg.len = len_list[message_index];
      memcpy(txmsg.buf, message_list[message_index], 8);

      //Write the data to the CAN bus.
      if (ok_to_send && ignitionCtlState) {
        send_can_messages(txmsg, channel);
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
      if (cycle_count * interval >= loop_cycles) {
        cycle_count = 0;
        ok_to_send = true;
      }
      return Thread::run();
    }
};

//Setup a container to keep track of the individual CAN threads
CanThread* can_messages[MAX_THREADS] = {};

void set_shortest_period() {
  if (commandString.length() > 0) {
    shortest_period = commandString.toInt();
    Serial.printf("SET Shortest CAN Broadcast Period to %lu milliseconds.\n",shortest_period);
  }
  //else
  // Serial.printf("INFO Shortest CAN Broadcast Period is %lu milliseconds.\n",shortest_period);
}

void getThreadName() {
  int index = commandString.toInt();
  Serial.printf("NAME of CAN Thread %d: ",index);
  Serial.println(can_messages[index]->ThreadName);
}

void getAllThreadNames() {
  int threadSize =  can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++) {
    Serial.printf("NAME of CAN Thread %d: ",i);
    Serial.println(can_messages[i]->ThreadName);
  }
}

void getThreadSize() {
  int threadSize =  can_thread_controller.size(false);
  //Serial.printf("INFO Size of CAN Thread = %d\n",threadSize);
}

int setupPeriodicCANMessage() {
  CANTimer.end();
  CanThread* can_message;
  int16_t index;
  int16_t sub_index;
  uint8_t channel;
  uint32_t tx_period;
  uint32_t tx_delay;
  uint8_t num_messages = 1;
  uint32_t stop_after_count;
  Serial.println("***************Entered setupPeriodicCANMessage*******************");
  Serial.println(commandString);

  uint16_t threadSize =  can_thread_controller.size(false);
  char commandBytes[128];
  commandString.toCharArray(commandBytes, sizeof(commandBytes));
  char delimiter[] = ";";
  char* commandValues;

  commandValues = strtok(commandBytes, delimiter);
  String threadName = commandValues;

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    index = constrain(atoi(commandValues), 0, threadSize + 1);
  }
  else {
    Serial.println(("ERROR SM command is missing arguments."));
    return -1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    num_messages = constrain(atoi(commandValues), 1, 255);
  }
  else {
    Serial.println(("ERROR SM command not able to determine the number of sub messages."));
    return -2;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    //constrain the sub_index to be one larger than the
    sub_index = constrain(atoi(commandValues), 0, num_messages - 1);
  }
  else {
    Serial.println(("ERROR SM command missing sub_index."));
    return -3;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    channel = constrain(atoi(commandValues), 0, 2);
  }
  else {
    Serial.println(("ERROR SM command not able to set CAN Channel."));
    return -4;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    tx_period = constrain(strtoul(commandValues, NULL, 10), shortest_period, 0xFFFFFFFF);
  }
  else {
    Serial.println(("ERROR SM command not able to set period information."));
    return -5;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    tx_delay = strtoul(commandValues, NULL, 10);
  }
  else {
    Serial.println(("ERROR SM command not able to set delay information."));
    return -6;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    stop_after_count = strtoul(commandValues, NULL, 10);
  }
  else {
    Serial.println(("ERROR SM command not able to set the total number count."));
    return -7;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    temp_txmsg.ext = constrain(atoi(commandValues), 0, 1);
  }
  else {
    Serial.println(("ERROR SM command not able to set extended ID flag."));
    temp_txmsg.ext = 1;
  }

  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    temp_txmsg.id = strtoul(commandValues, NULL, 16);
  }
  else {
    Serial.println(("WARNING SM command not able to set CAN ID information."));
    temp_txmsg.id = 0x3FFFFFFF ;
  }
  
  commandValues = strtok(NULL, delimiter);
  if (commandValues != NULL) {
    temp_txmsg.len = constrain(atoi(commandValues), 0, 8);
  }
  else {
    Serial.println(("WARNING SM command not able to set CAN data length code."));
    temp_txmsg.len = 8;
  }

  for (int i = 0; i < temp_txmsg.len; i++) {
    commandValues = strtok(NULL, delimiter);
    if (commandValues != NULL) {
      temp_txmsg.buf[i] = constrain(strtol(commandValues, NULL, 16), 0, 255);
    }
    else {
      temp_txmsg.buf[i] = 0xFF;
      Serial.printf("WARNING SM command not able to set CAN data byte in position %d.\n",i);
    }
  }
  char threadNameChars[25]; //25 (24 chars + /00) is the size of the message frame space for the thread name
  threadName.toCharArray(threadNameChars, sizeof(threadNameChars));

  if (index >= threadSize) { //Create a new entry
    index = threadSize;
    Serial.printf("THREAD %lu, %s (NEW)\n",index,threadNameChars);
    CanThread* can_message = new CanThread();
    can_thread_controller.add(can_message);
    can_messages[index] = can_message;
    can_messages[index]->enabled = false;
  }
  else {
    Serial.printf("THREAD %lu, %s (EXISTING)\n",index,threadNameChars);
  }

  Serial.printf("SET CAN name=%s \n index=%d\n num_messages=%d\n sub_index=%d\n channel=%d\n tx_period=%d\n tx_delay=%d\n stop_after_count=%d\n temp_txmsg.ext=%d\n ID=%08X\n DLC=%d\n DATA=[",
                threadNameChars, index, num_messages, sub_index, channel, tx_period, tx_delay, stop_after_count, temp_txmsg.ext, temp_txmsg.id, temp_txmsg.len);
  for (int i = 0; i < temp_txmsg.len - 1; i++) {
    Serial.printf("%02X, ",temp_txmsg.buf[i]);
  }
  Serial.printf("%02X]\n",temp_txmsg.buf[temp_txmsg.len-1]);

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
  can_messages[index]->tx_period = tx_period;
  can_messages[index]->setInterval(tx_period);
  can_messages[index]->loop_cycles =  tx_delay ;
  can_messages[index]->ThreadName = threadName;

  CANTimer.begin(runCANthreads, 500);
  return index;
}

void stopCAN() {
  int threadSize =  can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++) {
    can_messages[i]->enabled = false;
  }
  Serial.println(("INFO Stopped all CAN transmission."));
}

void clearCAN() {
  int threadSize =  can_thread_controller.size(false);
  for (int i = 1; i < threadSize; i++) { //Leave 0 in place because it it component ID
    delete can_messages[i] ;
  }
  can_thread_controller.clear();
  Serial.println(("INFO Cleared the CAN transmission thread. All messages must be reloaded."));
}

void goCAN() {
  int threadSize =  can_thread_controller.size(false);
  for (int i = 0; i < threadSize; i++) {
    can_messages[i]->enabled = true;
    can_messages[i]->transmit_number =0;
    can_messages[i]->message_index = 0;
    can_messages[i]->cycle_count = 0;
  }
  Serial.println(("INFO Started all CAN transmissions."));
}

void startCAN () {
  int threadSize =  can_thread_controller.size(false);
  if (threadSize > 0) {
    char commandBytes[40];
    commandString.toCharArray(commandBytes, 100);
    Serial.print("Command Bytes:   ");
    Serial.println(commandBytes);
    char delimiter[] = ";";
    char* commandValues;
    // commandValues = strtok(commandBytes, delimiter);
    commandValues = strtok(NULL, delimiter);
    Serial.print("Command Values:   ");
    Serial.println(commandValues);

    commandValues = strtok(commandBytes, delimiter);
    Serial.print("Command Values:   ");
    Serial.println(commandValues);

    int index = constrain(atoi(commandValues), 0, threadSize - 1);
    Serial.print("index: ");
    Serial.println(index);
    commandValues = strtok(NULL, delimiter);
    int setting = atoi(commandValues);
    Serial.print("setting: ");
    Serial.println(setting);
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
    //Serial.println("ERROR No CAN Messages Setup to turn on.");
  }
}

void setupComponentInfo()
{
  char byteEntry[4];
  uint8_t old_shortest_period = shortest_period;
  shortest_period = 1;
  uint16_t id_length = constrain(componentID.length(), 0, 7 * 256 - 1);
  char id[7 * 256];
  componentID.toCharArray(id, id_length + 1);
  uint8_t num_frames = id_length / 7;
  if (id_length % 7 > 0)
    num_frames++;
  char bytes_to_send[4];
  sprintf(bytes_to_send, "%02X", id_length);
  char frames_to_send[3];
  sprintf(frames_to_send, "%02X", num_frames);
  commandString = "CI from SSS2;0;1;0;0;1;0;1;1;18ECFF";
  sprintf(byteEntry, "%02X;", source_address);
  commandString += byteEntry;
  commandString += "8;20;";
  commandString += bytes_to_send;
  commandString += ";0;";
  commandString += frames_to_send;
  commandString += ";FF;EB;FE;00";

  setupPeriodicCANMessage();

  for (int i = 0; i < num_frames; i++)
  {
    commandString = "CI from SSS2;0;";
    sprintf(byteEntry, "%d;", num_frames + 1);
    commandString += byteEntry;
    sprintf(byteEntry, "%d;", i + 1);
    commandString += byteEntry;
    commandString += "0;1;0;";
    sprintf(byteEntry, "%d;", num_frames + 1);
    commandString += byteEntry;
    commandString += "1;18EBFF";
    sprintf(byteEntry, "%02X;", source_address);
    commandString += byteEntry;
    commandString += "8;";
    sprintf(byteEntry, "%02X;", i + 1);
    commandString += byteEntry;
    for (int j = 7 * i; j < 7 * i + 7; j++)
    {
      if (j < id_length)
        sprintf(byteEntry, "%02X;", id[j]);
      else
        sprintf(byteEntry, "%02X;", 0xFF);
      commandString += byteEntry;
    }
    comp_id_index = setupPeriodicCANMessage();
  }
  shortest_period = old_shortest_period;
}


void reloadCAN()
{
  setupComponentInfo();

  for (int i = 0; i < num_default_messages; i++)
  {
    commandString = default_messages[i];
    setupPeriodicCANMessage();
    delayMicroseconds(800);
  }
  commandString = "1";
  goCAN();
}

/*
 * *************************** END CAN Logic ********************************
*/

void connectionString(boolean switchState) {
  //memset(displayBuffer,0,sizeof(displayBuffer));
  //Serial.print(switchState);
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

  uint8_t terminationSettings =  uint8_t( CAN0term1 | CAN1term1 << 1 | CAN2term1 << 2 |  PWM4Out_28 << 3 |
                                          CAN1out << 4 | CAN1out << 5 | PWM5Out << 6 | PWM6Out << 7);
  digitalWrite(CSconfigAPin, LOW);
  SPI.transfer(terminationSettings);
  digitalWrite(CSconfigAPin, HIGH);
  status_buffer_1[TERMINATION_SETTINGS_LOC] = terminationSettings;
  return terminationSettings;
}

void getTerminationSwitches(uint8_t terminationSettings) {
  CAN0term1 = (terminationSettings & 0b00000001) >> 0;
  CAN1term1 = (terminationSettings & 0b00000010) >> 1;
  CAN2term1 = (terminationSettings & 0b00000100) >> 2;
  PWM4Out_28 = (terminationSettings & 0b00001000) >> 3;
  CAN1out   = (terminationSettings & 0b00010000) >> 4;
  PWM5Out   = (terminationSettings & 0b01000000) >> 6;
  PWM6Out   = (terminationSettings & 0b10000000) >> 7;
}


uint8_t setPWMSwitches() {
  uint8_t PWMSettings =  uint8_t( CAN0term | CAN1term << 1 | CAN2term << 2 |  LINmaster << 3 |
                                  PWM1Out << 4 | PWM2Out << 5 | PWM3Out << 6 | PWM4Out << 7);
  digitalWrite(CSconfigBPin, LOW);
  SPI.transfer(PWMSettings);
  digitalWrite(CSconfigBPin, HIGH);
  status_buffer_1[PWM_SETTINGS_LOC] = PWMSettings;
  return PWMSettings;
}

void getPWMSwitches(uint8_t PWMSettings) {
  //Set the termination Switches for U21
  CAN0term   = (PWMSettings & 0b00000001) >> 0;
  CAN1term   = (PWMSettings & 0b00000010) >> 1;
  CAN2term   = (PWMSettings & 0b00000100) >> 2;
  LINmaster  = (PWMSettings & 0b00001000) >> 3;
  PWM1Out    = (PWMSettings & 0b00010000) >> 4;
  PWM2Out    = (PWMSettings & 0b00100000) >> 5;
  PWM3Out    = (PWMSettings & 0b01000000) >> 6;
  PWM4Out    = (PWMSettings & 0b10000000) >> 7;
}

void setConfigSwitches() {
  //Set the termination Switches of U21 on Rev 3 based on the boolean values of the variables representing the GPIO pins of U21

  uint8_t configSwitchSettingsA =
    LIN1Switch | LIN2Switch  << 1 | P10or19Switch << 2 |  P15or18Switch << 3 | U1though8Enable << 4 | U9though16Enable << 5 |
    CAN1Switch << 6 | CAN2Switch << 7;
  Wire.beginTransmission(configExpanderAddr);
  Wire.write(uint8_t(MCP23017_GPIOA)); // sends instruction byte
  Wire.write(uint8_t(configSwitchSettingsA));     // sends potentiometer value byte
  byte ret_val = Wire.endTransmission();


  uint8_t configSwitchSettingsB =
    U1U2P0ASwitch |  U3U4P0ASwitch  << 1 |  U5U6P0ASwitch  << 2 | U7U8P0ASwitch << 3 |
    U9U10P0ASwitch << 4 | U11U12P0ASwitch << 5 | U13U14P0ASwitch << 6 | U15U16P0ASwitch << 7;
  Wire.beginTransmission(configExpanderAddr);
  Wire.write(uint8_t(MCP23017_GPIOB)); // sends instruction byte
  Wire.write(uint8_t(configSwitchSettingsB));     // sends potentiometer value byte
  Wire.endTransmission();

  uint16_t configSwitchSettings = (configSwitchSettingsB << 8) + configSwitchSettingsA;
  memcpy(&status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], &configSwitchSettings, 2);
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
  U1U2P0ASwitch     = (configSwitchSettings & 0b0000000100000000) >> 8;
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
  if (pin < 8) {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOA)); // sends instruction byte
    Wire.write(uint8_t(~(1 << pin)));     // sends potentiometer value byte
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOB)); // sends instruction byte
    Wire.write(uint8_t(~(1 << (pin - 8))));     // sends potentiometer value byte
    Wire.endTransmission();
  }

  delay(1);
  SPI.transfer(0x40); //Write to TCON Register
  SPI.transfer(TCON_Value + 8);
  SPI.transfer(0x4C); //Read Command
  uint8_t result = SPI.transfer(0xFF); //Read Terminal Connection (TCON) Register
  if (pin < 8) {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOA)); // sends instruction byte
    Wire.write(uint8_t(0xFF));     // sends potentiometer value byte
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOB)); // sends instruction byte
    Wire.write(uint8_t(0xFF));     // sends potentiometer value byte
    Wire.endTransmission();
  }
  status_buffer_2[pin + 1] = result & 0x07;
  return result & 0x07;
}

uint8_t MCP41HVExtender_SetWiper(uint8_t pin, uint8_t potValue)
{
  if (pin < 8) {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOA)); // sends instruction byte
    Wire.write(uint8_t(~(1 << pin)));     // sends potentiometer value byte
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOB)); // sends instruction byte
    Wire.write(uint8_t(~(1 << (pin - 8))));     // sends potentiometer value byte
    Wire.endTransmission();
  }

  delay(1);
  SPI.transfer(0x00); //Write to wiper Register
  SPI.transfer(potValue);
  SPI.transfer(0x0C); //Read command
  uint8_t result = SPI.transfer(0xFF); //Read Wiper Register

  if (pin < 8) {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOA)); // sends instruction byte
    Wire.write(uint8_t(0xFF));     // sends potentiometer value byte
    Wire.endTransmission();
  }
  else {
    Wire.beginTransmission(potExpanderAddr);
    Wire.write(uint8_t(MCP23017_GPIOB)); // sends instruction byte
    Wire.write(uint8_t(0xFF));     // sends potentiometer value byte
    Wire.endTransmission();
  }
  status_buffer_1[pin + 1] = result;
  return result;
}

uint8_t MCP41HVI2C_SetTerminals(uint8_t addr, uint8_t TCON_Value) {
  Wire.beginTransmission(addr);
  Wire.write(byte(0x40));            // sends instruction byte
  Wire.write(0xF8 | TCON_Value);     // sends potentiometer value byte
  Wire.endTransmission();

  Wire.requestFrom(addr, 2);    // request bytes from slave device
  uint8_t result = Wire.read(); //Read Wiper Register
  result = Wire.read() & 0x07; //Read Wiper Register
  if      (addr == U34_I2C_ADDR) status_buffer_2[U34_TCON_LOC] = result;
  else if (addr == U36_I2C_ADDR) status_buffer_2[U36_TCON_LOC] = result;
  else if (addr == U37_I2C_ADDR) status_buffer_2[U37_TCON_LOC] = result;
  return result;
}

uint8_t MCP41HVI2C_SetWiper(uint8_t addr, uint8_t potValue)
{
  Wire.beginTransmission(addr);
  Wire.write(byte(0x00));            // sends instruction byte
  Wire.write(potValue);             // sends potentiometer value byte
  Wire.endTransmission();

  Wire.requestFrom(addr, 2);   // request 2 bytes from slave device #8
  uint8_t result = Wire.read(); //Read Wiper Register, first byte is all 0
  result = Wire.read(); //Read Wiper Register
  if      (addr == U34_I2C_ADDR) status_buffer_1[U34_WIPER_LOC] = result;
  else if (addr == U36_I2C_ADDR) status_buffer_1[U36_WIPER_LOC] = result;
  else if (addr == U37_I2C_ADDR) status_buffer_1[U37_WIPER_LOC] = result;
  else if (addr == HVoutAdjAddr) status_buffer_1[HVADJOUT_LOC]  = result;
  return result;
}

/*   End Function Calls for Digital Potentiometer               */
/****************************************************************/

/****************************************************************/
/*   Begin Function Calls for DAC                               */

void initializeDACs(uint8_t address) {
  Serial.print("Setting DAC Internal Reference register with address of 0x");
  Serial.println(address, HEX);
  ;
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
  //Serial.print("Internal Reference Register: 0x");
  //Serial.println(lowC, HEX);         // print the character

  //Serial.println("Setting LDAC register.");
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
  //  Serial.print("LDAC Register: ");
  //  Serial.println(lowC, HEX);        // print the character
  //
  //  Serial.println("Setting DAC Power Down register to On.");
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
  //  Serial.print("Power Register: ");
  //  Serial.println(lowC, BIN);        // print the character
  //
  //  Serial.print(("Done with DAC at address 0x"));
  //  Serial.println(address, HEX);
}

uint16_t setDAC(uint16_t setting, uint8_t DACchannel, uint8_t address) {
  setting = setting * 0.811577603149; //Conversion constant to get close to millivolts
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
  uint16_t result = ((highC << 4) + (lowC >> 4)) * 1.232168059; //Show millivolts
  memcpy(&status_buffer_1[DACchannel * 2 + 17], &result, 2);
  return result;
}

/*   End Function Calls for DAC                              */
/****************************************************************/

/********************************************************************/
/*               Begin Define Settings                              */
/*
   To add another setting, update the numSettings, populate the settingNames and settingPins array and add
   new knob limits. The knob limits are used for value checking. These are for numerical Settings.
*/


char settingNames[numSettings][50] = {
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

  "PWM 1 Frequency",
  "PWM 2 Frequency",
  "PWM 3 Frequency",
  "PWM 4 Frequency",
  "PWM 5 and 6 Frequency",
  "PWM 4 Connect",
  "PWM 5 Value",
  "PWM 6 Value",
  "PWM 5 Connect",
  "PWM 6 Connect",
  "CAN1 Connect",
  "CAN2 Connect",

  "CAN0 Resistor 2",
  "CAN1 Resistor 2",
  "CAN2 Resistor 2",
};

char settingPins[numSettings][50] = {
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
  "Port 13 (J24-13)",
  "Port 14 (J24-14)",
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
  "Port 17 (J18-1) and 28 (J18-12)",

  "(J24-10)",
  "(J24-15)",
  "(J24-17 & J24-18)",
  "(J18-15 and J18-16)",
  "R44",
  "R45",
  "R46",
  "R59",

  "Port 27 (J18-10)",
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
  "Port 17 (J18-1)",
  "Ports 1 and 2 (J24-1 and 2)",
  "Port 28 (J18-12)",
  "Port 2  (J24-2)",
  "Port 1  (J24-1)",
  "Port 2 (J24-2)",
  "Port 1 (J24-1)",
  "Ports 3 and 4 (J24-3 and 4)",
  "(J18-15 and J18-16)",
  "R3",
  "R4",
  "R16",
};


void setLimits(uint8_t settingNum) {
  if (settingNum > 0 && settingNum <= 16) {
    knobLowLimit = 0;
    knobHighLimit = 255;
    knobJump = 1;
  }
  else if (settingNum > 16 && settingNum <= 24) {
    knobLowLimit = 0;
    knobHighLimit = 5005;
    knobJump = 20;
  }
  else if (settingNum > 24 && settingNum <= 32) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum > 32 && settingNum <= 36) {
    knobLowLimit = 0;
    knobHighLimit = 5005;
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
  else if (settingNum >= 81 && settingNum <= 82) {
    knobLowLimit = 245;
    knobHighLimit = 65535;
    knobJump = 1;
  }
  else if (settingNum >= 83 && settingNum <= 85) {
    knobLowLimit = 0;
    knobHighLimit = 65535;
    knobJump = 1;
  }
  else if (settingNum == 86) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else if (settingNum == 87 || settingNum == 88) {
    knobLowLimit = 0;
    knobHighLimit = 5000;
    knobJump = 1;
  }
  else if (settingNum >= 89 || settingNum <= 95) {
    knobLowLimit = 0;
    knobHighLimit = 1;
    knobJump = 1;
  }
  else {
    knobLowLimit = 1;
    knobHighLimit = numSettings - 1;
    knobJump = 1;
  }

  // Serial.print("Low Limit: ");
  // Serial.println(knobLowLimit);
  // Serial.print("High Limit: ");
  // Serial.println(knobHighLimit);
}

int16_t setSetting(uint8_t settingNum, int settingValue, bool debugDisplay) {


  if (debugDisplay) {
    Serial.print(settingNum);
    Serial.print(", ");
    Serial.print(settingNames[settingNum]);
    Serial.print(", ");
    Serial.print(settingPins[settingNum]);
    Serial.print(" = ");
  }
  if (settingNum >= 1 && settingNum <= 16) {
    if (settingValue > -1) SPIpotWiperSettings[settingNum - 1] = settingValue;
    else                   SPIpotWiperSettings[settingNum - 1] = status_buffer_1[settingNum];
    uint8_t w_position = MCP41HVExtender_SetWiper(settingNum - 1,
                         SPIpotWiperSettings[settingNum - 1]);
    if (debugDisplay) {
      Serial.print(SPIpotWiperSettings[settingNum - 1]);
      Serial.print(", ");
      Serial.println(w_position);
    }
    return w_position;
  }
  else if (settingNum > 16 && settingNum <= 24) {
    uint8_t DACchannel = settingNum - 17;
    if (settingValue > -1) DAC2value[DACchannel] = settingValue;
    else {
      uint16_t data_from_memory;
      memcpy(&data_from_memory, &status_buffer_1[DACchannel * 2 + 17], 2);
      DAC2value[DACchannel] = data_from_memory;
    }
    setDAC(DAC2value[DACchannel], DACchannel, Vout2address);
    if (debugDisplay) {
      Serial.println(DAC2value[DACchannel]);
    }
    return DAC2value[DACchannel];
  }
  else if (settingNum == 25) {
    if (settingValue > -1) U1U2P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U1U2P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U1U2P0ASwitch;
  }
  else if (settingNum == 26) {
    if (settingValue > -1) U3U4P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U3U4P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U3U4P0ASwitch;
  }
  else if (settingNum == 27) {
    if (settingValue > -1) U5U6P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U5U6P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U5U6P0ASwitch;
  }
  else if (settingNum == 28) {
    if (settingValue > -1) U7U8P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U7U8P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U7U8P0ASwitch;
  }
  else if (settingNum == 29) {
    if (settingValue > -1) U9U10P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U9U10P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U9U10P0ASwitch;
  }
  else if (settingNum == 30) {
    if (settingValue > -1) U11U12P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U11U12P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U11U12P0ASwitch;
  }
  else if (settingNum == 31) {
    if (settingValue > -1) U13U14P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U13U14P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U13U14P0ASwitch;
  }
  else if (settingNum == 32) {
    if (settingValue > -1) U15U16P0ASwitch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U15U16P0ASwitch);
      Serial.println(displayBuffer);
    }
    return U15U16P0ASwitch;
  }
  else if (settingNum >= 33 && settingNum <= 36 ) {
    if (settingValue > -1) pwmValue[settingNum - 33] = uint16_t(constrain(settingValue, 0, 4096));
    else {
      memcpy(&pwmValue[settingNum - 33], &status_buffer_1[(settingNum - 33) * 2 + 35], 2);
    }
    for (uint8_t i = 0; i < numPWMs; i++) analogWrite(PWMPins[i], pwmValue[i]);
    uint16_t result = pwmValue[settingNum - 33];
    if (debugDisplay) {
      Serial.println(result);
    }
    memcpy(&status_buffer_1[(settingNum - 33) * 2 + 35], &result, 2);
    return result;
  }
  else if (settingNum == 37) {
    if (settingValue > -1) P10or19Switch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(P10or19Switch);
      Serial.println(displayBuffer);
    }
    return P10or19Switch;
  }
  else if (settingNum == 38) {
    if (settingValue > -1) P15or18Switch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(P15or18Switch);
      Serial.println(displayBuffer);
    }
    return P15or18Switch;
  }
  else if (settingNum == 39) {
    if (settingValue > -1) CAN1Switch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(CAN1Switch);
      Serial.println(displayBuffer);
    }
    return CAN1Switch;
  }
  else if (settingNum == 40 || settingNum == 92 ) {
    if (settingValue > -1) CAN2Switch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(CAN2Switch);
      Serial.println(displayBuffer);
    }
    return CAN2Switch;
  }
  else if (settingNum == 41) {
    if (settingValue > -1) CAN0term = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(CAN0term);
      Serial.println(displayBuffer);
    }
    return CAN0term;
  }
  else if (settingNum == 42) {
    if (settingValue > -1) CAN1term = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(CAN1term);
      Serial.println(displayBuffer);
    }
    return CAN1term;
  }
  else if (settingNum == 43) {
    if (settingValue > -1) CAN2term = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(CAN2term);
      Serial.println(displayBuffer);
    }
    return CAN2term;
  }
  else if (settingNum == 44) {
    if (settingValue > -1) LINmaster = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(LINmaster);
      Serial.println(displayBuffer);
    }
    return LINmaster;
  }
  else if (settingNum == 45 || settingNum == 69)  {
    if (settingNum == 45) {
      if (settingValue > -1) IH1State = boolean(settingValue);
      else IH1State = boolean(status_buffer_1[HBRIDGE_LOC] & TWELVE_OUT_1_MASK);
    }
    else if (settingNum == 69) {
      if (settingValue > -1) IH1State = !boolean(settingValue);
      else IH1State = !boolean(status_buffer_1[HBRIDGE_LOC] & TWELVE_OUT_1_MASK);
    }
    if (IH1State) {
      PWM3Out = false;
      status_buffer_1[HBRIDGE_LOC] |= TWELVE_OUT_1_MASK;
    }
    else {
      status_buffer_1[HBRIDGE_LOC] &= ~TWELVE_OUT_1_MASK;
      PWM3Out = true;
    }
    setPWMSwitches();
    delay(1);
    digitalWrite(IH1Pin, IH1State);
    if (debugDisplay) {
      connectionString(IH1State);
      Serial.println(displayBuffer);
    }
    return IH1State;
  }
  else if (settingNum == 46) {
    if (settingValue > -1) IH2State = boolean(settingValue);
    else IH2State = boolean(status_buffer_1[HBRIDGE_LOC] & TWELVE_OUT_2_MASK);
    if (IH2State) {
      MCP41HVExtender_SetTerminals(10, 0); //Turn off all terminals on Pot 11
      status_buffer_1[HBRIDGE_LOC] |= TWELVE_OUT_2_MASK;
    }
    else {
      MCP41HVExtender_SetTerminals(10, SPIpotTCONSettings[10]); //Reset all terminals on Pot 11
      status_buffer_1[HBRIDGE_LOC] &= ~TWELVE_OUT_2_MASK; //Clear the bit
    }
    digitalWrite(IH2Pin, IH2State);
    if (debugDisplay) {
      connectionString(IH2State);
      Serial.println(displayBuffer);
    }
    return IH2State;
  }
  else if (settingNum == 47) {
    if (settingValue > -1) IL1State = boolean(settingValue);
    else IL1State = boolean(status_buffer_1[HBRIDGE_LOC] & GROUND_OUT_1_MASK);
    if (IL1State) {
      PWM4Out = false;
      status_buffer_1[HBRIDGE_LOC] |= GROUND_OUT_1_MASK; //Set the bit with an OR
    }
    else {
      PWM4Out = true;
      status_buffer_1[HBRIDGE_LOC] &= ~GROUND_OUT_1_MASK; //Clear the bit
    }
    setPWMSwitches(); //Turn off PWM4
    delay(1);
    digitalWrite(IL1Pin, IL1State);
    if (debugDisplay) {
      connectionString(IL1State);
      Serial.println(displayBuffer);
    }
    return IL1State;
  }
  else if (settingNum == 48) {
    if (settingValue > -1) IL2State = boolean(settingValue);
    else IL2State = boolean(status_buffer_1[HBRIDGE_LOC] & GROUND_OUT_2_MASK);
    if (IL2State) {
      MCP41HVExtender_SetTerminals(11, 0); //Turn off all terminals on Pot 12
      status_buffer_1[HBRIDGE_LOC] |= GROUND_OUT_2_MASK; //Set the bit with an OR
    }
    else {
      MCP41HVExtender_SetTerminals(11, SPIpotTCONSettings[11]); //Reset all terminals
      status_buffer_1[HBRIDGE_LOC] &= ~GROUND_OUT_2_MASK; //Clear the bit
    }
    delay(1);
    digitalWrite(IL2Pin, IL2State);
    if (debugDisplay) {
      connectionString(IL2State);
      Serial.println(displayBuffer);
    }
    return IL2State;
  }
  else if (settingNum == 49) {
    if (settingValue > -1) HVoutAdjValue = uint8_t(settingValue);
    else HVoutAdjValue = status_buffer_1[HVADJOUT_LOC];
    uint8_t ret_value = MCP41HVI2C_SetWiper(HVoutAdjAddr, HVoutAdjValue);
    if (debugDisplay) {
      Serial.print(HVoutAdjValue);
      Serial.print(", ");
      Serial.println(ret_value);
    }
    return ret_value;
  }
  else if (settingNum == 50) {
    if (settingValue > -1) ignitionCtlState = boolean(settingValue);
    digitalWrite(ignitionCtlPin, ignitionCtlState);
    if (ignitionCtlState) {
      status_buffer_1[HBRIDGE_LOC] |= IGNITION_RELAY_MASK; //Set the bit
      Can0.begin(0); //Autobaud
      Can1.begin(0); //Autobaud
    }
    else {
      status_buffer_1[HBRIDGE_LOC] &= ~(IGNITION_RELAY_MASK); //clear the bit
    }
    if (debugDisplay) {
      connectionString(ignitionCtlState);
      Serial.println(displayBuffer);
    }
    return ignitionCtlState;
  }
  else if (settingNum >  50 && settingNum <= 66) {
    // Terminal connections on the potentiometers
    if (settingValue > -1) SPIpotTCONSettings[settingNum - 51] = uint8_t(settingValue);
    else SPIpotTCONSettings[settingNum - 51] = status_buffer_2[settingNum - 50];
    uint8_t terminalConnection = MCP41HVExtender_SetTerminals(settingNum - 51, SPIpotTCONSettings[settingNum - 51]);
    if (debugDisplay) {
      Serial.print(SPIpotTCONSettings[settingNum - 51]);
      Serial.print(", ");
      terminalString(terminalConnection);
      Serial.println(displayBuffer);
    }
    return SPIpotTCONSettings[settingNum - 51];
  }
  else if (settingNum == 67) {
    if (settingValue > -1) PWM1Out = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(PWM1Out);
      Serial.println(displayBuffer);
    }
    return PWM1Out;
  }
  else if (settingNum == 68) {
    if (settingValue > -1) PWM2Out = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(PWM2Out);
      Serial.println(displayBuffer);
    }
    return PWM2Out;
  }
  else if (settingNum == 70) {
    if (settingValue > -1) PWM4Out = boolean(settingValue);
    else getPWMSwitches(status_buffer_1[PWM_SETTINGS_LOC]);
    setPWMSwitches();
    if (debugDisplay) {
      connectionString(PWM4Out);
      Serial.println(displayBuffer);
    }
    return PWM4Out;
  }
  else if (settingNum == 71) {
    if (settingValue > -1) LIN1Switch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(LIN1Switch);
      Serial.println(displayBuffer);
    }
    return LIN1Switch;
  }
  else if (settingNum == 72) {
    if (settingValue > -1) LIN2Switch = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (LIN2Switch) MCP41HVExtender_SetTerminals(15, 0); //Setting 66
    else MCP41HVExtender_SetTerminals(15, SPIpotTCONSettings[15]);
    if (debugDisplay) {
      connectionString(LIN2Switch);
      Serial.println(displayBuffer);
    }
    return LIN2Switch;
  }
  else if (settingNum == 73) {
    if (settingValue > -1) U1though8Enable = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U1though8Enable);
      Serial.println(displayBuffer);
    }
    return U1though8Enable;
  }
  else if (settingNum == 74) {
    if (settingValue > -1) U9though16Enable = boolean(settingValue);
    else {
      memcpy(&configSwitchSettings, &status_buffer_1[CONFIG_SWITCH_SETTINGS_LOC], 2);
      getConfigSwitches(configSwitchSettings);
    }
    setConfigSwitches();
    if (debugDisplay) {
      connectionString(U9though16Enable);
      Serial.println(displayBuffer);
    }
    return U9though16Enable;
  }
  else if (settingNum == 75) {
    if (settingValue > -1) I2CpotWiperSettings[0] = uint8_t(settingValue);
    else I2CpotWiperSettings[0] = status_buffer_1[U34_WIPER_LOC];
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(I2CpotAddr[0], I2CpotWiperSettings[0]);
    if (debugDisplay) {
      Serial.print(I2CpotWiperSettings[0]);
      Serial.print(", ");
      Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 76) {
    if (settingValue > -1) I2CpotWiperSettings[1] = uint8_t(settingValue);
    else I2CpotWiperSettings[1] = status_buffer_1[U36_WIPER_LOC];
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(I2CpotAddr[1], I2CpotWiperSettings[1]);
    if (debugDisplay) {
      Serial.print(I2CpotWiperSettings[1]);
      Serial.print(", ");
      Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 77) {
    if (settingValue > -1) I2CpotWiperSettings[2] = uint8_t(settingValue);
    else I2CpotWiperSettings[2] = status_buffer_1[U37_WIPER_LOC];
    uint8_t terminalConnection = MCP41HVI2C_SetWiper(I2CpotAddr[2], I2CpotWiperSettings[2]);
    if (debugDisplay) {
      Serial.print(I2CpotWiperSettings[2]);
      Serial.print(", ");
      Serial.println(terminalConnection);
    }
    return terminalConnection;
  }
  else if (settingNum == 78) {
    if (settingValue > -1) I2CpotTCONSettings[0] = uint8_t(settingValue);
    else I2CpotTCONSettings[0] = status_buffer_2[U34_TCON_LOC];
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(I2CpotAddr[0], I2CpotTCONSettings[0]);
    if (debugDisplay) {
      Serial.print(I2CpotTCONSettings[0]);
      Serial.print(", ");
      terminalString(terminalConnection);
      Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 79) {
    if (settingValue > -1) I2CpotTCONSettings[1] = uint8_t(settingValue);
    else I2CpotTCONSettings[1] = status_buffer_2[U36_TCON_LOC];
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(I2CpotAddr[1], I2CpotTCONSettings[1]);
    if (debugDisplay) {
      Serial.print(I2CpotTCONSettings[1]);
      Serial.print(", ");
      terminalString(terminalConnection);
      Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum == 80) {
    if (settingValue > -1) I2CpotTCONSettings[2] = uint8_t(settingValue);
    else I2CpotTCONSettings[2] = status_buffer_2[U37_TCON_LOC];
    uint8_t terminalConnection = MCP41HVI2C_SetTerminals(I2CpotAddr[2], I2CpotTCONSettings[2]);
    if (debugDisplay) {
      Serial.print(I2CpotTCONSettings[2]);
      Serial.print(", ");
      terminalString(terminalConnection);
      Serial.println(displayBuffer);
    }
    return terminalConnection;
  }
  else if (settingNum >= 81 && settingNum <= 85 ) {
    if (settingValue > -1)  pwmFrequency[settingNum - 81] = uint16_t(constrain(settingValue, 0, 4096));
    else {
      if      (settingNum == 81) memcpy(&pwmFrequency[0], &status_buffer_1[PWM1_FREQ_LOC], 2);
      else if (settingNum == 82) memcpy(&pwmFrequency[1], &status_buffer_1[PWM1_FREQ_LOC], 2);
      else if (settingNum == 83) memcpy(&pwmFrequency[2], &status_buffer_1[PWM3_FREQ_LOC], 2);
      else if (settingNum == 84) memcpy(&pwmFrequency[3], &status_buffer_1[PWM3_FREQ_LOC], 2);
      else if (settingNum == 85) memcpy(&pwmFrequency[4], &status_buffer_1[PWM5_FREQ_LOC], 2);
    }
    analogWriteFrequency(PWMPins[settingNum - 81], float(pwmFrequency[settingNum - 81]));
    for (uint8_t i = 0; i < numPWMs; i++) analogWrite(PWMPins[i], pwmValue[i]);
    if (debugDisplay) {
      Serial.println(pwmFrequency[settingNum - 81]);
    }
    if      (settingNum == 81) {
      pwmFrequency[1] = pwmFrequency[0];
      memcpy(&status_buffer_1[PWM1_FREQ_LOC], &pwmFrequency[0], 2);
    }
    else if (settingNum == 82) {
      pwmFrequency[0] = pwmFrequency[1];
      memcpy(&status_buffer_1[PWM1_FREQ_LOC], &pwmFrequency[1], 2);
    }
    else if (settingNum == 83) {
      pwmFrequency[2] = pwmFrequency[2];
      memcpy(&status_buffer_1[PWM3_FREQ_LOC], &pwmFrequency[2], 2);
    }
    else if (settingNum == 84) {
      pwmFrequency[2] = pwmFrequency[3];
      memcpy(&status_buffer_1[PWM3_FREQ_LOC], &pwmFrequency[3], 2);
    }
    else if (settingNum == 85) {
      memcpy(&status_buffer_1[PWM5_FREQ_LOC], &pwmFrequency[4], 2);
    }
    return pwmFrequency[settingNum - 81];
  }
  else if (settingNum == 86) {
    if (settingValue > -1) PWM4Out_28 = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(PWM4Out_28);
      Serial.println(displayBuffer);
    }
    return PWM4Out_28;
  }
  else if (settingNum == 87) { //PWM5
    if (settingValue > -1) pwmValue[4] = uint16_t(settingValue);
    else  memcpy(&pwmValue[4], &status_buffer_1[PWM5_LOC], 2);
    for (uint8_t i = 0; i < numPWMs; i++) analogWrite(PWMPins[i], pwmValue[i]);
    if (debugDisplay) Serial.println(pwmValue[4]);
    memcpy(&status_buffer_1[PWM5_LOC], &pwmValue[4], 2);
    return pwmValue[4];
  }
  else if (settingNum == 88) { //PWM6
    if (settingValue > -1) pwmValue[5] = uint16_t(settingValue);
    else  memcpy(&pwmValue[5], &status_buffer_1[PWM6_LOC], 2);
    analogWriteFrequency(PWMPins[5], pwmFrequency[4]);
    for (uint8_t i = 0; i < numPWMs; i++) analogWrite(PWMPins[i], pwmValue[i]);
    if (debugDisplay) Serial.println(pwmValue[5]);
    memcpy(&status_buffer_1[PWM6_LOC], &pwmValue[5], 2);
    return pwmValue[5];
  }
  else if (settingNum == 89) {
    if (settingValue > -1) PWM5Out = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(PWM5Out);
      Serial.println(displayBuffer);
    }
    return PWM5Out;
  }
  else if (settingNum == 90) {
    if (settingValue > -1) PWM6Out = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(PWM6Out);
      Serial.println(displayBuffer);
    }
    return PWM6Out;
  }
  else if (settingNum == 91) {
    if (settingValue > -1) CAN1out = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    if (CAN1out) {
      MCP41HVExtender_SetTerminals(2, 0);
      MCP41HVExtender_SetTerminals(3, 0);
    }
    else {
      MCP41HVExtender_SetTerminals(2, SPIpotTCONSettings[2]);
      MCP41HVExtender_SetTerminals(3, SPIpotTCONSettings[3]);
    }
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(CAN1out);
      Serial.println(displayBuffer);
    }
    return CAN1out;
  }
  else if (settingNum == 93) {
    if (settingValue > -1) CAN0term1 = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(CAN0term1);
      Serial.println(displayBuffer);
    }
    return CAN0term1;
  }
  else if (settingNum == 94) {
    if (settingValue > -1) CAN1term1 = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(CAN1term1);
      Serial.println(displayBuffer);
    }
    return CAN1term1;
  }
  else if (settingNum == 95) {
    if (settingValue > -1) CAN2term1 = boolean(settingValue);
    else  getTerminationSwitches(status_buffer_1[TERMINATION_SETTINGS_LOC]);
    setTerminationSwitches();
    if (debugDisplay) {
      connectionString(CAN2term1);
      Serial.println(displayBuffer);
    }
    return CAN2term1;
  }
  else return -1;
}
/*               End Define Settings                              */
/******************************************************************/

void setCompIdEEPROMdata () {
  char id[256];
  componentID.toCharArray(id, 255);
  EEPROM.put(componentIDAddress, id);
  Serial.print("SAVED ");
  Serial.println(componentID);
}

void getCompIdEEPROMdata () {
  char id[256];
  EEPROM.get(componentIDAddress, id);
  if (id[0] == 255) {
    setCompIdEEPROMdata ();
  }
  else {
    componentID = String(id);
    Serial.print("LOADED ");
    Serial.println(componentID);
  }
}

void displayJ1708() {
  if (commandString.toInt() > 0) {
    showJ1708 = true;
    status_buffer_2[NET_STATUS_LOC] |= STREAM_J1708_MASK;
  }
  else {
    showJ1708 = false;
    status_buffer_2[NET_STATUS_LOC] &= ~STREAM_J1708_MASK;
  }
  //clear the RX buffer on start.
  J1708.clear();
  J1708_index = 0;
  newJ1708Char = false;
  firstJ1708 = true;
}

void  adjustError() {
  Serial.println(("INFO SS - Condition not met. Turn adjust mode on by typing AO, then select a setting with CS"));
}



/********************************************************************************************/
/*                       Begin Function Calls for Serial Commands                           */

void turnOnAdjustMode() {
  ADJUST_MODE_ON = 1;
  Serial.println(("INFO AO - Turned Adjustment mode on. Type AF or click to turn off. Type SS,XXXX  or scroll knob and click to set settings."));
  Serial.print(("INFO Current Setting for Adjustement is "));
  Serial.print(currentSetting);
  Serial.print(" - ");
  Serial.println(settingNames[currentSetting]);
  knob.write(setSetting(currentSetting, -1, DEBUG_OFF));
  setLimits(currentSetting);

}

void turnOffAdjustMode() {
  ADJUST_MODE_ON = 0;
  Serial.println(("INFO AF - Turned Setting Adjustment mode off. Type AO to turn on. Scroll knob to select a setting."));
  knob.write(currentSetting);
  knobLowLimit = 1;
  knobHighLimit = numSettings - 1;
}

void fastSetSetting() {
  uint16_t returnval;
  currentSetting = commandPrefix.toInt();
  if (currentSetting > 0 && currentSetting < numSettings) {
    setLimits(currentSetting);
    if (commandString.length() > 0) {
      long settingValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
      returnval = setSetting(currentSetting, settingValue, DEBUG_OFF);
    }
    else {
      returnval = setSetting(currentSetting, -1, DEBUG_OFF);
    }
    Serial.print("SET ");
    Serial.print(currentSetting);
    Serial.print(",");
    Serial.println(returnval);
  }
  else Serial.println(("ERROR in setting value."));
}

void changeSetting() {
  Serial.println(("INFO CS - Change or Select Setting."));
  if (commandString.length() > 0) {
    currentSetting = constrain(commandString.toInt(), 0, numSettings);

  }
  //listSetting(currentSetting);
  if (ADJUST_MODE_ON) {
    setLimits(currentSetting);
    knob.write(setSetting(currentSetting, -1, DEBUG_OFF));
  }
  else {
    if (knob.read() == currentSetting) {
      Serial.print("INFO ");
      setSetting(currentSetting, -1, DEBUG_ON);
    }
    else knob.write(currentSetting); //automatic listSetting if knob changes
    knobLowLimit = 1;
    knobHighLimit = numSettings - 1;
  }
}

void listSettings() {
  Serial.println(("INFO LS - List Settings. "));
  for (int i = 1; i < numSettings; i++) {
    Serial.print("INFO ");
    setSetting(i, -1, DEBUG_ON);
  }
}

void changeValue() {
  //Set value from Serial commands
  if (ADJUST_MODE_ON && currentSetting != 0) {
    Serial.println(("INFO SS - Set Setting."));
    int adjustmentValue = constrain(commandString.toInt(), knobLowLimit, knobHighLimit);
    currentKnob = setSetting(currentSetting, adjustmentValue, DEBUG_ON);
    knob.write(currentKnob);
  }
  else
  {
    adjustError();
  }
}


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

void myClickFunction() {}
void myDoubleClickFunction() {
  save_settings();
}
void longPress() {} //Do nothing at this time.
void longPressStop() {}
/*                         End Function Calls for Knob Buttons                            */
/********************************************************************************************/




void startStopCAN0Streaming() {
  if (commandString.toInt() > 0) {
    displayCAN0 = true;
    status_buffer_2[NET_STATUS_LOC] |= STREAM_CAN0_MASK;
  }
  else {
    displayCAN0 = false;
    status_buffer_2[NET_STATUS_LOC] &= ~STREAM_CAN0_MASK;
  }
}

void startStopCAN1Streaming() {
  if (commandString.toInt() > 0) {
    displayCAN1 = true;
    status_buffer_2[NET_STATUS_LOC] |= STREAM_CAN1_MASK;
  }
  else {
    displayCAN1 = false;
    status_buffer_2[NET_STATUS_LOC] &= ~STREAM_CAN1_MASK;
  }
}

void startStopCAN2Streaming() {
  if (commandString.toInt() > 0) {
    displayCAN2 = true;
    status_buffer_2[NET_STATUS_LOC] |= STREAM_CAN2_MASK;
  }
  else {
    displayCAN2 = false;
    status_buffer_2[NET_STATUS_LOC] &= ~STREAM_CAN2_MASK;
  }
}

void sendMessage() {
  /* Sends a CAN message from the following byte fields:
      extended/channel id dlc data
      channel is 0 for CAN0 and 1 for CAN1
      id is the CAN ID. if the ID is less than 11 bits and the first bit is set, then it will be transmitted as an extended id
  */
  boolean goodID = false;
  boolean goodData = false;
  memset(base64, 0, sizeof(base64));
  char commandCharBuffer[64];
  if (commandString.length() > 0) {
    commandString.toCharArray(base64, commandString.length());
    uint8_t binary_length = decode_base64(base64, commandCharBuffer);
    uint8_t i = 0;
    while (i < binary_length) {
      int8_t can_channel = commandCharBuffer[i] & 0x0F;
      if (can_channel > 2) return;
      Serial.printf("CS %d ", can_channel);

      boolean extended = bool(commandCharBuffer[i] & 0xF0);
      i += 1;
      if (extended) {
        uint32_t tempID;
        memcpy(&tempID, &commandCharBuffer[i], 4);
        i += 4;
        txmsg.ext = 1;
        txmsg.id = (0x3FFFFFFF & tempID); //29 bit ID
        Serial.printf("%08X ", txmsg.id);
      }
      else {
        uint16_t tempID;
        memcpy(&tempID, &commandCharBuffer[i], 2);
        i += 2;
        txmsg.ext = 0;
        txmsg.id = (0x7FF & tempID); //11 bit ID
        Serial.printf("%03X ", txmsg.id);
      }
      txmsg.len = constrain(commandCharBuffer[i], 0, 8);
      i += 1;
      Serial.printf("%X ", txmsg.len);

      memset(txmsg.buf, 0xff, sizeof(txmsg.buf));
      for (int j = 0; j < txmsg.len; j++) {
        txmsg.buf[j] = commandCharBuffer[i];
        i += 1;
        Serial.printf("%02X ", txmsg.buf[j]);
      }
      Serial.println();
      // Send out the data
      send_can_messages(txmsg, can_channel);
      txmsg.ext = 1; //set default
      txmsg.len = 8;
    }
  }
}






void parseJ1939(CAN_message_t &rxmsg ) {
  uint32_t ID = rxmsg.id;
  uint8_t DLC = rxmsg.len;
  uint8_t SA = (ID & 0xFF);
  uint8_t PF = (ID & 0x03FF0000) >> 16;
  uint8_t PRIORITY = (ID & 0x3C000000) >> 26;
  uint8_t DA = 0xFF;
  uint32_t PGN;
  if (PF >= 240) {
    PGN = (ID & 0x03FFFF00) >> 8;
  }
  else {
    //pdu1 format
    PGN = (ID & 0x03FF0000) >> 8;
    DA = (ID & 0x00000FF00);
  }

  if (PGN == 0xEA00 && DA == 0xFF) {
    //request message
    if (rxmsg.buf[0] == 0xEB && rxmsg.buf[1] == 0xFE) {
      if (enableSendComponentInfo) {
        if (canComponentIDtimer > 5000) {
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
void printFrame(CAN_message_t rxmsg, uint8_t can_channel, uint32_t RXCount)
{
  uint32_t timestamp = uint32_t(now());
  //21 Byte frame, three per HID message

  uint8_t can_message_index = (can_buffer[0] & 0x03) * CAN_FRAME_LENGTH + 1;
  can_buffer[0]++; // Normally CAN buffer will start with a value of 0x20 CAN_MESSAGE_BASE
  memcpy(&can_buffer[can_message_index + TIMESTAMP_OFFSET], &timestamp, 4);
  can_buffer[can_message_index + CHANNEL_DLC_OFFSET] = (can_channel << 4) + rxmsg.len;
  uint32_t microsecond_value = uint32_t(microsecondsPerSecond);
  memcpy(&can_buffer[can_message_index + MICROSECONDS_OFFSET], &microsecond_value, 3);
  uint32_t ID = (rxmsg.ext << 31) | (rxmsg.id);
  memcpy(&can_buffer[can_message_index + CAN_ID_OFFSET], &ID, 4);
  memcpy(&can_buffer[can_message_index + CAN_DATA_OFFSET], &rxmsg.buf, 8);

  if ((can_buffer[0] & 0x03) == 0x03 || can_send_time > CAN_SEND_MS) {
    can_send_time = 0;
    can_buffer[61]++; // increment the counter
    uint16_t can_checksum = CRC16.ccitt(can_buffer, 62);
    memcpy(&can_buffer[62], &can_checksum, 2);
    byte ret_val = RawHID.send(can_buffer, timeout);
    memset(can_buffer, 0xFF, 61);
    can_buffer[0] = 0x20;
  }
}

time_t getTeensy3Time() {
  microsecondsPerSecond = 0;
  return Teensy3Clock.get();
}

const uint32_t DEFAULT_TIME = 10;
char timeStamp[45];

uint32_t processSyncMessage() {
  char timeChar[15];
  memset(timeChar, 0, 15);
  commandString.toCharArray(timeChar, 15);
  time_t pctime = atol(timeChar);
  if (pctime > DEFAULT_TIME) {
    return pctime;
  }
  else if (pctime == 0) {
    Serial.printf("INFO Time is %04d-%02d-%02d %02d:%02d:%02d\n", year(), month(), day(), hour(), minute(), second());
    return 0L;
  }
  else {
    Serial.println("ERROR Time value is invalid.");
    return 0L;
  }
}

void displayTime() {
  time_t t = processSyncMessage();
  if (t != 0) {
    Teensy3Clock.set(t); // set the RTC
    setTime(t);
    Serial.printf("SET Time to %04d-%02d-%02d %02d:%02d:%02d\n", year(), month(), day(), hour(), minute(), second());
  }
}

void print_uid()  {
  Serial.printf("ID: %s\n", kinetisUID());
}

void printStats(CAN_stats_t currentStats, int channel) {
  Serial.printf("STATS for CAN%d: Enabled:%d, RingRXHighWater: %ul, ringRXFramesLost: %lu, ringTXHighWater: %lu, mailbox use count:[%lu",
                channel, currentStats.enabled, currentStats.ringRxHighWater, currentStats.ringRxFramesLost, currentStats.ringTxHighWater, currentStats.mb[0].refCount);
  for (int i = 1; i < 16; i++) {
    Serial.printf(", %lu", currentStats.mb[i].refCount);
  }
  Serial.printf("], overrunCount: [%lu", currentStats.mb[0].overrunCount);
  for (int i = 1; i < 16; i++) {
    Serial.printf(", %lu", currentStats.mb[i].overrunCount);
  }
  Serial.println("]");
}

void displayStats() {
  CAN_stats_t currentStats = Can0.getStats();
  printStats(currentStats, 0);
  currentStats = Can1.getStats();
  printStats(currentStats, 1);
}


void clearStats() {
  Can0.clearStats();
  Can1.clearStats();
  Can0.startStats();
  Can1.startStats();
  Serial.println("INFO Cleared CAN Statisitics");
}

void listInfo() {
  Serial.print("INFO SSS2 Component ID (Make*Model*Serial*Unit): ");
  Serial.println(componentID);
}

void changeComponentID() {
  if (commandString.length() < 12) {
    Serial.print(("INFO SSS2 Component ID: "));
    Serial.println(componentID);
  }
  else {
    componentID = commandString;
    setCompIdEEPROMdata();
    Serial.print(("SET SSS2 Component ID: "));
    Serial.println(componentID);
    setupComponentInfo();
  }
}

uint8_t getBAUD(uint32_t baudrate) {
  switch (baudrate)
  {
    case (250000):
      return CAN_250KBPS;
    case (500000):
      return CAN_500KBPS;
    case (125000):
      return CAN_125KBPS;
    case (666666):
      return CAN_666KBPS;
    case (1000000):
      return CAN_1000KBPS;
    case (4096):
      return CAN_4K096BPS;
    case (5000):
      return CAN_5KBPS;
    case (10000):
      return CAN_10KBPS;
    case (20000):
      return CAN_20KBPS;
    case (31250):
      return CAN_31K25BPS;
    case (40000):
      return CAN_40KBPS;
    case (50000):
      return CAN_50KBPS;
    case (80000):
      return CAN_80KBPS;
    case (100000):
      return CAN_100KBPS;
    case (200000):
      return CAN_200KBPS;
    default:
      return 0xFF;
  }
}

uint32_t setBAUD(uint8_t baudindex) {
  switch (baudindex)
  {
    case (CAN_250KBPS):
      return 250000;
    case (CAN_500KBPS):
      return 500000;
    case (CAN_125KBPS):
      return 125000;
    case (CAN_666KBPS):
      return 666666;
    case (CAN_1000KBPS):
      return 1000000;
    case (CAN_4K096BPS):
      return 4096;
    case (CAN_5KBPS):
      return 5000;
    case (CAN_10KBPS):
      return 10000;
    case (CAN_20KBPS):
      return 20000;
    case (CAN_31K25BPS):
      return 31250;
    case (CAN_40KBPS):
      return 40000;
    case (CAN_50KBPS):
      return 50000;
    case (CAN_80KBPS):
      return 80000;
    case (CAN_100KBPS):
      return 100000;
    case (CAN_200KBPS):
      return 200000;
    default:
      return 0xFFFFFFFF;
  }
}

void autoBaud0() {
  char baudstring[9];
  if (commandString.length() > 0) {
    commandString.toCharArray(baudstring, 9);
    uint32_t tempBAUDRATE = strtoul(baudstring, 0, 10);
    Can0.begin(tempBAUDRATE);
  }
  else {
    //Auto baud
    Can0.begin(0);
  }
  status_buffer_2[CAN0_BAUD_LOC] = getBAUD(Can0.baud_rate);
}

void autoBaud1() {
  char baudstring[9];
  if (commandString.length() > 0) {
    commandString.toCharArray(baudstring, 9);
    uint32_t tempBAUDRATE = strtoul(baudstring, 0, 10);
    Can1.begin(tempBAUDRATE);
  }
  else {
    //Auto baud
    Can1.begin(0);
  }

  status_buffer_2[CAN1_BAUD_LOC] = getBAUD(Can1.baud_rate);
}

void autoBaudMCP() {
  char baudstring[9];
  BAUDRATE_MCP = 250000;
  if (commandString.length() > 0) {
    commandString.toCharArray(baudstring, 9);
    uint32_t tempBAUDRATE = strtoul(baudstring, 0, 10);
    for (uint8_t baudRateIndex = 0; baudRateIndex < baudRateListLength; baudRateIndex++) {
      if (tempBAUDRATE == baudRateList[baudRateIndex]) {
        BAUDRATE_MCP = tempBAUDRATE;
        break;
      }
    }
  }
  if (MCPCAN.begin(MCP_ANY, getBAUD(BAUDRATE_MCP), MCP_16MHZ) == CAN_OK) {
    status_buffer_2[CAN2_BAUD_LOC] = getBAUD(BAUDRATE_MCP);
  }
  else {
    status_buffer_2[CAN2_BAUD_LOC] = 0xFE;
  }

  //  Serial.println("MCP2515 Initialized Successfully!");
  //  else Serial.println("Error Initializing MCP2515...");
  //  Serial.print("SET MCPCAN baudrate set to ");
  //  Serial.println(BAUDRATE_MCP);

}

void setEnableComponentInfo() {
  if (commandString.toInt() > 0) {
    setupComponentInfo();
    enableSendComponentInfo = true;
    Serial.println(("SET Enable CAN transmission of Component ID"));
    if (canComponentIDtimer > 5000) {
      canComponentIDtimer = 0;
      can_messages[comp_id_index]->enabled = true;
      can_messages[comp_id_index]->transmit_number = 0;
      can_messages[comp_id_index]->ok_to_send = true;
      can_messages[comp_id_index]->loop_cycles = 0;
      can_messages[comp_id_index]->cycle_count = 0;
      can_messages[comp_id_index]->message_index = 0;
    }
  }
  else {
    enableSendComponentInfo = false;
    Serial.println(("SET Disable CAN transmission of Component ID"));
    can_messages[comp_id_index]->enabled = false;
  }
}

void getTeensyTime() {
  Serial.printf("INFO Timestamp: %D\n", now());
}

void checkAgainstUID() {
  Serial.println("OK:Authenticated");
}

void make_buffer_4() {
  status_buffer_4[0] = 0x40;
  //Set the CAN message data to the next one in the list.
  status_buffer_4[1] = can_messages[buffer4_index]->channel;
  memcpy(&status_buffer_4[2], &buffer4_index, 2);
  memcpy(&status_buffer_4[4], &buffer4_message_index, 2);
  status_buffer_4[6] = can_messages[buffer4_index]->txmsg.len;
  memcpy(&status_buffer_4[7], &can_messages[buffer4_index]->tx_period, 4);
  memcpy(&status_buffer_4[11], &can_messages[buffer4_index]->loop_cycles, 4);
  status_buffer_4[15] = can_messages[buffer4_index]->num_messages;
  memcpy(&status_buffer_4[16], &can_messages[buffer4_index]->stop_after_count, 4);
  memcpy(&status_buffer_4[20], &can_messages[buffer4_index]->id_list[buffer4_message_index], 4);
  memcpy(&status_buffer_4[24], &can_messages[buffer4_index]->message_list[buffer4_message_index], 8);
  status_buffer_4[32] = can_messages[buffer4_index]->enabled;
  memcpy(&status_buffer_4[33], &can_messages[buffer4_index]->transmit_number, 4);
  memset(&status_buffer_4[37], 0x00, 24); //Clear string
  memcpy(&status_buffer_4[37], &can_messages[buffer4_index]->ThreadName[0], min(can_messages[buffer4_index]->ThreadName.length(),24));
  buffer4_message_index++;
  if (buffer4_message_index >= can_messages[buffer4_index]->num_messages ) {
    buffer4_message_index = 0;
    buffer4_index++;
    if (buffer4_index >= can_thread_controller.size(false)) buffer4_index = 0;
  }

}

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  // Serial.print("0x");
  for (int i = 0; i < length; i++)
  {
    if (data[i] < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
}
