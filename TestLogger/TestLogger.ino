/*
 * NMFTA CAN Logger Project
 * 
 * Arduino Sketch to test the ability to receive CAN messages
 * 
 * Written By Dr. Jeremy S. Daily
 * The University of Tulsa
 * Department of Mechanical Engineering
 * 
 * 29 September 2016
 * 
 * Released under the MIT License
 *
 * Copyright (c) 2016        Jeremy S. Daily
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
 * This program logs data to a binary file.  Functions are included
 * to convert the binary file to a csv text file.
 *
 * Samples are logged at regular intervals.  The maximum logging rate
 * depends on the quality of your SD card. 
 * 
 * Data is written to the file using a SD multiple block write command.
 * 
 * Much of this sketch was inspired by the examples from https://github.com/greiman/SdFat
 * 
 */

// log file base name.  Must be five characters or less. Change this line for each upload
// The first four characters should match the label on the cable.
#define FILE_BASE_NAME "TU02_"

//included libraries 
#include <SPI.h>
#include "SdFat.h"
#include <FlexCAN.h>
#include <TimeLib.h>

SdFatSdioEX sd;

// 32 KiB buffer.
const size_t BUF_DIM = 4096;
const size_t nb = 2048;
File file;

uint8_t buf[BUF_DIM];

// buffer as uint32_t
uint32_t* buf32 = (uint32_t*)buf;

bool sdBusy() {
  return sd.card()->isBusy();
}
//-----------------------------------------------------------------------------
void errorHalt(const char* msg) {
    sd.errorHalt(msg);
}
//------------------------------------------------------------------------------
uint32_t kHzSdClk() {
  return sd.card()->kHzSdClk();
}  

static CAN_message_t rxmsg;
CAN_filter_t allPassFilter;

elapsedMillis rxtimer;

uint32_t RXCount;
char outMessage[27] ={};
char displayBuffer[64];
void printFrame(CAN_message_t rxmsg, int mailbox, uint8_t channel, uint32_t RXCount)
{ 
  uint32_t currentMicros = micros();
  uint8_t *idPointer = (uint8_t *)&rxmsg.id;
  uint8_t *RXCountPointer = (uint8_t *)&RXCount;
  uint8_t *microsPointer = (uint8_t *)&currentMicros;
  char outMessage[27] ={};
  outMessage[0]='C';
  outMessage[1]='A';
  outMessage[2]='N';
  outMessage[3]=channel;
  outMessage[4]=RXCountPointer[0];
  outMessage[5]=RXCountPointer[1];
  outMessage[6]=RXCountPointer[2];
  outMessage[7]=RXCountPointer[3];
  outMessage[8]=microsPointer[0];
  outMessage[9]=microsPointer[1];
  outMessage[10]=microsPointer[2];
  outMessage[11]=microsPointer[3];
  outMessage[12]=idPointer[0];
  outMessage[13]=idPointer[1];
  outMessage[14]=idPointer[2];
  outMessage[15]=idPointer[3];
  outMessage[16]=rxmsg.len;
  outMessage[17]=rxmsg.buf[0];
  outMessage[18]=rxmsg.buf[1];
  outMessage[19]=rxmsg.buf[2];
  outMessage[20]=rxmsg.buf[3];
  outMessage[21]=rxmsg.buf[4];
  outMessage[22]=rxmsg.buf[5];
  outMessage[23]=rxmsg.buf[6];
  outMessage[24]=rxmsg.buf[7];
  outMessage[25]=0x0A;
  //Serial.write(outMessage,26);
  

  sprintf(displayBuffer,"CAN%d,%10lu,%10lu,%08X,%d,%d,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
          channel,RXCount,micros(),rxmsg.id,rxmsg.ext,rxmsg.len,
          rxmsg.buf[0],rxmsg.buf[1],rxmsg.buf[2],rxmsg.buf[3],
          rxmsg.buf[4],rxmsg.buf[5],rxmsg.buf[6],rxmsg.buf[7]);
  Serial.print(displayBuffer);
}
uint8_t filecounter=0;

void setup(){
   while(!Serial);
  Serial.println("CAN Message Test");
  
  
  if (!sd.begin()) {
      sd.initErrorHalt("SdFatSdioEX begin() failed");
  }
  // make sdEx the current volume.
  sd.chvol();

  if (!file.open("CANData.bin", O_RDWR | O_CREAT)) {
    errorHalt("open failed");
  }
  Can0.begin(250000);
  Can1.begin(250000);
  
  Serial.print(F("Setting CAN Filters..."));
  //leave the first 4 mailboxes to use the default filter. Just change the higher ones
  allPassFilter.id=0;
  allPassFilter.ext=1;
  allPassFilter.rtr=0;
  for (uint8_t filterNum = 4; filterNum < 16;filterNum++){
    Can0.setFilter(allPassFilter,filterNum); 
    Can1.setFilter(allPassFilter,filterNum); 
  }
  Serial.println(" Done.");
  
  RXCount = 0;
 
}

void loop(){
  

  if (Can0.read(rxmsg)){
      printFrame(rxmsg, -1, 0, RXCount);
      file.write(displayBuffer,64);
      RXCount++;
      
  }
  if (Can1.read(rxmsg)){
      printFrame(rxmsg, -1, 1, RXCount);
      file.write(displayBuffer,64);
      RXCount++;
      
  }
  if (rxtimer >= 10000){
    
    rxtimer = 0;
    file.close();
    //Serial.printf("RX Messages = %lu\n",RXCount);
    char fname[20];
    sprintf(fname,"AData%03d.csv",filecounter++);
    if (!file.open(fname, O_RDWR | O_CREAT)) {
      errorHalt("open failed");
    }
  }
}

