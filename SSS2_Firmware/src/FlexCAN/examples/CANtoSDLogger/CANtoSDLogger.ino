/*
 * Object Oriented CAN example for Teensy 3.6 with Dual CAN buses 
 * By Collin Kidder. Based upon the work of Pawelsky and Teachop
 * 
 *
 * The reception of frames in this example is done via callbacks
 * to an object rather than polling. Frames are delivered as they come in.
 */
#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

// FlexCAN defines CAN messages and sets the time. 
#include "FlexCAN.h"
#include "SdFat.h"

struct StoredCANDataType_t {
  time_t timeStamp;
  uint32_t usec;
  uint32_t DLC;
  uint32_t ID;
  uint8_t dataField[8];
} StoredCANDataType_t;

#define BUFFER_SIZE   512
#define MESSAGE_COUNT BUFFER_SIZE/sizeof(StoredCANDataType_t)
// 2 GB file.
const uint32_t BUF_DIM = (1 << 31)/BUFFER_SIZE;
#define FILE_SIZE     BUFFER_SIZE*BUF_DIM

CAN_message_t msg0, msg1;


//uint8_t buf[BUF_DIM];

// buffer as uint32_t
//uint32_t* buf32 = (uint32_t*)buf;

SdFatSdioEX sdEx;

File file;

bool OKtoWrite = false;
bool waiting = false;


elapsedMicros writeMicros; // Keep track of performance
uint8_t RxError;
elapsedMillis printTimer; // Make sure serial print statements don't flood the bus
elapsedMillis errorCountTimer;

elapsedMillis recordTimer;

// -------------------------------------------------------------
void setup(void)
{
  
  Serial.println(F("Hello Teensy 3.6 dual CAN Test With Objects."));
  Serial.println("Creating files with ");

  //Start the SD Card
  if (!sdEx.begin()) {
    sdEx.initErrorHalt("SdFatSdioEX begin() failed");
  }
  
  // make sdEx the current volume.
  sdEx.chvol();
  
  //Make a new file.
  if (!file.open("TeensyCANLog.bin", (O_WRITE | O_CREAT | O_TRUNC))) {
    sdEx.errorHalt("open failed");
  }
  OKtoWrite = true;
  
  CAN_filter_t standardPassFilter;
  standardPassFilter.id=0;
  standardPassFilter.flags.extended=0;

  CAN_filter_t extendedPassFilter;
  extendedPassFilter.id=0;
  extendedPassFilter.flags.extended=1;
  
  // Options for a Detroit Diesel CPC4
  // J1939
  Can0.begin(250000,standardPassFilter); 
  
  // PT-CAN 
  // standardPassFilter is the default option and can be omitted. 
  Can1.begin(666666);

  // Give a few mailboxes the ability to read extended IDs
  for (uint8_t filterNum = 0; filterNum < 4; filterNum++){
    Can0.setFilter(extendedPassFilter,filterNum); 
    Can1.setFilter(extendedPassFilter,filterNum); 
  }

  recordTimer = 0;
  uint32_t messageIndex;
  uint32_t i = 0;
  uint32_t MESSAGE_COUNT = BUFFER_SIZE / 4;
  while (recordTimer<5000){
    Can1.read(msg1);
    if (Can0.read(msg0)){
      time_t t = msg0.utctime;
      char timeString[32];
      sprintf(timeString,"%04u-%02u-%02u %02u:%02u:%02u.%06u",year(t),month(t),day(t),hour(t),minute(t),second(t),uint32_t(msg0.microseconds));
      Serial.println(timeString);
//      ?Serial.println(messageIndex);
//      Serial.println(i);
      messageIndex ++;

      // Arbitration ID
      memcpy(&tempBuffer[i], &msg0.id, 4);
      i+=4;
      
       
      if (i>BUFFER_SIZE){
        Serial.println("Buffer overflow!");
      }
      if (messageIndex >= MESSAGE_COUNT){
        messageIndex = 0;
        writeMicros = 0;
        file.write(tempBuffer, BUFFER_SIZE);
        i = 0;
        Serial.println(writeMicros);
        
      }
    }
  }
  
  OKtoWrite = false;
  file.close();
  recordTimer = 0;
  // re-open the file for reading:
  file = sdEx.open("TeensyCANLog.bin", FILE_READ);
  if (file) {
    Serial.println("TeensyCANLog.bin:");

    // read from the file until there's nothing else in it:
    int count;
    char byteStr[4];
    Serial.println(file.available());
    while (file.available()) {
      OKtoWrite = false;
      sprintf(byteStr, " %02X", file.read()); 
      Serial.print(byteStr);
      count++;
      if (count >= BUFFER_SIZE){
        count = 0;
        delay(1);
        Serial.println();
      }
    }
    // close the file:
    file.close();
  }
  Serial.print("Done.");
}

// -------------------------------------------------------------
void loop(void)
{ 
  Can0.read(msg0);
  Can1.read(msg1);
  RxError = Can0.readRxError();
  if (RxError){
    if (errorCountTimer>5){
     errorCountTimer = 0;
     Serial.print("Can0 RxError: ");
     Serial.println(RxError);
    }
    
 
  }
  RxError = Can1.readRxError();
  if (RxError){
    Serial.print("Can1 RxError: ");
    Serial.println(RxError);
    
  }
  
}


