/*
 * Object Oriented CAN example for Teensy 3.6 with Dual CAN buses 
 * By Collin Kidder. Based upon the work of Pawelsky and Teachop
 * 
 *
 * The reception of frames in this example is done via callbacks
 * to an object rather than polling. Frames are delivered as they come in.
 */

#include <FlexCAN.h>

#ifndef __MK66FX1M0__
  #error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

// Create a new class to define functions in the CANListener class in FlexCAN.cpp
class CANPrinter : public CANListener 
{
public:
   //overrides the parent version so we can actually do something
   void printFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller);
   void logFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller);
   bool frameHandler(CAN_message_t &frame, int8_t mailbox, uint8_t controller); 
};

void CANPrinter::printFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller)
{
    // This function can't keep up with a fully loaded CAN bus.
    char message[19];
    if (mailbox < 0){
      sprintf(message, "%5d Can%d", frame.timestamp, controller);   
    }
    else {
      sprintf(message, "%5d Can%d (%d)", frame.timestamp, controller, mailbox);
    }
    Serial.print(message);

    if (frame.flags.extended){
      sprintf(message, "  %08X   [%d] ", frame.id, frame.len);
    }
    else {
      sprintf(message, "  %03X   [%d] ", frame.id, frame.len);
    }
    Serial.print(message);
    char byteStr[5];
    for (uint8_t c = 0; c < frame.len; c++) 
    {
       sprintf(byteStr, " %02X", frame.buf[c]); 
       Serial.print(byteStr);
    }
    Serial.println();
}

void CANPrinter::logFrame(CAN_message_t &frame, int8_t mailbox, uint8_t controller)
{
  //Add code to log data here.
  
}

bool CANPrinter::frameHandler(CAN_message_t &frame, int8_t mailbox, uint8_t controller)
{
    // This function can't keep up with a fully loaded CAN bus
    // A fully loaded bus may crash the Teensy when filling the serial buffer
    printFrame(frame, mailbox, controller);

    // be sure to return true to tell the library that you've processed the frame.
    // Otherwise, the data goes into a ringbuffer and waits to be read. 
    return true;
}

CANPrinter canPrinter;


// -------------------------------------------------------------
void setup(void)
{
  Serial.println(F("Hello Teensy 3.6 dual CAN Test With Objects."));

  CAN_filter_t standardPassFilter;
  standardPassFilter.id=0;
  standardPassFilter.flags.extended=0;

  CAN_filter_t extendedPassFilter;
  extendedPassFilter.id=0;
  extendedPassFilter.flags.extended=1;
  
  // Options for a Detroit Diesel CPC4
  Can0.begin(250000,standardPassFilter); 
  // standardPassFilter is the default option and can be omitted. 
  Can1.begin(666666);
  Can0.attachObj(&canPrinter);
  Can1.attachObj(&canPrinter);

  // Give a few mailboxes the ability to read extended IDs
  for (uint8_t filterNum = 0; filterNum < 4; filterNum++){
    Can0.setFilter(extendedPassFilter,filterNum); 
    Can1.setFilter(extendedPassFilter,filterNum); 
  }

  //Set up all the mailbox handlers
  for (uint8_t mailbox = 0; mailbox < NUM_MAILBOXES; mailbox++){
     canPrinter.attachMBHandler(mailbox);
  }
  // Alternatively, you can use the general handler, but this won't accept mixed
  // extended and standard ID frames.
  //canPrinter.attachGeneralHandler();

  
}

// -------------------------------------------------------------
void loop(void)
{
//  Serial.println(Can0.readRxError());
//  Serial.println(Can1.readRxError());
}


