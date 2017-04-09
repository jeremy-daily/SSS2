 /* Basic Raw HID Transmit Speed Test Example
   Teensy can send/receive 64 byte packets with a
   dedicated program running on a PC or Mac.

   You must select Raw HID from the "Tools > USB Type" menu

*/

#define VENDOR_ID               0x16C1
#define PRODUCT_ID              0x0481
//#define RAWHID_USAGE_PAGE       0xFFAC  // recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE            0x0201  // recommended: 0x0100 to 0xFFFF
//
//#define RAWHID_TX_SIZE          24      // transmit packet size
//#define RAWHID_TX_INTERVAL      1       // max # of ms between transmit packets
//#define RAWHID_RX_SIZE          64      // receive packet size
//#define RAWHID_RX_INTERVAL      8       // max # of ms between receive packets

// RawHID packets are always 64 bytes
uint8_t  HIDTXbuffer[24];
uint32_t packetCount = 0;

elapsedMillis displayTimer;
elapsedMicros transmitTimer;

void setup() {
}


void loop() {  
  uint32_t currentMicros = micros();
  HIDTXbuffer[0] = (currentMicros & 0xFF000000) >> 24;
  HIDTXbuffer[1] = (currentMicros & 0x00FF0000) >> 16;
  HIDTXbuffer[2] = (currentMicros & 0x0000FF00) >> 8;
  HIDTXbuffer[3] = (currentMicros & 0x000000FF) >> 0;
  
  HIDTXbuffer[60] = (packetCount & 0xFF000000) >> 24;
  HIDTXbuffer[61] = (packetCount & 0x00FF0000) >> 16;
  HIDTXbuffer[62] = (packetCount & 0x0000FF00) >> 8;
  HIDTXbuffer[63] = (packetCount & 0x000000FF) >> 0;
  if (transmitTimer >= 1000){
    transmitTimer = 0;
    uint8_t n = RawHID.send(HIDTXbuffer, 0);
    if (n > 0) packetCount++;
  }
  if (displayTimer >= 1000){
    displayTimer=0;
    Serial.print(packetCount);
    Serial.print(", ");
    Serial.println(currentMicros);
  }
}
