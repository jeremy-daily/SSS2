#include "FastCRC.h"
FastCRC16 CRC16; 

elapsedMillis usb_tx_timer;

uint8_t status_buffer_1[64];
uint8_t status_buffer_2[64];
uint8_t status_buffer_3[64];
uint8_t ret_val;

const uint8_t timeout = 0;

void setup() {
  // put your setup code here, to run once:
  status_buffer_1[0]=0x01;
  status_buffer_2[0]=0x02;
  status_buffer_3[0]=0x03;
}

void loop() {
  // put your main code here, to run repeatedly:

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
