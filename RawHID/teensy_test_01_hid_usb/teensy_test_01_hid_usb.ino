
#define VENDOR_ID               0x16C0
#define PRODUCT_ID              0x0480
#define RAWHID_USAGE_PAGE       0xFFAC  // recommended: 0xFF00 to 0xFFFF
#define RAWHID_USAGE            0x0200  // recommended: 0x0100 to 0xFFFF

#define RAWHID_TX_SIZE          64      // transmit packet size
#define RAWHID_TX_INTERVAL      2       // max # of ms between transmit packets
#define RAWHID_RX_SIZE          64      // receive packet size
#define RAWHID_RX_INTERVAL      8       // max # of ms between receive packets

elapsedMillis usb_tx_timer;
uint32_t counter;
int8_t ret_val;
uint8_t timeout = 2;

uint8_t usb_buffer[64];

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  counter++;
  
  usb_buffer[0] = counter;
  int8_t ret_val = RawHID.send(usb_buffer, timeout);

  delay(100);
}
