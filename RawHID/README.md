# RawHID for SSS2
This is a development directory to use the human interface device for communication with the PC from the SSS2.


Make teh RAWHID block in the usb_desc.h in the arduino directory look like this:
```
#elif defined(USB_RAWHID)
  #define VENDOR_ID  0x16C0 // 0x0525 //DG Tech
  #define PRODUCT_ID    0x0486
  #define RAWHID_USAGE_PAGE 0xFFAB  // recommended: 0xFF00 to 0xFFFF
  #define RAWHID_USAGE    0x0270  // recommended: 0x0100 to 0xFFFF
  #define MANUFACTURER_NAME {'S','y','n','e','r','c','o','n',' ','T','e','c','h','.'}
  #define MANUFACTURER_NAME_LEN 14
  #define PRODUCT_NAME    {'S','m','a','r','t',' ','S','e','n','s','o','r',' ','S','i','m',' ','2'}
  #define PRODUCT_NAME_LEN  18
  #define EP0_SIZE		64
  #define NUM_ENDPOINTS         4
  #define NUM_USB_BUFFERS	12
  #define NUM_INTERFACE		2
  #define RAWHID_INTERFACE      0	// RawHID
  #define RAWHID_TX_ENDPOINT    3
  #define RAWHID_TX_SIZE        64
  #define RAWHID_TX_INTERVAL    1
  #define RAWHID_RX_ENDPOINT    4
  #define RAWHID_RX_SIZE        64
  #define RAWHID_RX_INTERVAL    1
  #define SEREMU_INTERFACE      1	// Serial emulation
  #define SEREMU_TX_ENDPOINT    1
  #define SEREMU_TX_SIZE        64
  #define SEREMU_TX_INTERVAL    1
  #define SEREMU_RX_ENDPOINT    2
  #define SEREMU_RX_SIZE        32
  #define SEREMU_RX_INTERVAL    2
  #define ENDPOINT1_CONFIG	ENDPOINT_TRANSIMIT_ONLY
  #define ENDPOINT2_CONFIG	ENDPOINT_RECEIVE_ONLY
  #define ENDPOINT3_CONFIG	ENDPOINT_TRANSIMIT_ONLY
  #define ENDPOINT4_CONFIG	ENDPOINT_RECEIVE_ONLY
```