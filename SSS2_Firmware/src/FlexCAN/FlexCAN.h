// -------------------------------------------------------------
// a simple Arduino Teensy 3.1/3.2/3.6 CAN driver
// by teachop
// dual CAN support for MK66FX1M0 by Pawelsky
//
#ifndef __FLEXCAN_H__
#define __FLEXCAN_H__

#include <Arduino.h>

#define AUTOBAUD_TIMEOUT 150 //milliseconds to look for message
#define EEPROM_BIT_RATE_INDEX_ADDR 0 //Location in EEPROM for the bitrate index
#define NUM_BAUD_RATES 5
#define BAUD_RATE_LIST {250000, 500000, 125000, 666666, 1000000}

#define INCLUDE_FLEXCAN_DEBUG 0

#if(INCLUDE_FLEXCAN_DEBUG)
  #define dbg_print(fmt, args...)     Serial.print (fmt , ## args)
  #define dbg_println(fmt, args...)   Serial.println (fmt , ## args)
#else
  #define dbg_print(fmt, args...)
  #define dbg_println(fmt, args...)
#endif


#if !defined(SIZE_RX_BUFFER)
#define SIZE_RX_BUFFER  512 // receive incoming ring buffer size
#endif

#define USE_TX_BUFFER 0 //Set to 0 to just write directly to the mailboxes, which gives more predictable results.
#if !defined(SIZE_TX_BUFFER)
#define SIZE_TX_BUFFER  16 // transmit ring buffer size
#endif

#define SIZE_LISTENERS   4  // number of classes that can register as listeners on each CAN bus
#define NUM_MAILBOXES    16 // architecture specific but all Teensy 3.x boards have 16 mailboxes
#define NUM_TX_MAILBOXES 2
#define IRQ_PRIORITY     64 // 0 = highest, 255 = lowest
#define IRQ_LOW_PRIORITY 225 // 0 = highest, 255 = lowest

#define COLLECT_CAN_STATS 0


typedef struct CAN_message_t {
  uint32_t id;          // can identifier
  uint32_t micros;      // system microseconds
  uint32_t rxcount;      // number of received messages
  uint16_t timestamp;   // FlexCAN time when message arrived
  struct {
    uint8_t extended:1; // identifier is extended (29-bit)
    uint8_t remote:1;   // remote transmission request packet type
    uint8_t overrun:1;  // message overrun
    uint8_t reserved:5;
  } flags;
  uint8_t len;          // length of data
  uint8_t buf[8];
} CAN_message_t;

typedef struct CAN_filter_t {
  uint32_t id;
  struct {
    uint8_t extended:1;  // identifier is extended (29-bit)
    uint8_t remote:1;    // remote transmission request packet type
    uint8_t reserved:6;
  } flags;
} CAN_filter_t;

// statistics about the CAN interface

typedef struct CAN_stats_t {
  bool     enabled;           // enable collecting statistics
  uint32_t ringRxMax;         // number of entries in the ring buffer
  uint32_t ringRxHighWater;   // maximum entries used in the ring buffer
  uint32_t ringRxFramesLost;  // total number of frames lost
  uint32_t ringTxMax;         // number of entries in the ring buffer
  uint32_t ringTxHighWater;   // maximum entries used in the ring buffer
  struct {
    uint32_t refCount;        // mailbox reference (use) count
    uint32_t overrunCount;    // mailbox message overrun count
  } mb[NUM_MAILBOXES];
} CAN_stats_t;

// ring buffer data structure

typedef struct ringbuffer_t {
  volatile uint16_t head;
  volatile uint16_t tail;
  uint16_t size;
  volatile CAN_message_t *buffer;
} ringbuffer_t;

// for backwards compatibility with previous structure members

#define	ext flags.extended
#define	rtr flags.remote

class CANListener
{
public:
  CANListener ();

  virtual bool frameHandler (CAN_message_t &frame, int mailbox, uint8_t controller);

  void attachMBHandler (uint8_t mailBox);
  void detachMBHandler (uint8_t mailBox);
  void attachGeneralHandler (void);
  void detachGeneralHandler (void);

private:
  uint32_t callbacksActive; // bitfield indicating which callbacks are installed (for object oriented callbacks only)

  friend class FlexCAN;     // class has to have access to the the guts of this one
};

// -------------------------------------------------------------

class FlexCAN
{
private:
  bool autobaud;
  bool report_errors;
  uint8_t can_channel;
  uint8_t baud_rate_index;
  uint32_t baud_rates[NUM_BAUD_RATES] = BAUD_RATE_LIST;
  
  uint32_t flexcanBase;
  uint8_t eeprom_RATE_INDEX_ADDR;

  struct CAN_filter_t MBFilters[NUM_MAILBOXES];
  static struct CAN_filter_t defaultMask;
  void mailbox_int_handler (uint8_t mb, uint32_t ul_status);
  CANListener *listener[SIZE_LISTENERS];

  ringbuffer_t txRing;
  volatile CAN_message_t tx_buffer[SIZE_TX_BUFFER];
  ringbuffer_t rxRing;
  volatile CAN_message_t rx_buffer[SIZE_RX_BUFFER];

  void writeTxRegisters (const CAN_message_t &msg, uint8_t buffer);
  void readRxRegisters (CAN_message_t &msg, uint8_t buffer);

  void initRingBuffer (ringbuffer_t &ring, volatile CAN_message_t *buffer, uint32_t size);
  bool addToRingBuffer (ringbuffer_t &ring, const CAN_message_t &msg);
  bool removeFromRingBuffer (ringbuffer_t &ring, CAN_message_t &msg);
  bool isRingBufferEmpty (ringbuffer_t &ring);
  uint32_t ringBufferCount (ringbuffer_t &ring);

  void freezeMode(bool mode);
  bool set_baud_rate(uint32_t baud);
  uint32_t get_baud_rate(void);

#ifdef COLLECT_CAN_STATS
  CAN_stats_t stats;
#endif

protected:
  uint8_t numTxMailboxes;

public:
  FlexCAN (uint8_t id = 0);
  void begin (uint32_t baud = 250000, const CAN_filter_t &mask = defaultMask, uint8_t txAlt = 0, uint8_t rxAlt = 0);
  uint32_t baud_rate;
  void setFilter (const CAN_filter_t &filter, uint8_t n);
  bool getFilter (CAN_filter_t &filter, uint8_t n);
  void setMask (uint32_t mask, uint8_t n);
  void end (void);
  uint32_t available (void);
  int write (const CAN_message_t &msg);
  int read (CAN_message_t &msg);
  uint32_t rxBufferOverruns (void) { return stats.ringRxFramesLost; };

#ifdef COLLECT_CAN_STATS
  void startStats (void) { stats.enabled = true; };
  void stopStats (void) { stats.enabled = false; };
  void clearStats (void);
  CAN_stats_t getStats (void) { return stats; };
#endif

  void setListenOnly (bool mode); //pass true to go into listen only mode, false to be in normal mode
  void setSelfReception (bool mode); 
  void setReportErrors (bool mode); 

  bool attachObj (CANListener *listener);
  bool detachObj (CANListener *listener);

  //int watchFor(); //allow anything through
  //int watchFor(uint32_t id); //allow just this ID through (automatic determination of extended status)
  //int watchFor(uint32_t id, uint32_t mask); //allow a range of ids through
  //int watchForRange(uint32_t id1, uint32_t id2); //try to allow the range from id1 to id2 - automatically determine base ID and mask

  uint32_t setNumTxBoxes (uint32_t txboxes);

  void message_isr (void);
  void bus_off_isr (void);
  void error_isr (void);
  void tx_warn_isr (void);
  void rx_warn_isr (void);
  void wakeup_isr (void);

  uint8_t readTEC ();
  uint8_t readREC ();

};

extern FlexCAN Can0;
#ifdef __MK66FX1M0__
extern FlexCAN Can1;
#endif

#endif // __FLEXCAN_H__
