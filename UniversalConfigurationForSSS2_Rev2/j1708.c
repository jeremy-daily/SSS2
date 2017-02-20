/*
  Copyright Simma Software, Inc. - 2007

  j1708.c (Version 1.2)

  Implements SAE J1708 specification.
*/
#include <Arduino.h>
#include <inttypes.h>
#include "j1708.h"
//#include "bits.h"
//#include "hcs12d64.h"



/* NOTE: the next 2 macros need to read the UART's RX register and also clear
   the interrupt.  Check your datasheet to determine what is required to clear
   your RX interrupt and what is the proper order of macros in firmware.
   Change firmware as needed. */
/* PORT: read byte from UART */
#define J1708_HAL_RX_CHAR(val) {val = SCIDRL;}

/* PORT: clear RX interrupt flag */
#define J1708_HAL_RX_ISR_CLR(val) {val = SCISR1;}

/* PORT: transmit byte from UART */
#define J1708_HAL_TX_CHAR(val) {SCIDRL = val;}

/* PORT: read free running timer */
#define J1708_HAL_FRT_READ(val) {val = TCNT;}

/* PORT: clear ext int flag */
#define J1708_HAL_EXT_ISR_CLR() {TFLG1 = B2;}

/* PORT: lock critical code section */
#define J1708_LOCK() {DisableInterrupts;}

/* PORT: unlock critical code section */
#define J1708_UNLOCK() {EnableInterrupts;}

/* PORT: number of FRT cycles in 1.25 ms */
#define J1708_RX_IDLE_TIME               469

/* PORT: defines the number of transmit buffers to use */
#define J1708_TX_BUF_SIZE                  2 

/* PORT: defines the number of receive buffers to use */
#define J1708_RX_BUF_SIZE                  3 

/* PORT: set j1708_frt_t to the same size as the free running timer */
typedef uint16_t j1708_frt_t;

/* PORT: number of FRT cycles in X ms */
const j1708_frt_t
j1708_tx_idle_time[ 8 ] = {

   821, /* 21 bit times (2.19 ms) */ 
   899, /* 23 bit times (2.40 ms) */ 
   977, /* 25 bit times (2.61 ms) */ 
  1055, /* 27 bit times (2.82 ms) */ 
  1133, /* 29 bit times (3.03 ms) */ 
  1211, /* 31 bit times (3.23 ms) */ 
  1289, /* 33 bit times (3.44 ms) */ 
  1367  /* 35 bit times (3.65 ms) */ 
};



/* the longest J1708 message */
#define J1708_MAX_MSG_SIZE                21

/* bus state type:  IDLE is when the bus is idle, and NOT_IDLE is
   when the bus isn't idle */
typedef enum {

  J1708_BUS_IDLE = 0,
  J1708_BUS_NOT_IDLE

} j1708_bus_state_t;

/* transceiver state type:  TX is when the driver is transmitting, RX is when
   the driver is receiving, and NONE is when neither TX or RX is happening */
typedef enum {

  J1708_TCVR_NONE = 0,
  J1708_TCVR_TX,
  J1708_TCVR_RX

} j1708_tcvr_state_t;

/* transmit buffer type: these make up the transmit buffer */ 
typedef struct {

  volatile uint8_t buf[J1708_MAX_MSG_SIZE];
  volatile uint8_t size;
  volatile uint8_t pri;

} j1708_tx_t;

/* receive buffer type: these make up the receive buffer */ 
typedef struct {

  volatile uint8_t buf[J1708_MAX_MSG_SIZE];
  volatile uint8_t size;

} j1708_rx_t;


/* transmit buffers */
volatile j1708_tx_t j1708_tbuf[J1708_TX_BUF_SIZE];

/* transmit buffer helpers. how many buffers are used and start/end indexes */
volatile uint8_t j1708_tbuf_cnt = 0, j1708_tbuf_head = 0, j1708_tbuf_tail = 0;

/* current transmit count */
volatile uint8_t j1708_tx_cnt = 0;

/* receive buffers */
volatile j1708_rx_t j1708_rbuf[J1708_RX_BUF_SIZE];

/* receive buffer helpers. how many buffers are used and start/end indexes */
volatile uint8_t j1708_rbuf_cnt = 0, j1708_rbuf_head = 0, j1708_rbuf_tail = 0;

/* current receive count */
volatile uint8_t j1708_rx_cnt = 0;

/* state variables */
volatile j1708_tcvr_state_t j1708_tcvr_state = J1708_TCVR_NONE;
volatile j1708_bus_state_t j1708_bus_state = J1708_BUS_NOT_IDLE;

/* tracks how much time has elapsed since the bus was active (i.e. ext isr) */
volatile j1708_frt_t j1708_time_since_last_bus_event = 0;
volatile j1708_frt_t j1708_time_of_last_calc = 0;

/* ANSI C NOP: used to insert instructions between enable/disable ISR macros */
volatile uint8_t j1708_delay = 0;



uint8_t
j1708_checksum( uint8_t *buf, uint8_t size ) {

  uint8_t ret = 0;

  while( size-- )
    ret += buf[size];

  return ((~ret)+1);
}



void
j1708_init ( void ) {

  j1708_frt_t frt;
  uint8_t cnt;

  for( cnt = 0; cnt < J1708_TX_BUF_SIZE; cnt++ )
    j1708_tbuf[cnt].size = 0;

  for( cnt = 0; cnt < J1708_RX_BUF_SIZE; cnt++ )
    j1708_rbuf[cnt].size = 0;

  /* it's up to the module integrator where the following actually happens */
  /* PORT: configure UART and UART pins for operation */
  /* PORT: enable RX and external interrupts */

  J1708_HAL_FRT_READ(frt);
  j1708_time_of_last_calc = frt;

  return;
}



/* this function tracks how long it has been since there was activity on the
   bus.  it's also responsible for detecting when the receive process is over
   and marking the particular buffer as full */
void
j1708_process_rx_message ( void ) {

  j1708_frt_t frt, diff; 

  /* here we calculate the time since the last bus event */
  J1708_HAL_FRT_READ(frt);

  diff = frt - j1708_time_of_last_calc;
  j1708_time_of_last_calc = frt;

  /* if the calculation overflows, then set it to max, else do calc */
  if( (j1708_time_since_last_bus_event + diff) < diff )
    j1708_time_since_last_bus_event = ~0;
  else
    j1708_time_since_last_bus_event += diff;

  /* need to check if this is an end of a received message.  if we're in receive
     state & 12 bit times elapsed since the last byte, we consider it the end */
  if( j1708_tcvr_state == J1708_TCVR_RX ) {

    if( j1708_time_since_last_bus_event > J1708_RX_IDLE_TIME ) {

      /* mark buffer entry as used */
      j1708_rbuf[j1708_rbuf_head].size = j1708_rx_cnt;

      /* update the rx buffer count and head pointer */
      if( j1708_rbuf_cnt < J1708_RX_BUF_SIZE ) {

        j1708_rbuf_cnt++;

        if( ++j1708_rbuf_head >= J1708_RX_BUF_SIZE )
          j1708_rbuf_head = 0;
      }

      j1708_tcvr_state = J1708_TCVR_NONE;
    }
  }
}



/* this interrupt service routine handles the external interrupt input.  it
   first checks if a new message came in.  next, if the transceiver isn't busy
   this code places it in the the receive state (if we're not transmitting)
   because this activity means  data is coming in */
/* PORT: define j1708_bus_active_isr() to be an interrupt */
void
j1708_bus_active_isr ( void ) {

  j1708_frt_t frt;

  J1708_HAL_FRT_READ(frt);
 
  /* j1708_update might not have been called yet
     and we need to process any new received message */  
  j1708_process_rx_message();

  /* if the transceiver isn't doing anything, then we know this activity isn't
     from a transmission of ours.  therefore we move into the receive state */ 
  if( j1708_tcvr_state == J1708_TCVR_NONE ) {

    /* we only move into the RX state if there is room in the buffer */
    if( j1708_rbuf_cnt < J1708_RX_BUF_SIZE ) {

      j1708_rx_cnt = 0;
      j1708_tcvr_state = J1708_TCVR_RX;
    }
  } 

  j1708_time_since_last_bus_event = 0;
  j1708_time_of_last_calc = frt;
  j1708_bus_state = J1708_BUS_NOT_IDLE;

  /* we're all done with this interrupt, so clear the flag */
  J1708_HAL_EXT_ISR_CLR();

  return;
}


/* this function first checks to see if a new message has come in.  this has to
   be done here as well as in bus_active_isr because bus_active_isr only is
   called if data is being transfered.  if the bus is idle, this function is
   the one that will process any new message that might have just come in.
   this function also keeps track of the bus idle timing and updates the states.
   lastly, this function will start a transmission if there is a message to
   transmit and the bus is idle */
void
j1708_update ( void ) {

  uint8_t pri;
  j1708_frt_t frt;

  J1708_HAL_FRT_READ(frt);

  /* check to see if a new message has come in */
  J1708_LOCK();
  j1708_process_rx_message();
  J1708_UNLOCK();

  /* some CPUs do not immediately enable interrupts.  this delay should insure
     the following "LOCK" statement doesn't happen before interrupts are
     enabled.  this technique reduces the interrupt latency by insuring
     interrupts can execute between the UNLOCK and LOCK calls */
  for( j1708_delay = 0; j1708_delay < 1; j1708_delay++ );

  /* check to see if the bus is now idle */
  J1708_LOCK();
  if( j1708_time_since_last_bus_event > J1708_RX_IDLE_TIME ) {

    if( j1708_bus_state == J1708_BUS_NOT_IDLE ) {

      j1708_bus_state = J1708_BUS_IDLE;
      j1708_tcvr_state = J1708_TCVR_NONE;
    }
  }
  J1708_UNLOCK();

  /* see above */
  for( j1708_delay = 0; j1708_delay < 1; j1708_delay++ );

  /* check to see if we should transmit a message */
  J1708_LOCK();
  if( (j1708_bus_state == J1708_BUS_IDLE) &&
      (j1708_tcvr_state == J1708_TCVR_NONE) ) {

    /* valid range of priority is 1 - 8.  reduce it by 1 to make
       it aling with the array indexes of 0 - 7 */
    pri = j1708_tbuf[j1708_tbuf_tail].pri - 1;
    if( pri > 7 )
      pri = 7;

    if( j1708_time_since_last_bus_event >= j1708_tx_idle_time[pri] ) {

      if( j1708_tbuf_cnt ) {

        /* transmit a message */
        j1708_tcvr_state = J1708_TCVR_TX;
        j1708_bus_state = J1708_BUS_NOT_IDLE;
        j1708_time_since_last_bus_event = 0;
        j1708_time_of_last_calc = frt;
        j1708_tx_cnt = 0;
        J1708_HAL_TX_CHAR( j1708_tbuf[j1708_tbuf_tail].buf[0] );
      }
    }
  }
  J1708_UNLOCK();

  return;
}



/* this function buffers any incoming data and also transmits messages
   once they have been started by j1708_update() */
/* PORT: define j1708_rx_isr() to be an interrupt */
void
j1708_rx_isr ( void ) {

  uint8_t rx_val, tmp;

  /* clear receive interrupt flag */
  J1708_HAL_RX_ISR_CLR(tmp);

  /* read character from UART */
  J1708_HAL_RX_CHAR(rx_val);

  if( j1708_tcvr_state == J1708_TCVR_TX ) {

    /* does what we just received match what we just transmitted?
       if it does, then we are transmitting fine */
    if( j1708_tbuf[j1708_tbuf_tail].buf[j1708_tx_cnt] == rx_val ) {

      if( (++j1708_tx_cnt < j1708_tbuf[j1708_tbuf_tail].size) &&
          (j1708_tx_cnt < J1708_MAX_MSG_SIZE) ) {

        J1708_HAL_TX_CHAR( j1708_tbuf[j1708_tbuf_tail].buf[j1708_tx_cnt] );

      } else {

        /* that message is done, so note buffer is one less */
        if( j1708_tbuf_cnt )
          j1708_tbuf_cnt--;

        j1708_tcvr_state = J1708_TCVR_NONE;

        /* find next location */
        if( ++j1708_tbuf_tail >= J1708_TX_BUF_SIZE )
          j1708_tbuf_tail = 0;
      }

    } else {

      /* since we failed to get the bus, don't change the buf size
         so that it will automatically retry */
      j1708_tcvr_state = J1708_TCVR_NONE;
    }
  }

  if( j1708_tcvr_state == J1708_TCVR_RX ) {

    if( j1708_rx_cnt < J1708_MAX_MSG_SIZE )
      j1708_rbuf[j1708_rbuf_head].buf[j1708_rx_cnt++] = rx_val;
    else
      j1708_tcvr_state = J1708_TCVR_NONE;
  }

  return; 
}



/* this function should be called by an application layer function to transmit
   a j1708 message.  it will calculate and add the checksum.  this function
   only places the message into the j1708 transmit buffer.  it's the
   responsibility of j1708_update to actually start the transmission */
uint8_t
j1708_tx ( uint8_t *buf, uint8_t size, uint8_t pri ) {

  uint8_t cnt;

  /* we reject this message if there is no buffer available
     or if priority is invalid */
  if( (j1708_tbuf_cnt >= J1708_TX_BUF_SIZE) || (pri > 8) || (pri == 0) )
    return 0;

  /* copy incoming message into the J1708 transmit buffer */
  for( cnt = 0; cnt < size; cnt++ )
    j1708_tbuf[j1708_tbuf_head].buf[cnt] = buf[cnt];

  /* calculate checksum and copy over into j1708 tx buffer */
  j1708_tbuf[j1708_tbuf_head].buf[size] = j1708_checksum( buf, size );

  /* add one for checksum */
  j1708_tbuf[j1708_tbuf_head].size = size + 1;

  /* keep track of priority */
  j1708_tbuf[j1708_tbuf_head].pri = pri;

  /* find next location */
  if( ++j1708_tbuf_head >= J1708_TX_BUF_SIZE )
    j1708_tbuf_head = 0;

  /* indicate there is a buffer used */
  J1708_LOCK();
  j1708_tbuf_cnt++;
  J1708_UNLOCK();

  /* this function will start a transmission if there is data to go out */
  j1708_update();

  return 1;
}



/* this function returns a message from the receive buffer.  it returns the
   size of the message if one is availalbe, or 0 if no message. */
uint8_t
j1708_rx ( uint8_t *buf ) {

  uint8_t cnt, lsize, failure = 0;

  /* update might not have been called yet, so call it now
     because there might be a received message waiting */ 
  j1708_update();

  /* if nothing is available, then return 0 */
  if( j1708_rbuf_cnt == 0 )
    return 0;

  /* we're not going to pass back the checksum byte */
  lsize = j1708_rbuf[j1708_rbuf_tail].size - 1;

  /* do a sanity check on message size */
  if( lsize > (J1708_MAX_MSG_SIZE-1) )
    failure = 1;

  /* if size is valid, then check the checksum */
  if( failure == 0 )
    if( j1708_rbuf[j1708_rbuf_tail].buf[lsize] !=
        j1708_checksum((uint8_t *)j1708_rbuf[j1708_rbuf_tail].buf, lsize) )
      failure = 1;

  /* we only pass back the data if everything was ok */
  if( failure == 0 ) 
    for( cnt = 0; cnt < lsize; cnt++ )
      buf[cnt] = j1708_rbuf[j1708_rbuf_tail].buf[cnt];

  /* move onto next receive location */
  if( ++j1708_rbuf_tail >= J1708_RX_BUF_SIZE ) 
    j1708_rbuf_tail = 0;

  /* one less in the buffer */
  J1708_LOCK();
  j1708_rbuf_cnt--;
  J1708_UNLOCK();

  if( failure )
    lsize = 0;

  return lsize;
}
