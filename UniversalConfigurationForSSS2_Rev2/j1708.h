#ifndef _J1708_H_
#define _J1708_H_

extern void
j1708_init( void );

extern void
j1708_update( void );

extern void
j1708_rx_isr( void );

extern void
j1708_bus_active_isr( void );

extern uint8_t
j1708_rx ( uint8_t *buf );

extern uint8_t
j1708_tx ( uint8_t *buf, uint8_t size, uint8_t pri );

#endif
