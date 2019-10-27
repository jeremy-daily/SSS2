#define USB_FRAME_TYPE_MASK 0xF0
#define USB_FRAME_TYPE_LOC     0
#define STATUS_TYPE         0x00
#define COMMAND_TYPE        0x10
#define MESSAGE_TYPE        0x20
#define ESCAPE_TYPE         0x80




#define U1U2P0ASWITCH_LOC      33
#define U1U2P0ASWITCH_MASK   0x01
#define U3U4P0ASWITCH_MASK   0x02
#define U5U6P0ASWITCH_MASK   0x04
#define U7U8P0ASWITCH_MASK   0x08
#define U9U10P0ASWITCH_MASK  0x10
#define U11U12P0ASWITCH_MASK 0x20
#define U13U14P0ASWITCH_MASK 0x40
#define U15U16P0ASWITCH_MASK 0x80

#define TERMINATION_SETTINGS_LOC   52
#define PWM_SETTINGS_LOC           47
#define CONFIG_SWITCH_SETTINGS_LOC 33

#define U34_I2C_ADDR 0x3C
#define U36_I2C_ADDR 0x3F
#define U37_I2C_ADDR 0x3D

#define U34_WIPER_LOC 49
#define U36_WIPER_LOC 50
#define U37_WIPER_LOC 51

#define U34_TCON_LOC 17
#define U36_TCON_LOC 18
#define U37_TCON_LOC 19

#define VOUT2_ADDR  0x49

#define CAN0_RX_COUNT_LOC 29
#define CAN1_RX_COUNT_LOC 33
#define CAN2_RX_COUNT_LOC 37
#define CAN0_TX_COUNT_LOC 41
#define CAN1_TX_COUNT_LOC 45
#define CAN2_TX_COUNT_LOC 49
#define J1708_RX_COUNT_LOC 53
#define LIN_RX_COUNT_LOC 55
#define NET_STATUS_LOC 59
#define SERIAL_NUM_LOC 60
#define CRC_LOC 62

#define CAN0_BAUD_LOC 20
#define CAN1_BAUD_LOC 21
#define CAN2_BAUD_LOC 22

#define ANALOG_OUT_START_LOC 11

#define HBRIDGE_LOC 60
#define IGNITION_RELAY_MASK 0x80
#define TWELVE_OUT_1_MASK   0x01
#define TWELVE_OUT_2_MASK   0x02
#define GROUND_OUT_1_MASK   0x04
#define GROUND_OUT_2_MASK   0x08


#define MCP23017_ADDR 0x20

#define MCP23017_IODIRA   0x00      ///< Controls the direction of the data I/O for port A.
#define MCP23017_IODIRB   0x01      ///< Controls the direction of the data I/O for port B.
#define MCP23017_IPOLA    0x02      ///< Configures the polarity on the corresponding GPIO port bits for port A.
#define MCP23017_IPOLB    0x03      ///< Configures the polarity on the corresponding GPIO port bits for port B.
#define MCP23017_GPINTENA 0x04      ///< Controls the interrupt-on-change for each pin of port A.
#define MCP23017_GPINTENB 0x05      ///< Controls the interrupt-on-change for each pin of port B.
#define MCP23017_DEFVALA  0x06      ///< Controls the default comparaison value for interrupt-on-change for port A.
#define MCP23017_DEFVALB  0x07      ///< Controls the default comparaison value for interrupt-on-change for port B.
#define MCP23017_INTCONA  0x08      ///< Controls how the associated pin value is compared for the interrupt-on-change for port A.
#define MCP23017_INTCONB  0x09      ///< Controls how the associated pin value is compared for the interrupt-on-change for port B.
#define MCP23017_IOCON    0x0A      ///< Controls the device.
#define MCP23017_GPPUA    0x0C      ///< Controls the pull-up resistors for the port A pins.
#define MCP23017_GPPUB    0x0D      ///< Controls the pull-up resistors for the port B pins.
#define MCP23017_INTFA    0x0E      ///< Reflects the interrupt condition on the port A pins.
#define MCP23017_INTFB    0x0F      ///< Reflects the interrupt condition on the port B pins.
#define MCP23017_INTCAPA  0x10      ///< Captures the port A value at the time the interrupt occured.
#define MCP23017_INTCAPB  0x11      ///< Captures the port B value at the time the interrupt occured.
#define MCP23017_GPIOA    0x12      ///< Reflects the value on the port A.
#define MCP23017_GPIOB    0x13      ///< Reflects the value on the port B.
#define MCP23017_OLATA    0x14      ///< Provides access to the port A output latches.
#define MCP23017_OLATB    0x15      ///< Provides access to the port B output latches.

#define PWM1_LOC  35
#define PWM2_LOC  37
#define PWM3_LOC  39
#define PWM4_LOC  41
#define PWM5_LOC  43
#define PWM6_LOC  45

#define PWM1_FREQ_LOC 54
#define PWM3_FREQ_LOC 56
#define PWM5_FREQ_LOC 58

#define HVADJOUT_LOC 48

#define CAN_MESSAGE_BASE 0x20
#define TIMESTAMP_OFFSET 0
#define CHANNEL_DLC_OFFSET 4
#define MICROSECONDS_OFFSET 5
#define CAN_ID_OFFSET 8
#define CAN_DATA_OFFSET 12
#define CAN_SEND_MS 3
#define CAN_FRAME_LENGTH 20

uint8_t timeout = 0;
