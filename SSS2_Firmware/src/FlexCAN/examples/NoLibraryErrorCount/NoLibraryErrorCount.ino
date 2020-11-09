/* FlexCAN module I/O Base Addresss */
typedef volatile uint32_t vuint32_t;
#define FLEXCAN0_BASE                        (0x40024000L)
#define FLEXCAN1_BASE                        (0x400A4000L)
#define FLEXCANb_MCR(base)                   (*(vuint32_t*)(base+0x00))
#define FLEXCANb_CTRL1(base)                 (*(vuint32_t*)(base+0x04))
#define FLEXCANb_ECR(base)                   (*(vuint32_t*)(base+0x1C))

#define FLEXCAN_get_RX_ERR_COUNTER(address)  (((address) & FLEXCAN_ECR_RX_ERR_COUNTER_MASK)>>8)
#define FLEXCAN_get_TX_ERR_COUNTER(address)  (((address) & FLEXCAN_ECR_TX_ERR_COUNTER_MASK)>>0)
#define FLEXCAN_ECR_RX_ERR_COUNTER_MASK      (0x0000FF00)
#define FLEXCAN_ECR_TX_ERR_COUNTER_MASK      (0x000000FF)


#define FLEXCAN_CTRL_CLK_SRC        (0x00002000)
#define FLEXCAN_MCR_SRX_DIS         (0x00020000)
#define FLEXCAN_MCR_LPM_ACK         (0x00100000)
#define FLEXCAN_MCR_FRZ_ACK         (0x01000000)
#define FLEXCAN_MCR_SOFT_RST        (0x02000000)
#define FLEXCAN_MCR_FRZ             (0x40000000)
#define FLEXCAN_MCR_MDIS            (0x80000000)


//variable for the address location for the CAN error counter
uint32_t can_error_counter_address;
uint8_t errorCount;

elapsedMillis printTimer;

void setup() {
  // put your setup code here, to run once:
  
  //Set the pins to use CAN0
  CORE_PIN3_CONFIG = PORT_PCR_MUX(2);
  CORE_PIN4_CONFIG = PORT_PCR_MUX(2);
  
  //Set the pins to use CAN1
  CORE_PIN33_CONFIG = PORT_PCR_MUX(2);
  CORE_PIN34_CONFIG = PORT_PCR_MUX(2);

  // select clock source 16MHz xtal
  OSC0_CR |= OSC_ERCLKEN;

  SIM_SCGC6 |=  SIM_SCGC6_FLEXCAN0;
  SIM_SCGC3 |=  SIM_SCGC3_FLEXCAN1;

  FLEXCANb_CTRL1(FLEXCAN0_BASE) &= ~FLEXCAN_CTRL_CLK_SRC;
  FLEXCANb_CTRL1(FLEXCAN1_BASE) &= ~FLEXCAN_CTRL_CLK_SRC;
  // enable CAN
  FLEXCANb_MCR (FLEXCAN0_BASE) |=  FLEXCAN_MCR_FRZ;
  FLEXCANb_MCR (FLEXCAN1_BASE) |=  FLEXCAN_MCR_FRZ;
  
  FLEXCANb_MCR (FLEXCAN0_BASE) &= ~FLEXCAN_MCR_MDIS;
  FLEXCANb_MCR (FLEXCAN1_BASE) &= ~FLEXCAN_MCR_MDIS;

  while (FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_LPM_ACK);
  while (FLEXCANb_MCR(FLEXCAN1_BASE) & FLEXCAN_MCR_LPM_ACK);

  // soft reset
  FLEXCANb_MCR(FLEXCAN0_BASE) ^=  FLEXCAN_MCR_SOFT_RST;
  FLEXCANb_MCR(FLEXCAN1_BASE) ^=  FLEXCAN_MCR_SOFT_RST;

  while (FLEXCANb_MCR (FLEXCAN0_BASE) & FLEXCAN_MCR_SOFT_RST);
  while (FLEXCANb_MCR (FLEXCAN1_BASE) & FLEXCAN_MCR_SOFT_RST);

  // wait for freeze ack
  while (!(FLEXCANb_MCR(FLEXCAN0_BASE) & FLEXCAN_MCR_FRZ_ACK));
  while (!(FLEXCANb_MCR(FLEXCAN1_BASE) & FLEXCAN_MCR_FRZ_ACK));

  // disable self-reception
  FLEXCANb_MCR (FLEXCAN0_BASE) |= FLEXCAN_MCR_SRX_DIS;
  FLEXCANb_MCR (FLEXCAN1_BASE) |= FLEXCAN_MCR_SRX_DIS;

}

void loop() {
  // put your main code here, to run repeatedly:
  if (printTimer > 1){
    printTimer = 0;     
    can_error_counter_address = FLEXCANb_ECR(FLEXCAN0_BASE);
    errorCount = FLEXCAN_get_RX_ERR_COUNTER(can_error_counter_address);
    Serial.print("Can0 RX Error Count: ");
    Serial.println(errorCount);
  }
    
}
