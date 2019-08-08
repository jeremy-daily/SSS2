#include <i2c_t3.h>
#include <SPI.h>

#define PORTEXTENDER_ADDRESS 0x27
#define CONFIGEXTENDER_ADDRESS 0x23

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

#define MEM_LEN 256
uint8_t databuf[MEM_LEN];
uint8_t pin;
const uint16_t SINGLE_PIN[16] = {65534,65533,65531,65527,65519,65503,65471,65407,65279,65023,64511,63487,61439,57343,49151,32767};
uint8_t TCON_Value;
uint8_t TCON_values[20];
uint8_t current_potexpander_pin;
uint8_t result = 0xAA;

elapsedMillis request_timer;

uint8_t spi_status;
#define SPI_DONE 0
#define SEND_SPI 1


void requestDone(void)
{
    Wire.read(databuf, Wire.available());
    Serial.printf("%08b OK\n",databuf);
}
// Trigger on I2C Error
//
void errorEvent(void)
{
    Serial.print("FAIL - ");
    switch(Wire.status())
    {
    case I2C_TIMEOUT:  Serial.print("I2C timeout\n"); Wire.resetBus(); break;
    case I2C_ADDR_NAK: Serial.print("Slave addr not acknowledged\n"); break;
    case I2C_DATA_NAK: Serial.print("Slave data not acknowledged\n"); break;
    case I2C_ARB_LOST: Serial.print("Arbitration Lost, possible pullup problem\n"); Wire.resetBus(); break;
    case I2C_BUF_OVF:  Serial.print("I2C buffer overflow\n"); break;
    case I2C_NOT_ACQ:  Serial.print("Cannot acquire bus, possible stuck SDA/SCL\n"); Wire.resetBus(); break;
    case I2C_DMA_ERR:  Serial.print("DMA Error\n"); break;
    default:           break;
    }
}

void set_MCP23017_pin(uint8_t addr){
    Serial.println(SINGLE_PIN[current_potexpander_pin],BIN);
    Wire.beginTransmission(addr);
    Wire.write(MCP23017_GPIOA);
    Wire.write(lowByte(SINGLE_PIN[current_potexpander_pin]));
    Wire.write(highByte(SINGLE_PIN[current_potexpander_pin]));
    Wire.sendTransmission();
    //Wire.finish(5000);
    Serial.println("finished set_MCP23017_pin");
}

void clear_MCP23017_pin(uint8_t addr){
    Wire.beginTransmission(addr);
    Wire.write(MCP23017_GPIOA);
    Wire.write(0xFF);
    Wire.write(0xFF);
    Wire.sendTransmission();
    //Wire.finish(5000);
    Serial.println("finished clear_MCP23017_pin");
}

void transmitSPI(void)
{ 
  
  if (spi_status == SEND_SPI){ 
    Serial.println("Starting transmitSPI"); 
    SPI.transfer(0x40); //Write to TCON Register
    SPI.transfer(TCON_Value + 8);
    SPI.transfer(0x4C); //Read Command
    result = SPI.transfer(0x55); //Read Terminal Connection (TCON) Register
    spi_status = SPI_DONE;
    Serial.print("SPI Returns ");
    Serial.println(result);
    clear_MCP23017_pin(PORTEXTENDER_ADDRESS);
    TCON_values[current_potexpander_pin] = result & 0x07;
    
  }
}

uint8_t MCP41HVExtender_SetTerminals() {
  Wire.onTransmitDone(transmitSPI);
  spi_status = SEND_SPI;
  Serial.print("Calling set_MCP23017_pin for ");
  Serial.println(PORTEXTENDER_ADDRESS,HEX);
  set_MCP23017_pin(PORTEXTENDER_ADDRESS);
}

//uint8_t MCP41HVExtender_SetWiper(uint8_t pin, uint8_t potValue)
//{
//  PotExpander.writeGPIOAB(~(1 << pin));
//  delay(1);
//  SPI.transfer(0x00); //Write to wiper Register
//  SPI.transfer(potValue);
//  SPI.transfer(0x0C); //Read command
//  uint8_t result = SPI.transfer(0xFF); //Read Wiper Register
//  PotExpander.writeGPIOAB(0xFF);
//  return result;
//} 

/****************************************************************/
/*                         Pin Defintions                       */
const int8_t greenLEDpin       = 2;
const int8_t redLEDpin         = 5;
const int8_t CSCANPin          = 15;
const int8_t CSanalogPin       = 20;
const int8_t CStermPin         = 21;
const int8_t CSconfigAPin      = 21;
const int8_t CSconfigBPin      = -1;
const int8_t buttonPin         = 24;
const int8_t encoderAPin       = 28;
const int8_t encoderBPin       = 25;
const int8_t CStouchPin        = 26;
const int8_t IH1Pin            = 35;
const int8_t IH2Pin            = 36;
const int8_t IL1Pin            = 37;
const int8_t IL2Pin            = 38;
const int8_t ignitionCtlPin    = 39;
const int8_t linRXpin          = 0;
const uint8_t numPWMs = 4;
const int8_t PWMPins[numPWMs]     = {16,17,22,23};
uint16_t pwmValue[numPWMs] = {25,100,19,222};
uint16_t pwmFrequency[numPWMs] = {200,200,200,200};

const uint8_t numADCs = 1;
const int8_t analogInPins[6]= {A21,A22,A0,A2,A6,A11};

void setPinModes(){
    pinMode(A5, INPUT_PULLUP);
    pinMode(A4, INPUT_PULLUP);
    
    pinMode(greenLEDpin,     OUTPUT);
    pinMode(redLEDpin,       OUTPUT);
    pinMode(CSCANPin,        OUTPUT);
    pinMode(CSanalogPin,     OUTPUT);
    pinMode(CStermPin,       OUTPUT);
    pinMode(CSconfigAPin,    OUTPUT);
    pinMode(CSconfigBPin,    OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(CStouchPin,      OUTPUT);
    pinMode(IH1Pin,          OUTPUT);
    pinMode(IH2Pin,          OUTPUT);
    pinMode(IL1Pin,          OUTPUT);
    pinMode(IL2Pin,          OUTPUT);
    pinMode(ignitionCtlPin,  OUTPUT);
    pinMode(linRXpin,        INPUT);
     
    if (CSCANPin      > -1) digitalWrite(CSCANPin,     HIGH);
    if (CStouchPin    > -1) digitalWrite(CStouchPin,   HIGH);
    if (CStermPin     > -1) digitalWrite(CStermPin,    HIGH);
    if (CSconfigAPin  > -1) digitalWrite(CSconfigAPin, HIGH);
    if (CSconfigBPin  > -1) digitalWrite(CSconfigBPin, HIGH);
    if (redLEDpin     > -1) digitalWrite(redLEDpin,    HIGH);
    if (greenLEDpin   > -1) digitalWrite(greenLEDpin,  LOW);
    digitalWrite(CSanalogPin,HIGH);
    digitalWrite(CStouchPin,HIGH);
    digitalWrite(IH1Pin,LOW);
    digitalWrite(IH2Pin,LOW);
    digitalWrite(IL1Pin,LOW);
    digitalWrite(IL2Pin,LOW);
    digitalWrite(ignitionCtlPin,LOW);
    
    uint8_t i;
    for (i = 0; i < numPWMs; i++) pinMode(PWMPins[i], OUTPUT);
    for (i = 0; i < numADCs; i++) pinMode(analogInPins[i], INPUT);
    
}

void setup() {
  // put your setup code here, to run once:
  //while(!Serial);
  SPI.begin();
  setPinModes();

  //pinMode(18,INPUT_PULLUP);   
  //pinMode(19,INPUT_PULLUP);
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(10000); // 5ms

  //Send the direction of the port extender.
  Wire.beginTransmission(PORTEXTENDER_ADDRESS);
  Wire.write(MCP23017_IODIRA);
  Wire.write(0x00); // When the bit is clear, it is an output
  Wire.write(0x00); //Sequential byte moves to GPIOB
  Wire.endTransmission();
  Serial.println("Done Setting the direction of the port extender.");
  
  //Send Initial Value of High
  Wire.beginTransmission(PORTEXTENDER_ADDRESS);
  Wire.write(MCP23017_GPIOA);
  Wire.write(0xFF);
  Wire.write(0xFF); //Sequential byte moves to GPIOB
  Wire.endTransmission();
  Serial.println("Done setting all values high");
  
  Wire.onReqFromDone(requestDone);
  Wire.onError(errorEvent);
 
  TCON_Value = 2;
}



void loop() {
  // put your main code here, to run repeatedly:
  if (request_timer >= 100){
    request_timer = 0;
    Serial.println(current_potexpander_pin);
    if (Wire.done()) MCP41HVExtender_SetTerminals();
    
    current_potexpander_pin++;
    if (current_potexpander_pin >= 16) current_potexpander_pin = 0;
    
  }
  if (Wire.done()){
    digitalWrite(CSconfigAPin,LOW);
    SPI.transfer(current_potexpander_pin & 0xFF); 
    digitalWrite(CSconfigAPin,HIGH);
  }
}
