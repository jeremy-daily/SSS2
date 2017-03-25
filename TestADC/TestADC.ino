#include <SPI.h>

uint8_t out1;
uint8_t out2;

/****************************************************************/
/*                         Pin Defintions                       */
const uint8_t greenLEDpin       = 2;
const uint8_t redLEDpin         = 5;
const uint8_t CSdispPin         = 9;
const uint8_t CSCANPin          = 15;
const uint8_t PWM1Pin           = 16;
const uint8_t PWM2Pin           = 17;
const uint8_t CSanalogPin       = 20;
const uint8_t CStermPin         = 21;
const uint8_t PWM3Pin           = 22;
const uint8_t PWM4Pin           = 23;
const uint8_t buttonPin         = 24;
const uint8_t CStouchPin        = 26;
const uint8_t IH1Pin            = 35;
const uint8_t IH2Pin            = 36;
const uint8_t IL1Pin            = 37;
const uint8_t IL2Pin            = 38;
const uint8_t ignitionCtlPin    = 39;

void setPinModes(){
    pinMode(greenLEDpin, OUTPUT);
    pinMode(redLEDpin, OUTPUT);
    pinMode(CSdispPin, OUTPUT);
    pinMode(CSCANPin, OUTPUT);
    pinMode(PWM1Pin, OUTPUT);
    pinMode(PWM2Pin, OUTPUT);
    pinMode(CSanalogPin, OUTPUT);
    pinMode(CStermPin, OUTPUT);
    pinMode(PWM3Pin, OUTPUT);
    pinMode(PWM4Pin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(CStouchPin, OUTPUT);
    pinMode(IH1Pin, OUTPUT);
    pinMode(IH2Pin, OUTPUT);
    pinMode(IL1Pin, OUTPUT);
    pinMode(IL2Pin, OUTPUT);
    pinMode(ignitionCtlPin, OUTPUT);
    
    digitalWrite(CSCANPin, HIGH);
    digitalWrite(CSdispPin, HIGH);
    digitalWrite(CStouchPin, HIGH);
    digitalWrite(CStermPin, HIGH);
    digitalWrite(greenLEDpin, HIGH);
    
}

void setup() {
  // put your setup code here, to run once:
  setPinModes();
  
  SPI.begin();
  while(!Serial);
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSanalogPin, LOW);
  //Write to Range Register 1 to Select the range for input channels
  out1 = SPI.transfer(0b10111111);
  out2 = SPI.transfer(0b11100000); //Write to ADC Device
  digitalWrite(CSanalogPin, HIGH);
  Serial.println(out2 | (out1 << 8),HEX);
  delay(1);

  digitalWrite(CSanalogPin, LOW);
  //Write to Range Register 2 to Select the range for input channels
  out1 = SPI.transfer(0b11011111);
  out2 = SPI.transfer(0b11100000); //Write to ADC Device
  digitalWrite(CSanalogPin, HIGH);
  Serial.println(out2 | (out1 << 8),HEX);
  delay(1);

  digitalWrite(CSanalogPin, LOW);
  //Write to Seq. Register. This register selects the channels for conversion. We want them all.
  //Bit 16 (MSB) = 1 (Write)
  //Bits 15,14 = 11 (Sequence Register)
  //Bits 13-6 = 11111111 (Set all to on)
  //Bits 5-1 = 0 (Not used according to the data sheet)
  out1 = SPI.transfer(0b11111111);
  out2 = SPI.transfer(0b11100000);
  digitalWrite(CSanalogPin, HIGH);
  Serial.println(out2 | (out1 << 8),HEX);
  delay(1);

  digitalWrite(CSanalogPin, LOW);
  //Write to control register to select the final channel in the seq. Set Seq1 =1  and Seq2=0
  //Write  RegSel1 RegSel2 ADD2 ADD1 ADD0 Mode1 Mode0 PM1 PM0 Coding Ref Seq1 Seq2 Zero Zero
  out1 = SPI.transfer(0b10011100);
  out2 = SPI.transfer(0b00101000);
  digitalWrite(CSanalogPin, HIGH);
  Serial.println(out2 | (out1 << 8),HEX);
  delay(1);
  SPI.endTransaction();

}

void loop() {
  // put your main code here, to run repeatedly:
  SPI.beginTransaction(SPISettings(1000000,MSBFIRST, SPI_MODE0));
  for (int i = 0; i<8;i++){
    uint8_t controlHighTemplate = 0b10000000;
    uint8_t controlLowTemplate =  0b00000000;
    
    digitalWrite(CSanalogPin,LOW);
    delay(1);
    uint8_t spiHighByte =  SPI.transfer(controlHighTemplate | (i<<2));
    uint8_t spiLowByte =  SPI.transfer(controlLowTemplate);
    int data = spiLowByte | (spiHighByte << 8);
    Serial.print(controlHighTemplate | (i<<2),BIN);
    Serial.print(", ");
    Serial.println(data,HEX);
    digitalWrite(CSanalogPin,HIGH);
    delay(1);
  }
  
  SPI.endTransaction();
  Serial.println();
  delay(1000);
}
