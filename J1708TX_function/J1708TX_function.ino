/*
 * J1708 Transmit Example
 * Sends a request for the Component ID (MAKE*MODEL*SERIAL) 
 * using PID 243 from J1587
 * 
 * Tested with a DDEC4 and Bendix EC-60 
 */

/*                         Pin Defintions                       */
const int ignitionCtlPin    = 53; //Use this if connected to a relay to toggle ignition key

elapsedMicros J1708RXtimer; //Set up a timer to run after each byte is received.
elapsedMillis requestTimer; //Set up a time to send request messages.


boolean greenLEDstate = false;
boolean redLEDstate = true;
boolean LEDstate = true;
boolean ignitionCtlState = false;

uint8_t J1708RXbuffer[256];
uint8_t J1708TXbuffer[256];
uint8_t J1708ByteCount;
uint8_t J1708RXFrameLength = 0;
uint8_t J1708TXFrameLength = 0;
uint8_t J1708Checksum = 0;
const uint8_t componentIDrequest[3]={0xac,0,243};
char hexDisp[4];

uint8_t MID;
uint8_t PID;



uint8_t J1708RXmessage() {
  if (Serial3.available()){
    J1708RXtimer = 0; //Reset the RX message timer for J1708 message framing
    if (J1708ByteCount < sizeof(J1708RXbuffer)){ //Ensure the RX buffer can handle the new messages
       J1708RXbuffer[J1708ByteCount] = Serial3.read();
       J1708Checksum += J1708RXbuffer[J1708ByteCount];
       J1708ByteCount++; // Increment the recieved byte counts
    
    }
    else{
      //This is what we do if we don't have room in the RX buffer.
      Serial.println("J1708 Buffer Overflow");
      Serial3.clear();
      J1708ByteCount = 0;
    }
  }

  //Check to see if a message has been framed?
  if (J1708RXtimer > 2190 && J1708ByteCount > 0){ //21 bit times is 2.19 ms
    J1708RXFrameLength = J1708ByteCount;
    J1708ByteCount = 0; //reset the counter
    
    J1708Checksum -= J1708RXbuffer[J1708RXFrameLength]; //remove the received Checksum byte (last one)
    J1708Checksum = (~J1708Checksum + 1) & 0xFF; // finish calculating the checksum
    boolean J1708ChecksumOK = J1708Checksum == J1708RXbuffer[J1708RXFrameLength];
    J1708Checksum = 0;  
    if (J1708ChecksumOK) {
      return J1708RXFrameLength;
    }
    else {
      Serial.println ("Checksum failed, Message has errors.");
      return 0; //data would not be valid, so pretend it didn't come
    }
  }
  else
    return 0; //A message isn't ready yet.
}



uint8_t J1708TXmessage() {
  if (J1708RXtimer > 2190 && !Serial3.available()){
    J1708RXtimer = 0; //Reset the RX message timer for J1708 message framing
    uint8_t J1708Checksum = 0;
    for (uint8_t i = 0; i < J1708TXFrameLength; i++){
      J1708Checksum += J1708TXbuffer[i];
    }
    J1708Checksum = (~J1708Checksum + 1) & 0xFF;
    J1708TXbuffer[J1708TXFrameLength] = J1708Checksum;
    Serial3.write(J1708TXbuffer,J1708TXFrameLength+1);
    return J1708TXFrameLength+1;
  }
  else {
    return 0;
  }
}



void setup() {
  //This is needed for testing with an ECM
  pinMode(ignitionCtlPin,    OUTPUT);
  digitalWrite(ignitionCtlPin,HIGH);

  //The J1708 tranceiver is connected to UART3
  Serial3.begin(9600);
}



void loop() {
  // put your main code here, to run repeatedly:
  if (requestTimer >= 1000){
    requestTimer = 0;
    J1708TXFrameLength = 3;
    memcpy(J1708TXbuffer,componentIDrequest,J1708TXFrameLength );
    Serial.print(micros());
    Serial.print(" Sent ");
    Serial.print(J1708TXmessage()); 
    Serial.println(" bytes."); 
  }
  
  if ( J1708RXmessage() > 0 ){ //Execute this if the number of recieved bytes is more than zero.
    MID = J1708RXbuffer[0];
    PID = J1708RXbuffer[1];
    Serial.print(micros());
    Serial.print(" ");
    for (int i = 0; i < J1708RXFrameLength - 1; i++){ //Last byte is checksum and doesn't need to be printed
       if (PID == 243 && i > 3) Serial.write(J1708RXbuffer[i]);
       else{
         sprintf(hexDisp,"%02X ",J1708RXbuffer[i]);
         Serial.print(hexDisp);
       }
    }
    Serial.println();
  }  
}



