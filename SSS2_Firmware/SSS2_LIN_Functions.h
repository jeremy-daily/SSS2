
elapsedMicros LINtimer;
int LINbaud = 19200;
uint32_t LINbitTime = 52;
uint16_t LINsyncPause = 13*LINbitTime; //microseconds
bool LINbreak;
bool readyForLINsync;
bool readyForLINid;
bool readyForLINdata;
bool LINfinished;
bool readyForLINchecksum;
bool readyToTransmitShifter;
uint8_t LINindex=0;
uint8_t LINbuffer[8];
uint8_t LINchecksum;
uint8_t LINaddress;
uint8_t LINlength;
uint8_t LIN_ID;
uint8_t outByte[256]; 
uint8_t kounter;
uint8_t checksumValue;
bool sendLIN = true;
bool printLIN;

void displayLIN(){
  if (commandString.toInt() > 0){
    printLIN = true;
    status_buffer_2[NET_STATUS_LOC] |= SEND_LIN_MASK; 
  }
  else {
    printLIN  = false;
    status_buffer_2[NET_STATUS_LOC] &= ~SEND_LIN_MASK;
  }
}

void sendLINselect(){
  if (commandString.toInt() > 0){
    sendLIN = true;    
    status_buffer_2[NET_STATUS_LOC] |= SUPPRESS_LIN_MASK;
  }
  else {
    sendLIN  = false;
    status_buffer_2[NET_STATUS_LOC] &= ~SUPPRESS_LIN_MASK;
  }
}

void determineSync(){
  if(LINtimer > LINsyncPause) {
    detachInterrupt(linRXpin);
    LIN.begin(LINbaud,SERIAL_8N1);
    readyForLINsync=true;
  }
}

void resetLINtimer(){
  LINtimer = 0;
  attachInterrupt(linRXpin,determineSync,RISING);
}

void sendLINResponse(){
  if( LIN.available() ){
    if (readyForLINsync){
      readyForLINsync=false;
      uint8_t LIN_Sync = 0; 
      while (LIN_Sync != 0x55 && LINtimer < 4000){
        LIN_Sync = LIN.read();
      }
      if (LIN_Sync == 0x55){
        readyForLINid = true;
        //if (printLIN) Serial.printf("LIN Sync: %02X\n",LIN_Sync);
      }
      else{
        LINfinished = true;
        //if (printLIN) Serial.printf("ERROR: LIN Sync: %02X\n",LIN_Sync);
      }
    }
    else if (readyForLINid ){
      LIN_ID = LIN.read();
      readyForLINdata = true;
      readyForLINid = false;
      LINindex = 0;
       
      bool ID0 = (LIN_ID & 0b00000001) >> 0;
      bool ID1 = (LIN_ID & 0b00000010) >> 1; 
      bool ID2 = (LIN_ID & 0b00000100) >> 2;
      bool ID3 = (LIN_ID & 0b00001000) >> 3; 
      bool ID4 = (LIN_ID & 0b00010000) >> 4; 
      bool ID5 = (LIN_ID & 0b00100000) >> 5; 
      bool LINparity0 = (LIN_ID & 0b01000000) >> 6;
      bool LINparity1 = (LIN_ID & 0b10000000) >> 7;
      LINaddress = (LIN_ID & 0x0F) >> 0;

      if (ID4 && ID5) LINlength = 8;
      else if (!ID4 && ID5) LINlength = 4;
      else if (ID4 && !ID5) LINlength = 2;
      else if (!ID4 && !ID5) LINlength = 2;
      
      if (LIN_ID == 0x20){
        outByte[0] = 0x14;
        outByte[1] = (kounter << 4) + 0x01;
        kounter++;
        outByte[2] = 0x02;
        outByte[3] = 0x0D;
        int calculatedChecksum = 0;
        calculatedChecksum += 0x20;
        for (int i = 0; i<4;i++){
          calculatedChecksum +=outByte[i];
          if (sendLIN) LIN.write(outByte[i]);
        }
        outByte[4] = uint8_t(~(calculatedChecksum % 255) );
        if (sendLIN) LIN.write(outByte[4]);
      }
      LINfinished = true; 
    }
    else if (readyForLINdata){
      
      LINbuffer[LINindex] = LIN.read();
      LINindex++;
      if (LINindex == LINlength){
        readyForLINdata = false;
        readyForLINchecksum = true;
         LINindex = 0;
      }
    }
    else if (readyForLINchecksum ){
      LINchecksum = LIN.read();
      int calculatedChecksum = 0;
      calculatedChecksum += LIN_ID & 0x3F;
      //the inverted module-256 checksum
      for (LINindex = 0; LINindex < LINlength; LINindex++){
        calculatedChecksum += LINbuffer[LINindex];
      }
      checksumValue = uint8_t(~(calculatedChecksum % 255) );
     
      readyForLINchecksum = false;
      LINfinished = true;
      if (printLIN) Serial.printf("LIN %lu,%02X,",micros(),LIN_ID);
      for (LINindex = 0; LINindex < LINlength; LINindex++){
        if (printLIN) Serial.printf("%02X,",LINbuffer[LINindex]);
      }
      if (printLIN) Serial.printf("%02X,%02X\n",LINchecksum,checksumValue); 
    }
    else {
      if (printLIN) Serial.printf("LIN -1,%02X\n",LIN.read());
      else LIN.read();
    }
  }

  //Reset the LIN responder after 20 ms of no messages
  if( LINtimer > 50000) LINfinished = true;
   
  if (LINfinished && LINtimer > 9000){
    pinMode(linRXpin,INPUT);
    attachInterrupt(linRXpin,resetLINtimer,FALLING);
    LINfinished = false;
    readyForLINchecksum = false;
    readyForLINdata = false;
    readyForLINsync = false;
    readyForLINid = false;
  } 
}
