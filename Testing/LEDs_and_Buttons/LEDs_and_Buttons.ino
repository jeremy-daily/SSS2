const int8_t greenLEDpin       = 2;
const int8_t redLEDpin         = 5;
const int8_t buttonPin         = 24;
const int8_t encoderAPin       = 28;
const int8_t encoderBPin       = 25;

void setup() {
  // put your setup code here, to run once:
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  pinMode(greenLEDpin, OUTPUT);
  pinMode(redLEDpin, OUTPUT);

  digitalWrite(greenLEDpin,HIGH);
  digitalWrite(redLEDpin,HIGH);
  delay(1000);
  digitalWrite(greenLEDpin,HIGH);
  digitalWrite(redLEDpin,HIGH);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(buttonPin)){
    digitalWrite(greenLEDpin,digitalRead(encoderAPin)); 
    digitalWrite(redLEDpin,digitalRead(encoderBPin)); 
  }
  else
  {
    digitalWrite(greenLEDpin,HIGH); 
    digitalWrite(redLEDpin,HIGH);   
    }
}
