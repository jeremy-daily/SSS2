
void setup() {
  // put your setup code here, to run once:
  pinMode(2,OUTPUT);
  pinMode(24,INPUT_PULLUP);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(2,!digitalRead(24));
}
