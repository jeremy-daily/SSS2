#define GREEN_PIN 2
#define RED_PIN 5

uint32_t counter = 0;
elapsedMillis milliTimer;

void setup() {
 pinMode(GREEN_PIN, OUTPUT);
 pinMode(RED_PIN, OUTPUT);
 digitalWrite(RED_PIN,HIGH);
 digitalWrite(GREEN_PIN,LOW);
} 

void loop() {
  // put your main code here, to run repeatedly:
  counter++;
  if (milliTimer >= 1){
    milliTimer = 0;
    Serial.println(counter);
    Serial.send_now();
  }
}
