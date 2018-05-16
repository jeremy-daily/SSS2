#include "OneButton.h"
#define BUTTON_PIN 24
#define GREEN_PIN 2
#define RED_PIN 5
void setup() {
  // put your setup code here, to run once:
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN,HIGH);
  digitalWrite(GREEN_PIN,LOW);
  
} 

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(GREEN_PIN,digitalRead(BUTTON_PIN));
  digitalWrite(RED_PIN,digitalRead(BUTTON_PIN));
}
