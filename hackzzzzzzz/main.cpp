#include <Arduino.h>
#define LIGHT_PIN 11 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Start serial communication
  pinMode(LIGHT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      digitalWrite(LIGHT_PIN, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(10);  
      digitalWrite(LIGHT_PIN, LOW);
      digitalWrite(LED_BUILTIN, LOW); 
      delay(10); 
    } else if (command == '0') {
      digitalWrite(LIGHT_PIN, LOW);  
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
