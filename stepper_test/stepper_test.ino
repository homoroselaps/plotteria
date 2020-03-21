
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "Shared.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
//RF24 radio(9,10);
const byte directionPin = 6;
const byte stepPin = 5;

Message msg_data;
bool toggle = false;

void setup() {

  Serial.begin(115200);
  Serial.println(F("Right Unit"));
  
  /*radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[MAIN_UNIT]);
  radio.openReadingPipe(1,addresses[RIGHT_UNIT]);
  radio.startListening();*/
}

void loop() {
  if (toggle) {
    digitalWrite(directionPin, LOW);
  } else {
    digitalWrite(directionPin, HIGH);
  }
  for (unsigned int x = 1; x <= 100; x++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    delay(step_delay);
    if (x % 2 == 0) {
      digitalWrite(3, HIGH);
    } else {
      digitalWrite(3, LOW);
    }
  }
  toggle = !toggle;
  
  /*if( radio.available()){
    radio.read( &msg_data, sizeof(msg_data) );
    radio.stopListening(); 

    //Send Nonce
    radio.write( &msg_data.nonce, sizeof(msg_data.nonce) );

    Serial.print("Received");
    Serial.println(msg_data.nonce);

    Serial.println(msg_data.direction);
    if (msg_data.direction == BACKWARD) {
      digitalWrite(directionPin, LOW);
    } else {
      digitalWrite(directionPin, HIGH);
    }
    Serial.println(msg_data.value);
    bool toogle = false;
    for (unsigned int x = 1; x <= msg_data.value; x++) {
      digitalWrite(stepPin, HIGH);
      delay(step_delay);
      digitalWrite(stepPin, LOW);
      delay(step_delay);
      if (toogle) {
        digitalWrite(3, HIGH);
      } else {
        digitalWrite(3, LOW);
      }
      toogle = !toogle;
    }

    for (byte i = 0; i < 1; i++) {
      if (!radio.write( &msg_data.nonce, sizeof(msg_data.nonce) )){
        Serial.print("Send failed ");
        Serial.println(msg_data.nonce);
        continue;
      } else {
        Serial.print("Sent response ");
        Serial.println(msg_data.nonce);
        break;
      }
    }
    radio.startListening();
    //Serial.print(F("Sent response "));
    //Serial.print(F(" : "));
    //Serial.println(myData.value);
  }*/
}
