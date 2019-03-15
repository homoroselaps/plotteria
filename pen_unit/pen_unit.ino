
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <Servo.h>
#include <SPI.h>
#include "RF24.h"
#include "Shared.h"


/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);
Servo pen_servo;
byte servoPin = 8;

const byte angle_down = 0;
const byte angle_up = 90;

Message msg_data;

void setPen(PenPosition targetPos) {
  if (targetPos == UP) {
    pen_servo.write(angle_up);
  }
  else {
    pen_servo.write(angle_down);
  }
}

void setup() {

  Serial.begin(115200);
  Serial.println(F("Pen Unit"));
  
  radio.begin();

  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[MAIN_UNIT]);
  radio.openReadingPipe(1,addresses[PEN_UNIT]);
  radio.startListening();

  pen_servo.attach(servoPin);
  pen_servo.write(angle_down);
}

void loop() {
  if( radio.available()){
    radio.read( &msg_data, sizeof(msg_data) );             // Get the payload
    radio.stopListening();                               // First, stop listening so we can talk  

    //Send Nonce
    radio.write( &msg_data.nonce, sizeof(msg_data.nonce) );

    Serial.print("Received");
    Serial.println(msg_data.nonce);
    
    setPen((PenPosition)msg_data.direction);
    
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
  }
}
