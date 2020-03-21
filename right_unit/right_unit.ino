
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "Shared.h"

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);
const byte directionPin = 6;
const byte stepPin = 5;

unsigned long parallel_step_count = 0;
unsigned long total_steps = 0;

Message msg_data;

void setup() {

  Serial.begin(115200);
  Serial.println(F("Right Unit"));
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(0,15);
  radio.setPayloadSize(16);

  radio.openWritingPipe(addresses[MAIN_UNIT]);
  radio.openReadingPipe(1,addresses[RIGHT_UNIT]);
  radio.startListening();
}

void controlMotor(unsigned long step_count, unsigned long sleep) {
  for (unsigned long x = 1; x <= step_count; x++) {
    digitalWrite(stepPin, HIGH);
    digitalWrite(stepPin, LOW);
    delay(sleep);
    if (x % 5 == 0) Serial.print(".");
  }
  Serial.println("");
}

void setDirection(Direction dir) {
  if (dir == BACKWARD) {
    digitalWrite(directionPin, LOW);
  } else {
    digitalWrite(directionPin, HIGH);
  }
}

void loop() {
  if( radio.available()){
    radio.read( &msg_data, sizeof(msg_data) );

    Serial.println("Received: ");
    Serial.println(msg_data.value);
    Serial.println(msg_data.code);

    switch (msg_data.code)
    {
    case BACKWARD_DIRECT:
      setDirection(BACKWARD);
      controlMotor(msg_data.value, step_delay);
      break;
    case FORWARD_DIRECT:
      setDirection(FORWARD);
      controlMotor(msg_data.value, step_delay);
      break;
    case BACKWARD_PARALLEL:
      setDirection(BACKWARD);
      parallel_step_count = msg_data.value;
      break;
    case FORWARD_PARALLEL:
      setDirection(FORWARD);
      parallel_step_count = msg_data.value;
      break;
    case START_PARALLEL:
      total_steps = msg_data.value;
      controlMotor(parallel_step_count, total_steps / parallel_step_count * step_delay);
      break;
    default:
      Serial.println("Invalid Command");
      break;
    }

    //Serial.print(F("Sent response "));
    //Serial.print(F(" : "));
    //Serial.println(myData.value);
  }
}
