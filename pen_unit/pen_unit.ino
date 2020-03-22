
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <Servo.h>
#include <SPI.h>
#include "RF24.h"
#include "Shared.h"
#define SERVO_PIN 8
#define POWER_PIN 6
#define ANGLE_DOWN 0
#define ANGLE_UP 90

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);
Servo pen_servo;


unsigned long power_time = 0;
bool power_on = false;

Message msg_data;

void setPen(PenPosition targetPos) {
  if (targetPos == UP) {
    pen_servo.write(ANGLE_UP);
  }
  else {
    pen_servo.write(ANGLE_DOWN);
  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println(F("Pen Unit"));
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(0, 15);
  radio.setPayloadSize(16);
  radio.openWritingPipe(addresses[MAIN_UNIT]);
  radio.openReadingPipe(1, addresses[PEN_UNIT]);
  radio.startListening();

  pen_servo.attach(SERVO_PIN);
  pen_servo.write(ANGLE_DOWN);

  pinMode(POWER_PIN, OUTPUT);
}

void loop() {
  if( radio.available()){
    //turn of burning energy
    power_on = false;
    digitalWrite(POWER_PIN, LOW);

    radio.read( &msg_data, sizeof(msg_data) );

    Serial.println("Received: ");
    Serial.println(msg_data.value);
    Serial.println(msg_data.code);
    
    setPen((PenPosition)msg_data.code);
  }
  if (millis() - power_time > 10000) {
    power_time = millis();
    power_on = true;
    digitalWrite(POWER_PIN, HIGH);
  }
  else if (power_on && (millis() - power_time > 500)) {
    power_on = false;
    digitalWrite(POWER_PIN, LOW);
  }
}
