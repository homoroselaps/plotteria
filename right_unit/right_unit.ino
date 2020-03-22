
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"
#include "Shared.h"
#include <TMC2208Stepper.h>

#define IS_RIGHT_UNIT true
#define STEP_PIN 5
#define DIR_PIN 6
#define EN_PIN 4
#define MRES 4

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);

TMC2208Stepper driver = TMC2208Stepper(&Serial);	

unsigned long parallel_step_count = 0;
unsigned long total_steps = 0;

Message msg_data;

void setup() {

  Serial.begin(115200);
  while(!Serial);

  #ifdef IS_RIGHT_UNIT
    Serial.println(F("Right Unit"));
  #else
    Serial.println(F("Left Unit"));
  #endif

  //Setup Radio module
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(0, 15);
  radio.setPayloadSize(16);
  driver.shaft(false);

  radio.openWritingPipe(addresses[MAIN_UNIT]);
  #ifdef IS_RIGHT_UNIT
    radio.openReadingPipe(1, addresses[RIGHT_UNIT]);
  #else
    radio.openReadingPipe(1, addresses[LEFT_UNIT]);
    
  #endif
  radio.startListening();

  //Setup Driver module

  pinMode(EN_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  driver.pdn_disable(1);         // Use PDN/UART pin for communication
	driver.I_scale_analog(0);			 // Adjust current from the registers
	driver.rms_current(500);			 // Set driver current 500mA
  driver.intpol(true);           // enable x256 interpolation
  driver.shaft(false);
  driver.mres(MRES);             // set 1/2^4 microstepping
	driver.toff(0x2);							 // Enable driver

  digitalWrite(EN_PIN, LOW); //Enable driver
}

void controlMotor(unsigned long step_count, unsigned long sleep) {
  for (unsigned long x = 1; x <= step_count*pow(2, MRES); x++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delay(sleep);
    if (x % 5 == 0) Serial.print(".");
  }
  Serial.println("");
}

void setDirection(Direction dir) {
  if (dir == BACKWARD) {
    driver.shaft(true);
  } else {
    driver.shaft(false);
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
