
#include <SPI.h>
#include "RF24.h"
#include "Shared.h"
#include <TMC2208Stepper.h>

//#define DEBUG true

#define STEP_PIN 5
#define DIR_PIN 6
#define EN_PIN 4
#define UNIT_PIN A0

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);

TMC2208Stepper driver = TMC2208Stepper(&Serial);	

unsigned long parallel_step_count = 0;
unsigned long total_steps = 0;

Message msg_data;
bool is_right_unit = false;

void setup() {

  Serial.begin(115200);
  while(!Serial);
  
  pinMode(UNIT_PIN, INPUT);
  is_right_unit = digitalRead(UNIT_PIN);

  if (is_right_unit) {
    Serial.println(F("Right Unit")); 
  } else {
    Serial.println(F("Left Unit"));
  }

  //Setup Radio module
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.setRetries(0, 15);
  radio.setPayloadSize(16);
  //radio.printDetails();

  radio.openWritingPipe(addresses[MAIN_UNIT]);
  if (is_right_unit) {
    radio.openReadingPipe(1, addresses[RIGHT_UNIT]);
  } else {
    radio.openReadingPipe(1, addresses[LEFT_UNIT]);
  }
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
  driver.mstep_reg_select(1);     // configure microsteps via register
  driver.microsteps(MICROSTEPS);  // set microstepping
	driver.toff(0x2);							  // Enable driver

  digitalWrite(EN_PIN, LOW); //Enable driver
}

void controlMotor(unsigned long step_count, unsigned long sleep) {
  for (unsigned long x = 1; x <= step_count; x++) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(sleep);
    #ifdef DEBUG
      if (!(x%10)) Serial.print(".");
    #endif
  }
  #ifdef DEBUG
    Serial.println("");
  #endif
}

void setDirection(Direction dir) {
  driver.shaft(dir == BACKWARD);
}

void loop() {
  if( radio.available()){
    radio.read( &msg_data, sizeof(msg_data) );

    #ifdef DEBUG
      Serial.println("Received: ");
      Serial.println(msg_data.value);
      Serial.println(msg_data.code);
    #endif

    switch (msg_data.code)
    {
    case BACKWARD_DIRECT:
      setDirection(BACKWARD);
      controlMotor(msg_data.value, STEP_DELAY_US);
      break;
    case FORWARD_DIRECT:
      setDirection(FORWARD);
      controlMotor(msg_data.value, STEP_DELAY_US);
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
      controlMotor(parallel_step_count, total_steps / parallel_step_count * STEP_DELAY_US);
      break;
    default:
      #ifdef DEBUG
        Serial.println("Invalid Command");
      #endif
      break;
    }
  }
}
