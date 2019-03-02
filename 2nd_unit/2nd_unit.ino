
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"

const byte directionPin = 5;
const byte stepPin = 6;
bool msg_bit = false;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(9,10);

byte addresses[][6] = {"Unit1","Unit2"}; // Unit1 = Main, Unit2 = Satellite
const unsigned byte step_delay = 100;
enum direction : bool { FORWARD = true, BACKWARDS = false};

struct dataStruct{
  bool msg_bit;
  bool direction;
  unsigned int value;
} dataStruct;

void setup() {

  Serial.begin(115200);
  Serial.println(F("Unit2"));
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  if( radio.available()){
    radio.read( &dataStruct, sizeof(dataStruct) );             // Get the payload
    radio.stopListening();                               // First, stop listening so we can talk  

    // skip message 
    if (msg_bit == dataStruct.msg_bit) {

      if (myData.direction == BACKWARDS) {
        digitalWrite(directionPin, LOW);
      } else {
        digitalWrite(directionPin, HIGH);
      }
      for (unsigned int x = 1; x <= dataStruct.value; x++) {
        digitalWrite(stepPin, HIGH);
        digitalWrite(stepPin, LOW);
        delay(step_delay);
      }
    }
    radio.write( &dataStruct.msg_bit, sizeof(&dataStruct.msg_bit) );
    radio.startListening();
    Serial.print(F("Sent response "));
    Serial.print(F(" : "));
    Serial.println(myData.value);
  }

/****************** Change Roles via Serial Commands ***************************/

  if ( Serial.available() )
  {
    char c = toupper(Serial.read());
    if ( c == 'T' && role == 0 ){      
      Serial.print(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
      role = 1;                  // Become the primary transmitter (ping out)
    
   }else
    if ( c == 'R' && role == 1 ){
      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));      
       role = 0;                // Become the primary receiver (pong back)
       radio.startListening();
       
    }
  }


} // Loop
