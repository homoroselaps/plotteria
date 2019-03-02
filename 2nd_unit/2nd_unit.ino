
/*
* Getting Started example sketch for nRF24L01+ radios
* This is an example of how to send data from one node to another using data structures
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"

byte addresses[][6] = {"Unit1","Unit2"};
const unsigned byte step_delay = 100;

/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
const bool radioNumber = 1;
byte directionPin = 5;
byte stepPin = 6;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(9,10);
/**********************************************************/

struct dataStruct{
  unsigned long _micros;
  unsigned int msg_count;
  int value;
} myData;

void setup() {

  Serial.begin(115200);
  Serial.println(F("Satellite unit "));
  
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  
  // Start the radio listening for data
  radio.startListening();
}

void loop() {
  if( radio.available()){
    radio.read( &myData, sizeof(myData) );             // Get the payload
    radio.stopListening();                               // First, stop listening so we can talk  

    if (myData.value < 0) {
      digitalWrite(directionPin, HIGH);
    } else {
      digitalWrite(directionPin, LOW);
    }
    for (unsigned int x = 1; x <= abs(myData.value); x++) {
      digitalWrite(stepPin, HIGH);
      digitalWrite(stepPin, LOW);
      if (x < num_steps) delay(step_delay);
    }
    radio.write( &myData, sizeof(myData) );              // Send the final one back.      
    radio.startListening();                              // Now, resume listening so we catch the next packets.     
    Serial.print(F("Sent response "));
    Serial.print(myData._micros);  
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
