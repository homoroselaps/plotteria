// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

#include <SPI.h>
#include "RF24.h"
#include "Shared.h"

RF24 radio(10,9); // check the pins for SPI 
byte directionPinLeft = 6;
byte stepPinLeft = 5;
byte ledPin = 13;
bool pen_updown = true;

Message msg_data;

unsigned int point_index;
const float motor_distance = 700.0; // the distance between the two motors in mm
unsigned long a_steps; // the number of steps the pen is away from the left motor
unsigned long b_steps; // the number of steps the pen is away from the right motor
Point img_offset = { motor_distance / 2.0, motor_distance / 2.0 };

const byte POINT_COUNT = 5;
float getImg(byte point, bool XorY) {
  const float points[POINT_COUNT*2] = {
    0.0, 0.0, 20.0, 20.0,
    0.0, 40.0, -20.0, 20.0,
    0.0, 0.0,
  };
  if (!XorY) {
    return points[point * (2)  + (byte)XorY] + img_offset.x;  
  } else {
    return points[point * (2)  + (byte)XorY] + img_offset.y;  
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Main unit"));
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(1,addresses[0]);
  radio.startListening();

  // initialize a/b_steps by calibration
  a_steps = (unsigned int) (1.4142135 * (motor_distance/2.0) / step_length);
  b_steps = (unsigned int) (1.4142135 * (motor_distance/2.0) / step_length);
  point_index = 0;
}

void flush() {
  while (radio.available()) {
    char data;
    radio.read( &data, sizeof(data));
    Serial.print(data);
  }
}

bool sendMessage(AddrIndex addr, unsigned int wait_millis=0, byte num_retry = 5) {
  static byte nonce = 0;
  radio.openWritingPipe(addresses[addr]);
  
  for (byte i = 0; i < num_retry; i++) {
    Serial.print("try #");
    Serial.println(i);
    msg_data.nonce = nonce;
    radio.stopListening();
    if (!radio.write( &msg_data, sizeof(msg_data) )){
      Serial.print("Send failed ");
      Serial.println(nonce);
    }

    radio.startListening();
    
    unsigned long started_waiting_at = micros();
    unsigned long timewaited = 0;
    byte counter = 0;
    while ( ! radio.available() ){
      timewaited = micros() - started_waiting_at;
      if (timewaited / 100000 > counter) {
        counter++;
        Serial.print("|");
      }
      if (micros() - started_waiting_at > 2000000 ) {
        break;
      }
    }

    boolean success = false;
    if ( radio.available() ) {
      byte response_nonce;
      radio.read( &response_nonce, sizeof(response_nonce));
      if (response_nonce == nonce) {
        Serial.print(F("Nounce Correct"));
        Serial.println(response_nonce);
        success = true;
      } else {
        Serial.print(F("Nounce Incorrect"));
        Serial.print(nonce);
        Serial.print(F(" : "));
        Serial.println(response_nonce);
      }
    }

    //wait for the right motor to finish
    unsigned int wait_duration = max((int)wait_millis - (int)(timewaited / 1000), 0);
    delay(wait_duration);

    started_waiting_at = micros();
    counter = 0;
    while ( ! radio.available()){
      timewaited = micros() - started_waiting_at;
      if (timewaited / 100000 > counter) {
        counter++;
        Serial.print("|");
      }
      if (micros() - started_waiting_at > 2000000 ) {
        break;
      }
    }
    
    if ( radio.available() ) {
      byte response_nonce;
      radio.read( &response_nonce, sizeof(response_nonce));
      if (response_nonce == nonce) {
        Serial.print(F("Nounce Correct"));
        Serial.println(response_nonce);
        success = true;
      } else {
        Serial.print(F("Nounce Incorrect"));
        Serial.print(nonce);
        Serial.print(F(" : "));
        Serial.println(response_nonce);
      }
    }

    flush();

    if ( success ){
      Serial.println("Success");
      nonce++;
      return true;
    }
  }
  return false;
}

bool controlRightMotor(Direction dir, unsigned int num_steps) {
  Serial.print("Set Right:");
  Serial.print(dir);
  Serial.print(" ,");
  Serial.println(num_steps);
  msg_data.direction = dir;
  msg_data.value = num_steps;
  return sendMessage(RIGHT_UNIT, msg_data.value * step_delay);
}

bool controlPen(PenPosition pos) {
  Serial.print("Set Pen:");
  Serial.println(pos);
  msg_data.direction = pos;
  msg_data.value = 0;
  return sendMessage(PEN_UNIT);
}

void loop() {
  if (point_index >= POINT_COUNT) {
    Serial.println("Finish");
    delay(5000);
    return;
  }

  /*Test 
  Serial.println("Loop");
  
  controlRightMotor(FORWARD, 100);
  //controlPen(UP);
  delay(2000);
  controlRightMotor(BACKWARD, 100);
  //controlPen(DOWN);
  delay(2000);
  
  return;*/

  //iterate over all pair of points
  pen_updown = !pen_updown;
  if (controlPen((PenPosition)pen_updown)) {
    Serial.println("Pen success");
  } else {
    Serial.println("Pen Failure");
  }

  float a = a_steps * step_length; //distance pen to motor_l
  float b = b_steps * step_length; //distance pen to motor_r
  float a_next = sqrt(sq(getImg(point_index, false))+sq(getImg(point_index,true))); //calc dist from p_next to motor_l
  float b_next = sqrt(sq(motor_distance-getImg(point_index, false))+sq(getImg(point_index,true))); //calc dist from p_next to motor_r
  float a_diff = a_next - a;
  float b_diff = b_next - b;
  int step_count_a = a_diff / step_length;
  int step_count_b = b_diff / step_length;
  Direction dir_b, dir_a;
  
  //update the current position of the pen
  a_steps += step_count_a;
  b_steps += step_count_b;
  
  if (step_count_a < 0) {
    dir_a = BACKWARD;
    digitalWrite(directionPinLeft, HIGH);
  } else {
    dir_a = FORWARD;
    digitalWrite(directionPinLeft, LOW);
  }

  if (step_count_b < 0) {
    dir_b = BACKWARD;
  } else {
    dir_b = FORWARD;
  }

  // drop the information of direction
  step_count_a = abs(step_count_a);
  step_count_b = abs(step_count_b);

  //figure out how you want to step which motor
  unsigned int group_size_a = 1;
  unsigned int group_size_b = 1;
  
  //start with the side thats needs more steps
  if (step_count_a > step_count_b) {
    group_size_a = max(step_count_a / (step_count_b + 1),1);
  } else {
    group_size_b = max(step_count_b / (step_count_a + 1),1);
  }

  do {
    delay(step_delay/3);
    // move a closer to target
    unsigned int num_steps = min(step_count_a, group_size_a);
    Serial.print("Set Left:");
    Serial.print(dir_a);
    Serial.print(" ,");
    Serial.println(num_steps);
    for (unsigned int x = 1; x <= num_steps; x++) {
      digitalWrite(stepPinLeft, HIGH);
      digitalWrite(stepPinLeft, LOW);
      if (x < num_steps) delay(step_delay);
    }
    step_count_a -= num_steps;
    delay(step_delay/3);
    
    // move b closer to target
    num_steps = min(step_count_b, group_size_b);
    if (controlRightMotor(dir_b, num_steps)) {
      step_count_b -= num_steps;
      // save state to eeprom
    }
  
    //Serial.print("End of Loop: ");
    //Serial.print(step_count_a);
    //Serial.print(" ");
    //Serial.println(step_count_b);
  } while (step_count_a > 0 || step_count_b > 0);
  
  point_index++;
}
