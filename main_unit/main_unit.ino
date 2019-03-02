// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

#include <Servo.h>
#include <SPI.h>
#include "RF24.h"

Servo pen_servo;
byte servoPin = 8;

RF24 radio(9,10); // check the pins for SPI 
byte directionPinLeft = 9;
byte stepPinLeft = 10;
byte ledPin = 13;
bool pen_updown = true;

int numberOfSteps = 200;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 50; // milliseconds - or try 1000 for slower steps

byte addresses[][6] = {"Unit1","Unit2"}; // Unit1 = Main, Unit2 = Satellite
const unsigned long step_delay = 100;
enum direction : bool { FORWARD = true, BACKWARDS = false};

struct dataStruct {
  bool msg_bit;
  bool direction;
  unsigned int value;
} dataStruct;

struct Point {
  float x;
  float y;
};

unsigned int point_index;
const float motor_distance = 700.0; // the distance between the two motors in mm
const float step_length = 40.0/200.0; // the distance a step increases the string in mm
unsigned long a_steps; // the number of steps the pen is away from the left motor
unsigned long b_steps; // the number of steps the pen is away from the right motor
Point img_offset = { motor_distance / 2.0, motor_distance / 2.0 };
bool unit2_msg_bit = false;

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
  Serial.println(F("Main unit "));
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1,addresses[0]);
  radio.startListening();

  pen_servo.attach(servoPin);
  // initialize a/b_steps by calibration
  a_steps = (unsigned int) (1.4142135 * (motor_distance/2.0) / step_length);
  b_steps = (unsigned int) (1.4142135 * (motor_distance/2.0) / step_length);
  point_index = 0;
}

void set_pen(bool UpOrDown) {
  const byte angle_down = 35;
  const byte angle_up = 30;
  if (!UpOrDown) {
    pen_servo.write(angle_up);
  }
  else {
    for (int pos = angle_up; pos <= angle_down; pos += 1) { // goes from 0 degrees to 180 degrees
      pen_servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(150);                       // waits 15ms for the servo to reach the position
    }  
  }
}

void loop() {
  if (point_index >= POINT_COUNT) {
    Serial.println("Finish");
    set_pen(false);
    delay(5000);
    return;
  }
  //iterate over all pair of points
  pen_updown = !pen_updown;
  set_pen(pen_updown);
  float a = a_steps * step_length; //distance pen to motor_l
  float b = b_steps * step_length; //distance pen to motor_r
  float a_next = sqrt(sq(getImg(point_index, false))+sq(getImg(point_index,true))); //calc dist from p_next to motor_l
  float b_next = sqrt(sq(motor_distance-getImg(point_index, false))+sq(getImg(point_index,true))); //calc dist from p_next to motor_r
  float a_diff = a_next - a;
  float b_diff = b_next - b;
  int step_count_a = a_diff / step_length;
  int step_count_b = b_diff / step_length;
  Direction dir_b,dir_a;
  
  //update the current position of the pen
  a_steps += step_count_a;
  b_steps += step_count_b;
  
  if (step_count_a < 0) {
    dir_a = BACKWARDS;
    digitalWrite(directionPinLeft, HIGH);
  } else {
    dir_a = FORWARD;
    digitalWrite(directionPinLeft, LOW);
  }

  if (step_count_b < 0) {
    dir_b = BACKWARDS;
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
    for (unsigned int x = 1; x <= num_steps; x++) {
      digitalWrite(stepPinLeft, HIGH);
      digitalWrite(stepPinLeft, LOW);
      if (x < num_steps) delay(step_delay);
    }
    step_count_a -= num_steps;
    delay(step_delay/3);
    
    // move b closer to target
    num_steps = min(step_count_b, group_size_b);

    bool success = false;
    while (!success) {
      radio.stopListening();
      // send the message 
      dataStruct.msg_bit = unit2_msg_bit;
      dataStruct.direction = dir_b;
      dataStruct.value = num_steps;

      if (!radio.write( &dataStruct, sizeof(dataStruct) )){
        Serial.println(F("Send failed"));
        continue;
      }

      radio.startListening();
      unsigned long started_waiting_at = micros();
      boolean timeout = false;
      while ( ! radio.available() ){
        if (micros() - started_waiting_at > 200000 ){
            timeout = true;
            break;
        }
      }

      if ( timeout ){
        Serial.println(F("Response timeout"));
      } else {
        bool response_msg_bit;
        radio.read( &response_msg_bit, sizeof(response_msg_bit));
        if (response_msg_bit == unit2_msg_bit) {
          unit2_msg_bit = !unit2_msg_bit;
          step_count_b -= num_steps;
          // save state to eeprom
          success = true;
        }
      }
    }
  
    //Serial.print("End of Loop: ");
    //Serial.print(step_count_a);
    //Serial.print(" ");
    //Serial.println(step_count_b);
  } while (step_count_a > 0 || step_count_b > 0);
  
  point_index++;
}
