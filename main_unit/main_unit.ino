// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

#include <SPI.h>
#include "RF24.h"
#include "Shared.h"
#include <IRremote.h> 

enum STMState : byte { READY = 0, DRAW, MOVER, MOVEL, MOVEU, MOVED, MOTOR_RF, MOTOR_RB, MOTOR_LF, MOTOR_LB};
byte ce_pin  = 10;
byte csn_pin = 9;
byte ir_pin  = 8;
byte dir_pin = 6;
byte step_pin = 5;

RF24 radio(ce_pin,csn_pin); // (CE,CSN)
Message msg_data;

IRrecv irrecv(ir_pin);
decode_results results;

bool pen_updown = true;
STMState current_state = READY;

unsigned int point_index;
const float motor_distance = 700.0; // the distance between the two motors in mm
unsigned long current_a; // the number of steps the pen is away from the left motor
unsigned long current_b; // the number of steps the pen is away from the right motor
Point img_offset;
Point origin;

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

  irrecv.enableIRIn(); // Start the receiver
  
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(15, 15);
  radio.openReadingPipe(1,addresses[0]);
  radio.startListening();

  // initialize a/current_b by calibration
  calibrate();
  save_img_offset();
  point_index = 0;
}

void calibrate() {
  current_a = (unsigned int) (1.4142135 * (motor_distance/2.0) / step_length);
  current_b = (unsigned int) (1.4142135 * (motor_distance/2.0) / step_length);
  origin = (Point){ motor_distance / 2.0, motor_distance / 2.0 };
  Serial.println((String)"origin saved"+origin.x+", "+origin.y);
  Serial.println((String)"ab:"+current_a+", "+current_b);
}

void flush() {
  while (radio.available()) {
    char data;
    radio.read( &data, sizeof(data));
    Serial.print(data);
  }
}

bool sendMessage(AddrIndex addr, unsigned int wait_millis=0, byte num_retry = 2) {
  static byte nonce = 0;
  radio.openWritingPipe(addresses[addr]);
  
  for (byte i = 0; i < num_retry; i++) {
    Serial.print("try #");
    Serial.print(i);
    msg_data.nonce = nonce;
    radio.stopListening();
    if (!radio.write( &msg_data, sizeof(msg_data) )){
      Serial.print("Send failed ");
      Serial.println(nonce);
      continue;
    }
    nonce++;
    return true;

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
    Serial.println("");

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

bool controlLeftMotor(Direction dir, unsigned int num_steps) {
  Serial.print("Set Left:");
  Serial.print(dir);
  Serial.print(" ,");
  Serial.println(num_steps);
  if (dir == BACKWARD) {
    digitalWrite(dir_pin, HIGH);
  } else {
    digitalWrite(dir_pin, LOW);
  }
  for (unsigned int x = 1; x <= num_steps; x++) {
    digitalWrite(step_pin, HIGH);
    digitalWrite(step_pin, LOW);
    if (x < num_steps) delay(step_delay);
  }
  return true;
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

Point abToPoint(unsigned int steps_a, unsigned int steps_b, float motor_distance) {
  float length_a = steps_a * step_length;
  float length_b = steps_b * step_length;
  float s = 0.5 * (length_a + length_b + motor_distance);
  float y = (2.0 / motor_distance) * sqrt(s*(s-length_a)*(s-length_b)*(s-motor_distance));
  float x = sqrt(sq(length_a)-sq(y));
  return (Point){x,y};
}

void save_img_offset() {
  img_offset = abToPoint(current_a, current_b, motor_distance);
  Serial.println((String)"img offset saved"+img_offset.x+", "+img_offset.y);
}

bool moveTo(
  unsigned int target_a,
  unsigned int target_b
) {
  Direction dir_a = target_a < current_a ? BACKWARD : FORWARD;
  Direction dir_b = target_b < current_b ? BACKWARD : FORWARD;
  long step_count_a = target_a - current_a;
  long step_count_b = target_b - current_b;

  //update the current position of the pen
  current_a += step_count_a;
  current_b += step_count_b;

  //drop direction information
  step_count_a = abs(step_count_a);
  step_count_b = abs(step_count_b);

  //figure out how you want to step which motor
  unsigned int group_size_a = 1;
  unsigned int group_size_b = 1;
  
  //start with the side thats needs more steps
  if (step_count_a > step_count_b) {
    group_size_a = max(step_count_a / (step_count_b + 1), 1);
  } else {
    group_size_b = max(step_count_b / (step_count_a + 1), 1);
  }

  do {
    delay(step_delay/3);
    // move a closer to target
    unsigned int num_steps_a = min(step_count_a, group_size_a);
    if (controlLeftMotor(dir_a, num_steps_a)) {
      step_count_a -= num_steps_a;
    }
    
    // move b closer to target
    unsigned int num_steps_b = min(step_count_b, group_size_b);
    if (controlRightMotor(dir_b, num_steps_b)) {
      step_count_b -= num_steps_b;
      // save state to eeprom
    }
    Serial.println((String)"moveTo:"+step_count_a+", "+step_count_b);
    Serial.println((String)"moveTo:"+num_steps_a+", "+num_steps_b);
  } while (step_count_a > 0 || step_count_b > 0);
  return true;
}

bool moveTo(
  float x, 
  float y
) {
  float target_a = sqrt(sq(x)+sq(y)) / step_length;
  float target_b = sqrt(sq(motor_distance-x)+sq(y)) / step_length;
  return moveTo((unsigned int)target_a, (unsigned int)target_b);
}

void handle_READY() {
  if (irrecv.decode(&results))
  {
    Serial.println(results.value, HEX);
    switch (results.value)
    {
      case R1_Play: current_state = DRAW; break;
      case R1_Enter: save_img_offset(); break;
      case R1_Setup: calibrate(); break;
      case R1_Stop:  moveTo(origin.x, origin.y); break;
      case R1_VolPlus:  controlPen(UP); break;
      case R1_VolMinus: controlPen(DOWN); break;
      case R1_Right: current_state = MOVER; break;
      case R1_Left:  current_state = MOVEL; break;
      case R1_Up:    current_state = MOVEU; break;
      case R1_Down:  current_state = MOVED; break;
      case R1_Prev:  current_state = MOTOR_LB; break;
      case R1_TuneL: current_state = MOTOR_LF; break;
      case R1_TuneR: current_state = MOTOR_RF; break;
      case R1_Next:  current_state = MOTOR_RB; break;

      default: break;
    }
    irrecv.resume();
  }
}

void handle_MOTOR_LB() {
  if (controlLeftMotor(BACKWARD, 1)) {
    current_a--;
  }
  delay(step_delay);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOTOR_LF() {
  if (controlLeftMotor(FORWARD, 1)) {
    current_a++;
  }
  delay(step_delay);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOTOR_RB() {
  if (controlRightMotor(BACKWARD, 1)) {
    current_b--;
  }
  delay(step_delay);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOTOR_RF() {
  if (controlRightMotor(FORWARD, 1)) {
    current_b++;
  }
  delay(step_delay);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOVER() {
  Point current_position = abToPoint(current_a, current_b, motor_distance);
  moveTo(current_position.x+5, current_position.y);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOVEL() {
  Point current_position = abToPoint(current_a, current_b, motor_distance);
  moveTo(current_position.x-5, current_position.y);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOVEU() {
  Point current_position = abToPoint(current_a, current_b, motor_distance);
  moveTo(current_position.x, current_position.y-5);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_MOVED() {
  Point current_position = abToPoint(current_a, current_b, motor_distance);
  moveTo(current_position.x, current_position.y+5);
  if (irrecv.decode(&results) && results.value == 0xFFFFFFFF)
  {
    irrecv.resume(); //consume
  } else {
    current_state = READY;
  }
}

void handle_DRAW() {
  if (irrecv.decode(&results)) 
  {
    switch (results.value)
    {
      case R1_Pause: 
        current_state = READY;
        return;
      default: break;
    }
    irrecv.resume(); //consume
  }
  if (point_index >= POINT_COUNT) {
    Serial.println("Finish");
    current_state = READY;
    point_index = 0;
    return;
  }
  pen_updown = !pen_updown;
  if (controlPen((PenPosition)pen_updown)) {
    Serial.println("Pen success");
  } else {
    Serial.println("Pen Failure");
  }

  moveTo(getImg(point_index, false), getImg(point_index, true));
  point_index++;
}

void loop() {
  switch (current_state)
  {
    case READY: handle_READY(); break;
    case DRAW: handle_DRAW(); break;
    case MOTOR_LB: handle_MOTOR_LB(); break;
    case MOTOR_LF: handle_MOTOR_LF(); break;
    case MOTOR_RF: handle_MOTOR_RF(); break;
    case MOTOR_RB: handle_MOTOR_RB(); break;
    case MOVER: handle_MOVER(); break;
    case MOVEL: handle_MOVEL(); break;
    case MOVEU: handle_MOVEU(); break;
    case MOVED: handle_MOVED(); break;
    default: break;
  }
}
