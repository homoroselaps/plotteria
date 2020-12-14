#include <CircularBuffer.h>

#include <SPI.h>
#include "RF24.h"
#include "Shared.h"
#include <TMC2208Stepper.h>

//#define DEBUG true

#define STEP_PIN 5
#define DIR_PIN 6
#define EN_PIN 4
#define UNIT_PIN A0

#define LED_PIN 2
#define BRIGHT_PIN A6

#define BRIGHT_THRESHOLD 20
#define MAX_BUFFER_LENGTH 8
#define SAFETY_MAX_STEP_COUNT ((long)(5 * numberOfSteps))
#define MARK_BASIS ((long)(step_length * MICROSTEPS * 50)) //the length of the shortest streak and the common divisor of all, 50=10mm
#define MARK_MAX_ERROR (MARK_BASIS / 10) // 10% as error margin for mark length
//#define UPPER_MARK_ERROR (MARK_BASIS + MARK_MAX_ERROR)
#define LOWER_MARK_ERROR (MARK_BASIS - MARK_MAX_ERROR)
#define MAX_ERROR_LENGTH (MARK_BASIS / 100) // 1% as error margin to prevent incorrect brightness readings
#define PATTERN_LENGTH 3 // the number of white marks that resamble a pattern

enum MarkColor : char { WHITE = 1, BLACK = -1};

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);

TMC2208Stepper driver = TMC2208Stepper(&Serial);	

CircularBuffer<long, MAX_BUFFER_LENGTH> buffer;
long step_buffer_start_rel = 0;

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

  pinMode(LED_PIN, OUTPUT);

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

int readBrightnessValue() {
  digitalWrite(LED_PIN, LOW);
  delay(10);
  int b1 = analogRead(BRIGHT_PIN); // higher values are darker, lower values are brighter
  digitalWrite(LED_PIN, HIGH);
  delay(10);
  int b2 = analogRead(BRIGHT_PIN);
  return b1-b2; // higher values are brighter
}

void runCalibration() {
  long step_start, step_end, step_cursor = 0; //step_cursor is used for step_start and step_end
  int value_last, value_current, value_start, value_end = 0;
  int brightness_current, brightness_last = -1;
  //long current_streak = 0; // positive numbers > 0 mean white
  long step_current_rel = 0;
  MarkColor color_to_find = BLACK;
  #define STEPS_PER_READ 320 //20*16
  #define MIN_BRIGHTNESS_CHANGE 100

  step_cursor = 0;
  step_buffer_start_rel = 0;
  value_start = readBrightnessValue();

  while (step_current_rel < SAFETY_MAX_STEP_COUNT) {
    setDirection(FORWARD);
    controlMotor(STEPS_PER_READ, STEP_DELAY_US);
    step_current_rel += STEPS_PER_READ;
    value_last = value_current;
    value_current = readBrightnessValue();
    int brightness_change_current = (value_current - value_last);
    if (brightness_change_current == 0) continue;

    //find first black stronge diff
    if (step_cursor <= 0) {
      if (-brightness_change_current >= MIN_BRIGHTNESS_CHANGE) {
          step_start = step_current_rel;
          step_cursor = step_current_rel;
          Serial.println("first found");
      }
    }
    
    // no mark start found yet
    if (step_start <= 0) {
      // if change has correct sign
      if ((color_to_find==BLACK && brightness_change_current < 0) || (color_to_find==WHITE && brightness_change_current > 0)) {
        // if change is big enough save it as mark's start
        if (abs(value_current-value_start) >= MIN_BRIGHTNESS_CHANGE) {
          step_start = step_cursor;
          step_cursor = 0;
          Serial.println("start found");
        }
      } else {
        // if we are searching white then give the distance to the previous black part
        //step_cursor = 0; break break // can't do this as there must be no gaps in the buffer. you have to clear buffer here !!!
        value_start = 0;
        Serial.println("start flux");
      }
    }
    
    //Found a mark start, no end and total change is too small
    if (step_start > 0 && step_end <= 0 && abs(value_start-value_current) < MIN_BRIGHTNESS_CHANGE) {
      buffer.clear(); // clear the buffer as we read a 
      step_cursor = step_current_rel;
      step_buffer_start_rel = step_current_rel;
      step_start = 0;
      step_end = 0;
      value_start = 0;
      value_end = 0;
      
      color_to_find = (color_to_find==WHITE?BLACK:WHITE);
      Serial.println("Detected bullshit-buffer reset");
    }

    //found mark start, find mark end
    if (step_start > 0) {
      // if change has correct sign
      if ((color_to_find==BLACK && brightness_change_current > 0) || (color_to_find==WHITE && brightness_change_current < 0)) {
        //first time
        if (step_cursor <= 0) {
          step_cursor = step_current_rel;
          value_end = value_current;
          Serial.println("end first");
        }
        // if change is big enough save it as mark's start
        if (abs(value_current-value_end) >= MIN_BRIGHTNESS_CHANGE) {
          step_end = step_cursor;
          step_cursor = 0;
          Serial.println("end found");
        }
      } else {
        step_cursor = 0;
        value_end = 0;
        Serial.println("end flux");
      }
    }
    
    //found start & end
    if (step_start > 0 && step_end > 0) {
      Serial.println("mark found:");
      Serial.println((color_to_find==WHITE?-1:1)*(step_end-step_start));
      buffer.unshift((color_to_find==WHITE?-1:1)*(step_end-step_start)); // add to the front
      if (calibrate()) break;
      step_start = step_end;
      value_start = value_end;
      step_cursor = 0;
      color_to_find = (color_to_find==WHITE?BLACK:WHITE);
    }
  }
  //shut down unit to prevent harm
  if (step_current_rel >= SAFETY_MAX_STEP_COUNT) {
    exit(1);
  }
}

long markLength(long mark) {
  const long remainder = abs(mark) % MARK_BASIS;
  if (remainder < MARK_MAX_ERROR || remainder > LOWER_MARK_ERROR)
      return (abs(mark) + MARK_MAX_ERROR) / MARK_BASIS;
  return 0;
}

long patternToPosition(long pattern[PATTERN_LENGTH]) {
  return 42;
}

bool calibrate() {
  //valid pattern if all marks are a multiple of the mark_basis and black patterns are exactly one mark_basis
  //sum all marks up until the first valid pattern so you know the difference
  
  const bool is_last_black = buffer.last() < 0;
  const bool is_first_black = buffer.first() < 0;
  if (buffer.size() -(char)is_last_black -(char)is_first_black < PATTERN_LENGTH * 2 -1) return false;
  
  long pattern[PATTERN_LENGTH];
  long distance = 0;
  for (int index = (char)is_first_black; index <= buffer.size() -1 -(( PATTERN_LENGTH -1)*2) -(char)is_last_black; index += 2) {
    for (int i = 0; i < PATTERN_LENGTH; i++) {
      // parse the white mark length and save it
      long white_mark = buffer[index+i*2];
      unsigned long white_mark_length = markLength(white_mark);
      if (white_mark_length == 0) goto outer;
      pattern[i] = white_mark_length;

      // check that black mark has correct length
      if (index+i*2+1 < buffer.size()) {
        long black_mark = buffer[index+i*2+1];
        unsigned long black_mark_length = markLength(black_mark);
        if (black_mark_length != 1) goto outer;
      }
    }
    unsigned long position = patternToPosition(pattern);
    if (position > 0)
      //global_position = position + distance
      return true;
    outer:;
    distance += abs(buffer[index+1]) + abs(buffer[index]);
  }
  return false;
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
  digitalWrite(LED_PIN, LOW);
  delay(10);
  int b1 = analogRead(BRIGHT_PIN);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(10);
  int b2 = analogRead(BRIGHT_PIN);
  Serial.println(b1-b2);
  delay(500);

  /*MarkColor color = readBeltMark();
  Serial.println(color);
  delay(500);*/

/*  if( radio.available()){
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
  }*/
}


/*

GOAL
- measure distance from lower turning point to higher turning point
- while moving forward save each difference between Light off and Light on
each cell in the array refers to one motor step
save the start step count for the 0 array position
HOW TO
- color = false 'black'
- iterate over all values in the array

brightness_current, brightness_last = NaN
color = BLACK
step_start, step_end = NaN
value_step_start = NaN
step_current_rel = 0
//idea for finer resolution: if brightness change is in the right direction but not steep enough, sum it up until it is
while (max_calibration_length not reached) {
  brightness_current_change = (value_last-value_current) / distance_traveled // the higher the brighter aka the lower analogValue
  
  if (step_start == NaN && current_brightness_change*(color==BLACK?1:-1) < MIN_BRIGHTNESS_CHANGE) {
      step_start = step_current
      value_step_start = value_current
  }
  if (step_start != NaN && abs(value_step_start-value_current) < MIN_BRIGHTNESS_CHANGE) {
    // reset step_start
    step_start = NaN
    step_end = NaN
    
    // change color as we moved forward
    color = (color==BlACK ? WHITE : BLACK)
  }
  if (step_start != NaN && current_brightness_change*(color==BLACK?1:-1) > MIN_BRIGHTNESS_CHANGE) {
    //register the found mark
    register_mark(step_start-step_end, color)
    
    //start next mark at current's end
    step_start = step_end
    value_step_start = value_current
    color = (color==BlACK ? WHITE : BLACK)
  }
}
- save the first step index S1 for which the diff to its direct neighbor S1+1 is (color?negative:positive) and surpasses threshold T1
- if (diff(last_step, current_step) is (color?positive:negative) and the diff surpasses threshold T1
- reset S1/S2 if current step diff to S1 is lower T1

# this threshold can be smaller the harder the cliff is
- save S1-S2 as a mark of color (color?white:black)
- toggle color to detect next mark

# Idea 2

CONST gather_step_count D1 // the distance traveled must include black and white parts
drive 

*/
