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

#define BRIGHT_THRESHOLD 100
#define MAX_BUFFER_LENGTH 8
#define SAFETY_MAX_STEP_COUNT ((long)(step_length * 100))
#define MARK_BASIS ((long)(step_length * 15)) //the length of the shortest streak and the common divisor of all
#define MARK_MAX_ERROR (MARK_BASIS / 10) // 10% as error margin for mark length
#define UPPER_MARK_ERROR (MARK_BASIS + MARK_MAX_ERROR)
#define LOWER_MARK_ERROR (MARK_BASIS - MARK_MAX_ERROR)
#define MAX_ERROR_LENGTH (MARK_BASIS / 100) // 1% as error margin to prevent incorrect brightness readings
#define PATTERN_LENGTH 3 // the number of white marks that resamble a pattern

enum MarkColor : char { WHITE = 1, BLACK = -1};

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);

TMC2208Stepper driver = TMC2208Stepper(&Serial);	

CircularBuffer<long, MAX_BUFFER_LENGTH> buffer;

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

MarkColor readBeltMark() {
  digitalWrite(LED_PIN, LOW);
  int b1 = analogRead(BRIGHT_PIN);
  digitalWrite(LED_PIN, HIGH);
  int b2 = analogRead(BRIGHT_PIN);
  return b2-b1 > BRIGHT_THRESHOLD ? WHITE : BLACK;
}

void runCalibration() {
  //move to the start of the nearest white area 
  setDirection(FORWARD);
  while (readBeltMark() == WHITE) {
    controlMotor(100, STEP_DELAY_US);
  }
  setDirection(BACKWARD);
  while (readBeltMark() == BLACK) {
    controlMotor(100, STEP_DELAY_US);
  }

  long last_streak = 0;
  long current_streak = 0; // positive numbers > 0 mean white
  long steps_taken = 0;
  while (steps_taken < SAFETY_MAX_STEP_COUNT) {
    steps_taken++;
    setDirection(BACKWARD);
    controlMotor(100, STEP_DELAY_US);
    MarkColor current_mark = readBeltMark();
    // streak continues
    if (!(current_mark > 0 ^ current_streak > 0)) {
      current_streak += current_mark;
    } else { // streak is broken
      if (abs(current_streak) <= MAX_ERROR_LENGTH) {
        current_streak = last_streak - current_streak + current_mark;
        last_streak = 0;
      } else {
        if (abs(last_streak) > 0) {
          buffer.push(last_streak);
          if (calibrate()) break;
        }
        last_streak = current_streak;
        current_streak = current_mark;
      }
    }
  }
  if (steps_taken >= SAFETY_MAX_STEP_COUNT) {
    exit(1);
  }
}

long markLength(long mark) {
  long remainder = abs(mark) % MARK_BASIS;
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

  //change possible black in first and last spot
  //good luck with that

  if (buffer.size() < PATTERN_LENGTH * 2 -1) return false;
  const bool skip_last_black = buffer.last() < 0;
  const unsigned char start_index = (buffer.size()-1) - (char)skip_last_black;
  long pattern[PATTERN_LENGTH];
  long distance = 0;
  for (int index = start_index; index >= (PATTERN_LENGTH -1) * 2; index -= 2) {
    for (int i = 0; i < PATTERN_LENGTH; i++) {
      long white_mark = buffer[index-i*2];
      unsigned long white_mark_length = markLength(white_mark);
      if (white_mark_length == 0) goto outer;
      pattern[PATTERN_LENGTH-i] = white_mark_length;

      if (index-i*2-1 >= 0) {
        long black_mark = buffer[index-i*2-1];
        unsigned long black_mark_length = markLength(black_mark);
        if (black_mark_length != 1) goto outer;
      }
    }
    unsigned long position = patternToPosition(pattern);
    if (position > 0)
      //global_position = position + distance;
      return true;
    outer:;
    distance += abs(buffer[index-1]) + abs(buffer[index]);
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
