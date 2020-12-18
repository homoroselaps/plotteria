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

#define BRIGHT_THRESHOLD 675 //(950-400)/2 + 400
#define MAX_BUFFER_LENGTH 8
#define SAFETY_MAX_STEP_RADIUS 10000
#define MARK_BASIS 800 //the length of the shortest streak and the common divisor of all
#define STEPS_PER_READ 5 // number of steps to move forward after each calibration step read
#define MARK_MAX_ERROR (MARK_BASIS / 8) // 12.5% error margin for mark length
//#define UPPER_MARK_ERROR (MARK_BASIS + MARK_MAX_ERROR)
#define LOWER_MARK_ERROR (MARK_BASIS - MARK_MAX_ERROR)
#define MAX_ERROR_LENGTH (4*STEPS_PER_READ) // x continues false readings as error margin to prevent incorrect/fluctuating brightness readings
#define PATTERN_LENGTH 3 // the number of white marks that resamble a pattern
#define MAX_RETRIES 32

enum MarkColor : char { WHITE = 1, BLACK = -1};

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins*/
RF24 radio(10,9);

TMC2208Stepper driver = TMC2208Stepper(&Serial);	

CircularBuffer<long, MAX_BUFFER_LENGTH> buffer;
CircularBuffer<Message, 128> debug_buffer;

unsigned long parallel_step_count = 0;
unsigned long total_steps = 0;
long steps_taken = 0;

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
  //radio.enableAckPayload();
  radio.setRetries(15, 15);
  radio.setPayloadSize(8);
  //radio.printDetails();

  radio.stopListening();
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

MarkColor readBeltMark22(long steps_taken) {
  const int mock[21] = {0, -1, 2, -4, 8, -16, 32, -50, 75, -100, 150, -200, 1000, -1800, 2600, -3400, 4200, -5800, 6600, -7400, 9600};
  MarkColor value = BLACK;
  for (size_t i = 0; i < 21; i++)
  {
    if (steps_taken >= abs(mock[i])) 
      value = mock[i] < 0 ? BLACK : WHITE;
    else
      return value;
  }
  return value;
}

MarkColor readBeltMark(long _) {
  digitalWrite(LED_PIN, LOW);
  delay(10);
  int b_off = analogRead(BRIGHT_PIN);
  digitalWrite(LED_PIN, HIGH);
  delay(10);
  int b_on = analogRead(BRIGHT_PIN);
  //Serial.println(b_off);
  //Serial.println(b_on);
  return b_on < BRIGHT_THRESHOLD-(1024-b_off) ? WHITE : BLACK;
}

void printBuffer() {
  //Serial.print("buffer: ");
  add_debug_msg(4,0);
  for (int index = 0; index <= buffer.size() -1; index++) {
    if (buffer[index] < 0)
      add_debug_msg(41, abs(buffer[index]));
    else
      add_debug_msg(4, buffer[index]);
    //Serial.print(buffer[index]);
    //Serial.print(", ");
  }
  //Serial.println(" /");
}

void runCalibration() {
  buffer.clear();
  //move to the start of the nearest white area 
  add_debug_msg(1,0);
  setDirection(FORWARD);
  while (readBeltMark(steps_taken) >= WHITE && steps_taken < SAFETY_MAX_STEP_RADIUS) {
    controlMotor(STEPS_PER_READ, STEP_DELAY_US * 5);
    steps_taken += STEPS_PER_READ;
  }
  if (steps_taken >= SAFETY_MAX_STEP_RADIUS) {
    add_debug_msg(11,steps_taken);
    return;
  }
  add_debug_msg(10,steps_taken);
  setDirection(BACKWARD);
  while (readBeltMark(steps_taken) <= BLACK && steps_taken > -SAFETY_MAX_STEP_RADIUS) {
    controlMotor(STEPS_PER_READ, STEP_DELAY_US * 5);
    steps_taken -= STEPS_PER_READ;
  }
  add_debug_msg(20,steps_taken);

  long last_streak = 0;
  long current_streak = 0; // positive numbers > 0 mean white
  while (steps_taken <= SAFETY_MAX_STEP_RADIUS) {
    steps_taken += STEPS_PER_READ;
    setDirection(FORWARD);
    controlMotor(STEPS_PER_READ, STEP_DELAY_US * 5);
    int current_mark = (int)readBeltMark(steps_taken)*STEPS_PER_READ;
    // streak continues
    if (!(current_mark > 0 ^ current_streak > 0)) {
      current_streak += current_mark;
    } else { // streak is broken
      //Serial.println("streak broken: ");
      //Serial.println(current_streak);
      if (abs(current_streak) <= MAX_ERROR_LENGTH) {
        //Serial.println("classified as error");
        add_debug_msg(31,abs(current_streak));
        current_streak = last_streak - current_streak + current_mark;
        last_streak = 0;
      } else {
        //Serial.println("classified as new mark");
        add_debug_msg(32,abs(last_streak));
        if (abs(last_streak) > 0) {
          buffer.unshift(last_streak); // add to the front
          printBuffer();
          if (calibrate()) break;
        }
        last_streak = current_streak;
        current_streak = current_mark;
      }
    }
  }
  if (steps_taken >= SAFETY_MAX_STEP_RADIUS) {
    add_debug_msg(3,3);
    return;
  }
}

long markLength(long mark) {
  const long remainder = abs(mark) % MARK_BASIS;
  if (remainder <= MARK_MAX_ERROR || remainder >= LOWER_MARK_ERROR)
      return (abs(mark) + MARK_MAX_ERROR) / MARK_BASIS;
  return 0;
}

long patternToPosition(long pattern[PATTERN_LENGTH]) {
  return 42;
}

bool add_debug_msg(long code, long value) {
  return debug_buffer.push(Message{code, value});
}

bool calibrate() {
  //Serial.println("calibrate");
  //valid pattern if all marks are a multiple of the mark_basis and black patterns are exactly one mark_basis
  //sum all marks up until the first valid pattern so you know the difference
  
  const bool is_last_black = buffer.last() < 0;
  const bool is_first_black = buffer.first() < 0;
  if (buffer.size() -(char)is_last_black -(char)is_first_black < PATTERN_LENGTH * 2 -1) return false;
  
  long pattern[PATTERN_LENGTH];
  long distance = 0;
  for (int index = (char)is_first_black; index <= buffer.size() -1 -(( PATTERN_LENGTH -1)*2) -(char)is_last_black; index += 2) {
    //Serial.print("Pattern-");
    //Serial.println(index);
    for (int i = 0; i < PATTERN_LENGTH; i++) {
      //Serial.print("Mark-");
      //Serial.print(i);
      //Serial.print(": ");
      // parse the white mark length and save it
      long white_mark = buffer[index+i*2];
      unsigned long white_mark_length = markLength(white_mark);
      //Serial.print(white_mark);
      //Serial.print("#");
      //Serial.print(white_mark_length);
      //Serial.print(", ");
      if (white_mark_length == 0) goto outer;
      pattern[i] = white_mark_length;

      // check that black mark has correct length
      if (index+i*2+1 < buffer.size()) {
        long black_mark = buffer[index+i*2+1];
        unsigned long black_mark_length = markLength(black_mark);
        //Serial.print(black_mark);
        //Serial.print("#");
        //Searial.println(black_mark_length);
        if (black_mark_length != 1) goto outer;
      }
    }
    unsigned long position = patternToPosition(pattern);
    if (position > 0)
      //global_position = position + distance // the last streak is added to the buffer hence think about this carefully
      add_debug_msg(50, position);
      for (size_t i = 0; i < PATTERN_LENGTH; i++)
      {
        add_debug_msg(50, pattern[i]);
      }
      //Serial.println("position found");
      return true;
    outer:;
    //Serial.println("_");
    add_debug_msg(51,0);
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
  if(radio.available()){
    radio.read( &msg_data, sizeof(msg_data) );

    #ifdef DEBUG
      Serial.print("Received code:");
      Serial.print(msg_data.code);
      Serial.print(" value:");
      Serial.println(msg_data.value);
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
    case CALIBRATE_DEVICE:
      //Serial.println("RunCalibration");
      runCalibration();
      //Serial.println("FinishedCalibration");
      break;
    case DEBUG_DEVICE:
      radio.stopListening();
      bool success = false;
      msg_data.code = 0;
      msg_data.value = debug_buffer.size();
      for (size_t i = 0; i < MAX_RETRIES; i++)
        {
        success = radio.write(&msg_data, sizeof(msg_data));
        #ifdef DEBUG
          Serial.print(msg_data.code);
          Serial.print(",");
          Serial.print(msg_data.value);
          Serial.print(": ");
          Serial.println(success);
        #endif
        if (success) break;
        delay(53);
      }
      if (!success) break;
      
      for (int index = 0; index <= debug_buffer.size() -1; index++) {
        for (size_t i = 0; i < MAX_RETRIES; i++)
        {
          success = radio.write(&(debug_buffer[index]),sizeof(msg_data));
          #ifdef DEBUG
            Serial.print(debug_buffer[index].code);
            Serial.print(",");
            Serial.print(debug_buffer[index].value);
            Serial.print(": ");
            Serial.println(success);
          #endif
          delay(53);
          if (success) break;
        }
      }
      radio.startListening();
      break;
    default:
      #ifdef DEBUG
        Serial.println("Invalid Command");
      #endif
      break;
    }
  }
}
