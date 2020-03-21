
const unsigned int step_delay = 100;

uint64_t addresses[4] = { 0xF0F0F0F0E1, 0xF0F0F0F0D2, 0xF0F0F0F0C3, 0xF0F0F0F0B4 };
enum AddrIndex : byte { MAIN_UNIT = 0, LEFT_UNIT = 1, RIGHT_UNIT = 2, PEN_UNIT = 3 };
enum Command : byte { BACKWARD_DIRECT = 0, FORWARD_DIRECT = 1, BACKWARD_PARALLEL = 2, FORWARD_PARALLEL = 3, START_PARALLEL = 4 };
enum Direction : bool { FORWARD = true, BACKWARD = false};
enum PenPosition : bool { DOWN = true, UP = false};

int numberOfSteps = 3200;
const float step_length = 40.0 / numberOfSteps; // the distance a step increases the string in mm

struct Message {
  unsigned long value;
  unsigned long code;
};

struct Point {
  float x;
  float y;
};

//Alex Remote Control
const unsigned long R1_Play = 0x59A6C03F;
const unsigned long R1_Pause = 0x59A6D827;
const unsigned long R1_Right = 0x59A6629D;
const unsigned long R1_Left  = 0x59A6DA25;
const unsigned long R1_Up    = 0x59A6F20D;
const unsigned long R1_Down  = 0x59A66A95;
const unsigned long R1_Prev  = 0x59A6609F;
const unsigned long R1_TuneL = 0x59A6E817;
const unsigned long R1_TuneR = 0x59A6E01F;
const unsigned long R1_Next  = 0x59A640BF;
const unsigned long R1_VolPlus   = 0x59A642BD;
const unsigned long R1_VolMinus  = 0x59A652AD;
const unsigned long R1_Stop  = 0x59A6F00F;
const unsigned long R1_Enter = 0x59A64AB5;
const unsigned long R1_Setup = 0x59A62AD5;
