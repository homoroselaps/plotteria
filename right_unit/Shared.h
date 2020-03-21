
const unsigned long step_delay = 100;

byte addresses[][6] = { "Unit1", "Unit2", "Unit3" };
enum AddrIndex : byte { MAIN_UNIT = 0, RIGHT_UNIT = 1, PEN_UNIT = 2 };
enum Direction : bool { FORWARD = true, BACKWARD = false };
enum PenPosition : bool { DOWN = true, UP = false };

int numberOfSteps = 200;
const float step_length = 40.0 / numberOfSteps; // the distance a step increases the string in mm

struct Message {
  byte nonce;
  bool direction;
  unsigned int value;
};

struct Point {
  float x;
  float y;
};
