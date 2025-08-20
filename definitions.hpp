#ifndef DICTIONARY_GUARD
#define DICTIONARY_GUARD

#define DEBOUNCE_DELAY_MS 500

#define LEFT_INTERRUPT_PIN 18
#define RIGHT_INTERRUPT_PIN 19
#define TOP_INTERRUPT_PIN 21
#define BOTTOM_INTERRUPT_PIN 20

enum state {
  IDLE,
  PARSING,
  HOMING,
  MOVING,
  ERROR,
  CALIBRATION
};

//motor set up 
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

const int CCW  = HIGH;
const int CW = LOW;

// limit switches 
bool left_hit = false;
bool right_hit = false;
bool top_hit = false;
bool bottom_hit = false;

//limit switch debouncing 
int long left_last_time = 0;
int long left_now = 0;
int long top_last_time = 0;
int long top_now = 0;

int long right_last_time = 0;
int long right_now = 0;
int long bottom_last_time = 0;
int long bottom_now = 0;

// abosulte x and y position in mm
float currentX;
float currentY;

#endif 