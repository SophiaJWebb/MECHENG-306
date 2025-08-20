#ifndef DICTIONARY_GUARD
#define DICTIONARY_GUARD

#define DEBOUNCE_DELAY_MS 2000

#define LEFT_INTERRUPT_PIN 19
#define RIGHT_INTERRUPT_PIN 18
#define TOP_INTERRUPT_PIN 20
#define BOTTOM_INTERRUPT_PIN 21

enum state {
  IDLE,
  HOMING,
  MOVING,
  ERROR
};

//motor set up 
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

const int CCW  = HIGH;
const int CW = LOW;

bool left_hit = false;
bool right_hit = false;
bool top_hit = false;
bool bottom_hit = false;

int absolute_x_mm = 0;
int absolute_y_mm = 0;

#endif 