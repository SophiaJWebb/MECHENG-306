#ifndef DICTIONARY_GUARD
#define DICTIONARY_GUARD

#define DEBOUNCE_DELAY_MS 500

#define LEFT_INTERRUPT_PIN 18
#define RIGHT_INTERRUPT_PIN 19
#define TOP_INTERRUPT_PIN 21
#define BOTTOM_INTERRUPT_PIN 20

enum state {
  IDLE,
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

#endif 