#include "definitions.hpp"

int long left_last_time = 0;
int long left_now = 0;
int long top_last_time = 0;
int long top_now = 0;

int long right_last_time = 0;
int long right_now = 0;
int long bottom_last_time = 0;
int long bottom_now = 0;

bool m1direction=HIGH;
bool m2direction=HIGH;

enum state STATE = IDLE;
bool error_running = false; // Global variable for the limit switch being triggered while NOT in calibration phase
// Global varibale for the limit switch being triggered while in calibration phase
bool left_limit_calibration = false;
bool right_limit_calibration = false;
bool top_limit_calibration = false;
bool bottom_limit_calibration = false;

using namespace std;

void setup() {
  pinMode(LEFT_INTERRUPT_PIN, INPUT);
  pinMode(RIGHT_INTERRUPT_PIN, INPUT);
  pinMode(TOP_INTERRUPT_PIN, INPUT);
  pinMode(BOTTOM_INTERRUPT_PIN, INPUT);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_PIN), left_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_PIN), right_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(TOP_INTERRUPT_PIN), top_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_INTERRUPT_PIN), bottom_limit_switch_hit, RISING);
  
}

void loop() {
  while (1) {
    switch (STATE) {
      case IDLE: {
        // Wait for command
        break;
      }
      case HOMING: {
        // Run homing routine
        break;
      }
      case MOVING: {
        // Move bracket
        break;
      }
      case ERROR: {
        // Fix error then rehome
        break;
      }
      case CALIBRATION: {
        // Calibrate
        break;
      }
    }
  }
}

void left_limit_switch_hit() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != CALIBRATION) {
      error_running = true; // Flag to change state to ERROR
    } else {
      left_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Left limit switch hit");
  }
  left_last_time = left_now;
}

void right_limit_switch_hit() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != CALIBRATION) {
      error_running = true; // Flag to change state to ERROR
    } else {
      right_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Right limit switch hit");
  }
  right_last_time = right_now;
}

void top_limit_switch_hit() {
  top_now = millis();
  if(top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != CALIBRATION) {
      error_running = true; // Flag to change state to ERROR
    } else {
      top_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Top limit switch hit");
  }
  top_last_time = top_now;
}

void bottom_limit_switch_hit() {
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != CALIBRATION) {
      error_running = true; // Flag to change state to ERROR
    } else {
      bottom_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Bottom limit switch hit");
  }
  bottom_last_time = bottom_now;
}