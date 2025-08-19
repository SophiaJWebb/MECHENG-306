#include "GCode.hpp"
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

float currentX;
float currentY;

enum state STATE = IDLE;
bool error_running = false; // Global variable for the limit switch being triggered while NOT in calibration phase
// Global varibale for the limit switch being triggered while in calibration phase
bool left_limit_calibration = false;
bool right_limit_calibration = false;
bool top_limit_calibration = false;
bool bottom_limit_calibration = false;

GCodeParser Parser;

using namespace std;

void setup() 
{
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
    String command;
    switch (STATE) {
      case IDLE: {
        Serial.println("State Idle");
        Serial.println("Enter GCode command");
        while (Serial.available() == 0){
        }
        command = Serial.readStringUntil('\n');  // Read until newline
        STATE = PARSING;
        break;
      }

      case PARSING: {
        int state = Parser.ExecuteCommand(command.c_str());
        if (state == 0){
          STATE = IDLE;
          break;
        }
        if (state == 1){
          STATE = HOMING;
          break;
        }
        if (state == 2){
          if (Parser.ValidateParameters(currentX, currentY)){
            STATE = MOVING;
          }
          else {
            STATE = IDLE;
          }
          break;
        }
        break;
      }
      
      case HOMING: {
        // Run homing routine
        Serial.println("Running homing routine");
       //Move down
    digitalWrite(M1, CW);
    digitalWrite(M2, CCW);
    analogWrite(E1, 100);  // Can increase speed later
    analogWrite(E2, 100);
    // move until bottom switch is pressed
    while (!bottom_hit){
      Serial.println(bottom_hit);
    }
    Serial.println("exited while loop");
    bottom_hit = false;
    digitalWrite(M1, CW);
    digitalWrite(M2, CCW);
    analogWrite(E1, 255);  
    analogWrite(E2, 255);
    delay(1000); // change from delay 
    //back up until encoder count is tbd 
    digitalWrite(M1, CCW);
    digitalWrite(M2, CW);
    analogWrite(E1, 100);  
    analogWrite(E2, 100);
    while (!bottom_hit){
    }
    bottom_hit = false;
    // Set absolute_x 0

    // //move left
    // digitalWrite(M1, CCW);
    // digitalWrite(M2, CCW);
    // analogWrite(E1, 100);  // Can increase speed later
    // analogWrite(E2, 100);
    // // move until left switch is pressed
    // while (!left_hit){
    // }
    // left_hit = false;
    // digitalWrite(M1, CCW);
    // digitalWrite(M2, CCW);
    // analogWrite(E1, 70);  
    // analogWrite(E2, 70);
    // delay(1000); // change from delay 
    // //back up until encoder count is tbd 
    // digitalWrite(M1, CCW);
    // digitalWrite(M2, CCW);
    // analogWrite(E1, 70);  
    // analogWrite(E2, 70);
    // while (!left_hit){
    // }
    // bottom_hit = false;

        STATE = IDLE;
        break;
      }
      case MOVING: {
        Serial.println("Running moving routine");
        STATE = IDLE;
        break;
      }
      case ERROR: {
        // Fix error then rehome
        break;
      }
    }
  }
}

void left_limit_switch_hit() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != HOMING) {
      error_running = true; // Flag to change state to ERROR
    } else {
      left_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Left limit switch hit");
    left_hit = true;
    // Stop motors
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
  left_last_time = left_now;
}

void right_limit_switch_hit() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != HOMING) {
      error_running = true; // Flag to change state to ERROR
    } else {
      right_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Right limit switch hit");
    right_hit = true;

    // Stop motors
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
  right_last_time = right_now;

  right_hit = true;

  // Stop motors
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

void top_limit_switch_hit() {
  top_now = millis();
  if(top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != HOMING) {
      error_running = true; // Flag to change state to ERROR
    } else {
      top_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Top limit switch hit");
    top_hit = true;

    // Stop motors
    analogWrite(E1, 0);
    analogWrite(E2, 0);

  }
  top_last_time = top_now;

  top_hit = true;
}

void bottom_limit_switch_hit() {
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    if (STATE != HOMING) {
      error_running = true; // Flag to change state to ERROR
    } else {
      bottom_limit_calibration = true; // Flag to tell when switch has been hit while in calibration
    }
    Serial.println("Bottom limit switch hit");
    //Serial.println(bottom_hit);
    bottom_hit = true;
    // Stop motors
    analogWrite(E1, 0);
    analogWrite(E2, 0);
  }
  bottom_last_time = bottom_now;
}
