#include "GCode.hpp"
#define LIMIT_SWITCH
#define DEBOUNCE_DELAY_MS 500

#define LEFT_INTERRUPT_PIN 18
#define RIGHT_INTERRUPT_PIN 19
#define TOP_INTERRUPT_PIN 21
#define BOTTOM_INTERRUPT_PIN 20

//motor set up 
#define E2 5
#define M2 4 // Right
#define E1 6
#define M1 7 // Left

// Encoder setup
#define RENCA 3
#define RENCB 2
#define LENCB 10
#define LENCA 11

float K_p = 0;
float K_i = 0;
float K_d = 0;

double VELOCITYDELAY = 1; //1/24 of a second try 1/8 if too fast
int CTCTIMER = VELOCITYDELAY*15624; //Every 1 second 15624 
double GEARRATIO = 171.79;
double COUNTTODISTANCERATIO = 65.618946; // For 1mm 65.61 counts are needed

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

// These all change during moving or homing command
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// These variables only change during moving command only and is reset to 0 each time the moving command is run
// ***************************************************
// Relative distance moved from start of moving command (use for PID control)
float delta_A_rel = 0;
float delta_B_rel = 0;

int long delta_A_count_rel = 0;
int long delta_B_count_rel = 0;
// ****************************************************

int long delta_A_count = 0;
int long delta_B_count = 0;

//Delta A is left motor, delta B is right motor
float delta_A_position_absolute = 0;
float delta_B_position_absolute = 0;

float currentX = 0;
float currentY = 0;

// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

float delta_A_ref = 0;
float delta_B_ref = 0;

GCodeParser Parser;

//  Guess CW is when ENCB = LOW and CCW is when ENCB = HIGH
enum direction {
  CW,
  CCW
};

enum state {
  IDLE,
  PARSING,
  HOMING,
  MOVING,
  ERROR,
  CALIBRATION
};

enum direction LDIRECTION = CCW;
enum direction RDIRECTION = CCW;

enum state STATE = IDLE;

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

  attachInterrupt(digitalPinToInterrupt(RENCA), RENCA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA), LENCA_ISR, RISING);
}

void loop() {
  String command;
  // Put homing command here to run before anything happens (on boot up)
  Homing();

  while (1) {
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
      STATE = IDLE;
      break;
    }
    case MOVING: {
      Serial.println("Running moving routine");
      // Find the delta A (left motor) and delta B (right motor) as the inputs to PID function
      delta_A_ref = inputs_to_encoder_count_delta_A(Parser.GetParameters()[0], Parser.GetParameters()[1]);
      delta_B_ref = inputs_to_encoder_count_delta_B(Parser.GetParameters()[0], Parser.GetParameters()[1]);


      STATE = IDLE;
      break;
    }

      case ERROR: {
        break;
      }
    }
  }
}

// Function to tell how much to move delta A (in mm) which is the left motor
float inputs_to_encoder_count_delta_A(float delta_X, float delta_Y) {
  return (delta_X + delta_Y);
}

// Function to tell how much to move delta B (in mm) which is the right motor
float inputs_to_encoder_count_delta_B(float delta_X, float delta_Y) {
  return (delta_X - delta_Y);
}

void m1counting()
{
    delta_A_count_rel = RDIRECTION ? delta_A_count + 1 : delta_A_count - 1;

    delta_A_rel = countToDistance(delta_A_count_rel);
}

void m2counting()
{
    delta_B_count_rel = LDIRECTION ? delta_B_count + 1 :delta_B_count - 1;

    delta_B_rel = countToDistance(delta_B_count_rel);
}

// Check what RENCB is (0/1) when RENCA triggers the external interrupt on pin 3
void RENCA_ISR()
{
    RDIRECTION = digitalRead(RENCB) ? CCW : CW;

    m1counting();
}

// Check what LENCB is (0/1) when LENCA triggers the external interrupt on pin 11
void LENCA_ISR()
{
    LDIRECTION = digitalRead(LENCB) ? CCW : CW;
    
    m2counting();
}

double countToDistance(int count)
{
  return (count/COUNTTODISTANCERATIO);
}

int distanceToCount(float distance)
{
  return ((int)distance*COUNTTODISTANCERATIO);
}

// Testing for just distance in one axis
void PID_control(float delta_A_ref_in, float delta_B_ref_in) {
  bool running = true;
  // Reset the relative encoder counts
  delta_A_count_rel = 0;
  delta_B_count_rel = 0;

  delta_A_direction = (delta_A_ref <= 0) ? 1 : 0; // CCW (1) or CW (0)
  delta_B_direction = (delta_B_ref <= 0) ? 1 : 0;

  while (running) {
    
  }
}

void left_limit_switch_hit() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
    if (!left_hit){
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
    left_hit = true;
  }
  left_last_time = left_now;
}

void right_limit_switch_hit() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
    if (!right_hit){
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
    right_hit = true;
  }
  right_last_time = right_now;
}

void top_limit_switch_hit() {
  top_now = millis();
  if(top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    if (!top_hit){
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
    top_hit = true;
  }
  top_last_time = top_now;
}

void bottom_limit_switch_hit() {
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Bottom limit switch hit");
    //Serial.println(bottom_hit);
    if (!bottom_hit){
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
    bottom_hit = true;
  }
  bottom_last_time = bottom_now;
}


void Homing() {
  digitalWrite(M1,CCW);
  digitalWrite(M2,CW);
  analogWrite(E1, 200); //PWM Speed Control
  analogWrite(E2, 200); //PWM Speed Control
  while(!bottom_hit){
    Serial.println("hit: ");
    Serial.print(bottom_hit);
  }
  digitalWrite(M1,CW);
  digitalWrite(M2,CCW);
  analogWrite(E1, 100); //PWM Speed Control
  analogWrite(E2, 100);
  delay(2000);
  bottom_hit = false;
  digitalWrite(M1,CCW);
  digitalWrite(M2,CW);
  analogWrite(E1, 100); //PWM Speed Control
  analogWrite(E2, 100); //PWM Speed Control
  while(!bottom_hit){
    Serial.println("moving");
  }
//left homing
  bottom_hit = false;
  digitalWrite(M1,CW);
  digitalWrite(M2,CW);
  analogWrite(E1, 200); //PWM Speed Control
  analogWrite(E2, 200); //PWM Speed Control
  while(!left_hit){
    Serial.println("hit: ");
    Serial.print(left_hit);
  }
  digitalWrite(M1,CCW);
  digitalWrite(M2,CCW);
  analogWrite(E1, 100); //PWM Speed Control
  analogWrite(E2, 100);
  delay(2000);
  left_hit = false;
  digitalWrite(M1,CW);
  digitalWrite(M2,CW);
  analogWrite(E1, 100); //PWM Speed Control
  analogWrite(E2, 100); //PWM Speed Control
  while(!left_hit){
    Serial.println("moving");
  }
  left_hit = false;

  currentX = 0;
  currentY = 0;
}

#ifndef LIMIT_SWITCH
  void left_limit_switch_hit() {
    left_now = millis();
    if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
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
      Serial.println("Bottom limit switch hit");
      //Serial.println(bottom_hit);
      bottom_hit = true;
      // Stop motors
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
    bottom_last_time = bottom_now;
  }
#endif
