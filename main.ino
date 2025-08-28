#include "GCode.hpp"
#define LIMIT_SWITCH
#define DEBOUNCE_DELAY_MS 500

#define LEFT_INTERRUPT_PIN 18
#define RIGHT_INTERRUPT_PIN 19
#define BOTTOM_INTERRUPT_PIN 20
#define TOP_INTERRUPT_PIN 21


//motor set up 
#define E1 5
#define M1 4
#define E2 6
#define M2 7

// Encoder setup
#define RENCA 2
#define RENCB 10
#define LENCB 11
#define LENCA 3

float K_p = 20;
float K_i = 0;
float K_d = 0;

double VELOCITYDELAY = 1; //1/24 of a second try 1/8 if too fast
int CTCTIMER = VELOCITYDELAY*15624; //Every 1 second 15624 
double GEARRATIO = 171.79;
double COUNTTODISTANCERATIO = 65.62; // For 1mm 65.61 counts are needed

// limit switches 
bool left_hit = false;
bool right_hit = false;
bool top_hit = false;
bool bottom_hit = false;

bool error_flag = false;

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

// Variables for homing routine 
bool homing = false; 
enum directions {
  right,
  top
};

//-----------SET UP-----------//
void setup() {
  pinMode(LEFT_INTERRUPT_PIN, INPUT);
  pinMode(RIGHT_INTERRUPT_PIN, INPUT);
  pinMode(TOP_INTERRUPT_PIN, INPUT);
  pinMode(BOTTOM_INTERRUPT_PIN, INPUT);

  pinMode(RENCB, INPUT);
  pinMode(LENCB, INPUT);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_PIN), left_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_PIN), right_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(TOP_INTERRUPT_PIN), top_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_INTERRUPT_PIN), bottom_limit_switch_hit, RISING);

  attachInterrupt(digitalPinToInterrupt(RENCA), RENCA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA), LENCA_ISR, RISING);
}

/////////////////////////////////////////////////////////////////////
//-------------Finite state machine---------------///////////////////
void loop() {
  String command;
  // Put homing command here to run before anything happens (on boot up)
  //Homing();

  while (1) {
    switch (STATE) {
      case IDLE: {
        error_flag= false;
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
        Serial.println("Running homing routine");
        homing = true; 
        Homing();
        Serial.println("left homing");
        if (!error_flag){
          STATE = IDLE;
        }
        break;
      }
      case MOVING: {
        Serial.println("Running moving routine");
        bool left_hit = false;
        bool right_hit = false;
        bool top_hit = false;
        bool bottom_hit = false;
        
        //Find the delta A (left motor) and delta B (right motor) as the inputs to PID function
        delta_A_ref = inputs_to_encoder_count_delta_A(Parser.GetParameters()[0], Parser.GetParameters()[1]);
        delta_B_ref = inputs_to_encoder_count_delta_B(Parser.GetParameters()[0], Parser.GetParameters()[1]);

        // PID_control(delta_A_ref, delta_B_ref);
        if (!error_flag){
          STATE = IDLE;
        }
        break;
      }

      case ERROR: {
        Serial.println("State Error");
        int state = 3;
        while (state != 0){
          Serial.println("Enter GCode command");
          while (Serial.available() == 0){
          }
          command = Serial.readStringUntil('\n');  // Read until newline

          state = Parser.ExecuteCommand(command.c_str());
          if (state == 1 | state == 2){
            Serial.println("Cannot run command from error state");
          }
        }
        STATE = IDLE;
        break;
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//----------------Encoder functions----------------//

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
  // Update both relative and absolute counts
    delta_A_count_rel = LDIRECTION ? delta_A_count_rel + 1 : delta_A_count_rel - 1;
    delta_A_count = LDIRECTION ? delta_A_count + 1 : delta_A_count - 1;

    delta_A_rel = countToDistance(delta_A_count_rel);
}

void m2counting()
{
  // Update both relative and absolute counts
    delta_B_count_rel = RDIRECTION ? delta_B_count_rel + 1 : delta_B_count_rel - 1;
    delta_B_count = RDIRECTION ? delta_B_count + 1 : delta_B_count - 1;

    delta_B_rel = countToDistance(delta_B_count_rel);
}

// Check what RENCB is (0/1) when RENCA triggers the external interrupt on pin 3
void RENCA_ISR()
{
    RDIRECTION = digitalRead(RENCB) ? CCW : CW;

    m2counting();
}

// Check what LENCB is (0/1) when LENCA triggers the external interrupt on pin 11
void LENCA_ISR()
{
    LDIRECTION = digitalRead(LENCB) ? CCW : CW;
    m1counting();
}

double countToDistance(int count)
{
  return (count/COUNTTODISTANCERATIO);
}

int distanceToCount(float distance)
{
  return ((int)distance*COUNTTODISTANCERATIO);
}

//----------------PID control----------------//

// Testing for just distance in one axis
void PID_control(float delta_A_ref_in, float delta_B_ref_in) {
  bool running = true;
  uint8_t M1_speed = 0;
  uint8_t M2_speed = 0;
  // Reset the relative encoder counts
  delta_A_count_rel = 0;
  delta_B_count_rel = 0;

  bool delta_A_direction = (delta_A_ref <= 0) ? 1 : 0; // CCW (1) or CW (0)
  bool delta_B_direction = (delta_B_ref <= 0) ? 1 : 0;

  while ((delta_A_ref_in - delta_A_rel) > 1) {
      M1_speed = K_p*(delta_A_ref_in - delta_A_rel);

    // if (delta_A_ref_in - delta_A_rel >= 0) {
    //   delta_A_direction = 0;
    //   digitalWrite(M1, delta_A_direction);
    //   analogWrite(E1, M1_speed);
    // } else {
    //   delta_A_direction = 1;
    //   digitalWrite(M1, delta_A_direction);
    //   analogWrite(E1, M1_speed);
    // }

    M2_speed = K_p*(delta_B_ref_in - delta_B_rel);
    if (delta_B_ref_in - delta_B_rel >= 0) {
      digitalWrite(M2, delta_B_direction);
      analogWrite(E2, M2_speed);
    } else {
      digitalWrite(M2, !delta_B_direction);
      analogWrite(E2, M2_speed);
    }

    Serial.print(delta_B_ref_in - delta_B_rel);
    Serial.print(" ");
    Serial.print("Direction is: ");
    Serial.println(delta_B_direction);
  }

  Serial.println("Finished P control");
}

//------------------Move funtions------------------//
void move_left(int value) {
  digitalWrite(M1, CW);
  digitalWrite(M2, CW);
  analogWrite(E1, value);
  analogWrite(E2, value);
}

void move_right(int value) {
  digitalWrite(M1, CCW);
  //digitalWrite(M2, CCW);
  analogWrite(E1, value);
  //analogWrite(E2, value);
}

void move_top(int value) {
  digitalWrite(M1, CCW);
  digitalWrite(M2, CW);
  analogWrite(E1, value);
  analogWrite(E2, value);
}

void move_bottom(int value) {
  digitalWrite(M1, CW);
  digitalWrite(M2, CCW);
  analogWrite(E1, value);
  analogWrite(E2, value);
}

//--------------Homing back_up function------------//
void back_up(int direction){
  Serial.println("in back up");
  delta_A_count_rel = 0;
  delta_A_rel = 0;
  // direction = 0 -> top,  direction = 1 -> right
  if (direction == 1){
    move_top(100);
    while (delta_A_rel > -10 & !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
  } else if (direction == 0) {
    move_right(100);
    while (delta_A_rel > -10 & !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
  }
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

//---------------HOMING function--------------//
void Homing() {
  homing = true;

  if (digitalRead(LEFT_INTERRUPT_PIN) == 0){
    left_hit = false;
    if (error_flag){return;}
    move_left(200);
    while(!left_hit & !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
    if (error_flag){return;}
    back_up(0); // Right
    left_hit = false; //reset
    if (error_flag){return;}
    move_left(100);
    while(!left_hit & !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
    left_hit = false; // reset
  }
  
  if (digitalRead(BOTTOM_INTERRUPT_PIN) == 0){
    bottom_hit = false;
    if (error_flag){return;}
    move_bottom(200);
    while(!bottom_hit & !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
    if (error_flag){return;}
    back_up(1); // Top
    bottom_hit = false; // reset
    if (error_flag){return;}
    move_bottom(100);
    while(!bottom_hit & !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
    bottom_hit = false; // reset
  }

  // homing complete
  currentX = 0;
  currentY = 0;
  homing = false;
}

//-------------MOVING function-------------//
void moving(float x, float y, float v){
  float A = inputs_to_encoder_count_delta_A(x, y);
  float B = inputs_to_encoder_count_delta_B(x, y);

  int M1_direction = CCW;
  int M2_direction = CCW;

  bool A_complete = false;
  bool B_complete = false;

  bool positive_A = false;
  bool positive_B = false;

  delta_A_count_rel = 0;
  delta_A_rel = 0;
  delta_B_count_rel = 0;
  delta_B_rel = 0;

  if (A > 0){ //CCW motor 1
    M1_direction = CCW;
    positive_A = true;
  }
  else if (A < 0){
    M1_direction = CW;
  }
  if (B > 0){ //CCW motor 2
    M2_direction = CCW;
    positive_B = true;
  }
  else if (B < 0){
    M2_direction = CW;
  }
  //start motors
  digitalWrite(M1, M1_direction);
  digitalWrite(M2, M2_direction);
  analogWrite(E1, v);
  analogWrite(E2, v);

  // stop motors on complete movement in each direction 
  while (!A_complete | !B_complete){
    if (positive_A){
      if (delta_A_rel >= A){
        analogWrite(E1, 0);
        A_complete = true;
      }
    }
    else{
      if (delta_A_rel <= A){
        analogWrite(E1, 0);
        A_complete = true;
      }
    }
    if (positive_B){
      if (delta_B_rel >= A){
        analogWrite(E1, 0);
        B_complete = true;
      }
    }
    else{
      if (delta_B_rel <= A){
        analogWrite(E1, 0);
        B_complete = true;
      }
    }
  }
}

//-----------Limit switch ISRs------------//

void left_limit_switch_hit() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
    //Serial.println(bottom_hit);
    if (!left_hit){
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }
    left_hit = true;
    if (!homing){
      error_flag = true;
      STATE = ERROR;
    }
  }
  left_last_time = left_now;
}

void right_limit_switch_hit() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    right_hit = true;
    STATE = ERROR;
    error_flag = true;
  }
  right_last_time = right_now;
}

void top_limit_switch_hit() {
  top_now = millis();
  if(top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top limit switch hit");
    analogWrite(E1, 0);
    analogWrite(E2, 0);
    top_hit = true;
    STATE = ERROR;
    error_flag = true;
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
    if (!homing){
      STATE = ERROR;
      error_flag = true;
    }
  }
  bottom_last_time = bottom_now;
}
