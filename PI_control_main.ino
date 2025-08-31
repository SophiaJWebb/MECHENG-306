// =================== HEADERS ===================
#include "Encoder.h"
#include "GCode.hpp"

// =================== CONFIG / DEFINES ===================
#define LIMIT_SWITCH
#define DEBOUNCE_DELAY_MS 500

// ----- Board: Arduino MEGA external-interrupt pins: 2,3,18,19,20,21 -----
// FIX: move limit switches to real interrupt pins on MEGA
#define LEFT_INTERRUPT_PIN   19
#define RIGHT_INTERRUPT_PIN  18
#define TOP_INTERRUPT_PIN    20
#define BOTTOM_INTERRUPT_PIN 21

// Motor driver pins
#define E2 5
#define M2 4  // Right DIR
#define E1 6
#define M1 7  // Left  DIR

// Encoder pins (A must be interrupt-capable; B can be any digital)
#define RENCA 2  // Right encoder A (interrupt capable)
#define RENCB 8  // Right encoder B
#define LENCA 3  // Left  encoder A (interrupt capable)
#define LENCB 9  // Left  encoder B

// If one encoder is mounted flipped, flip its tick sign once here:
const bool L_ENCODER_REVERSED = true;   // FIX: set true/false to make +Y behave

// ======== FEEDFORWARD / FRICTION ========
// Friction PI co-efficients (found using trial and error)
float KVFF_R = 0.04f;
float KVFF_L = 0.04f;
float KF_R = 10.0f;
float KF_L = 10.0f;

// Minimum PWM value the motors can take
float MIN_PWM = 0.0f;

// =================== GAINS (PI on velocity) ===================
float K_p = 0.05f;
float K_i = 0.02f;

// =================== MECHANICS / UNITS ===================
const float COUNTTODISTANCERATIO = 43.74f;  // encoder counts per mm

// =================== STATE FLAGS (limit switches) ===================
volatile bool left_hit = false;
volatile bool right_hit = false;
volatile bool top_hit = false;
volatile bool bottom_hit = false;

// limit switch debouncing
unsigned long left_last_time = 0, left_now = 0;
unsigned long right_last_time = 0, right_now = 0;
unsigned long top_last_time = 0, top_now = 0;
unsigned long bottom_last_time = 0, bottom_now = 0;

bool error_flag = false;

bool homing = false;
bool justfinishedhoming=false;


enum directions {
  right,
  top
};

// For motor control
enum direction {
  CW,
  CCW
};

// =================== APP STATE ===================
float currentX = 0;
float currentY = 0;
int motorRDirection = 1;
int motorLDirection = 1;

GCodeParser Parser;
Encoder motorR;  // Right
Encoder motorL;  // Left

// FIX: remove CW=0 / CCW=1 confusion; use explicit +/-1 signs when ticking.
// (Direction pins still use HIGH/LOW via driveMotor())

enum state { IDLE, PARSING, HOMING, MOVING, ERROR, CALIBRATION };
state STATE = IDLE;

// ========== MOTION PROFILE ==========
float VMAX_MMPS = 50.0f;
float ACCEL_MMPS2 = 200.0f;

inline float vmax_cps() { return VMAX_MMPS * COUNTTODISTANCERATIO; }
inline float accel_cps2() { return ACCEL_MMPS2 * COUNTTODISTANCERATIO; }

// Control-loop timing
const float LOOP_HZ = 100.0f;
const float DT = 1.0f / LOOP_HZ;

// Stop tolerances
const long  POS_TOL_COUNTS = 10;
const float VEL_TOL_CPS    = 100.0f;

// =================== HELPERS ===================
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float sgnf(float x) { return (x > 0) - (x < 0); }

static inline void driveMotor(int pinDir, int pinPwm, float u) {
  if (fabs(u) < 1.0f) { analogWrite(pinPwm, 0); return; }
  int dir = (u >= 0.0f) ? HIGH : LOW;
  int pwm = (int)clampf(fabs(u), 0.0f, 255.0f);
  digitalWrite(pinDir, dir);
  analogWrite(pinPwm, pwm);
}

// Trapezoid in counts (remain in counts, v_cmd in counts/s)
static inline float stepTrapezoidCounts(float v_cmd_cps, float remain_counts) {
  float s = sgnf(remain_counts);
  float v = fabs(v_cmd_cps);
  float vmax = vmax_cps();
  float a = accel_cps2();
  float d = fabs(remain_counts);
  float d_brake = (v * v) / (2.0f * a + 1e-9f);

  if (d <= 1.0f)        v = 0.0f;
  else if (d <= d_brake) v = max(0.0f, v - a * DT);
  else                   v = min(vmax, v + a * DT);

  return v * s;
}

// =================== FORWARD DECLS ===================
void left_limit_switch_hit();
void right_limit_switch_hit();
void top_limit_switch_hit();
void bottom_limit_switch_hit();
void RENCA_ISR();
void LENCA_ISR();
void Homing();
void PI_control(long delta_R_ref_in, long delta_L_ref_in);
float inputs_to_encoder_count_delta_L(float delta_X, float delta_Y);
float inputs_to_encoder_count_delta_R(float delta_X, float delta_Y);

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  // ----- Limit switches -----
  // FIX: use INPUT_PULLUP + FALLING (wire switch to GND)
  pinMode(LEFT_INTERRUPT_PIN,   INPUT_PULLUP);
  pinMode(RIGHT_INTERRUPT_PIN,  INPUT_PULLUP);
  pinMode(TOP_INTERRUPT_PIN,    INPUT_PULLUP);
  pinMode(BOTTOM_INTERRUPT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_PIN),
                  left_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_PIN),
                  right_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(TOP_INTERRUPT_PIN),
                  top_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_INTERRUPT_PIN),
                  bottom_limit_switch_hit, RISING);

  // ----- Encoders -----
  pinMode(RENCA, INPUT_PULLUP);
  pinMode(RENCB, INPUT_PULLUP);
  pinMode(LENCA, INPUT_PULLUP);
  pinMode(LENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RENCA), RENCA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA), LENCA_ISR, RISING);

  // ----- Motors -----
  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);
  analogWrite(E1, 0);
  analogWrite(E2, 0);

  interrupts();
}

// =================== LOOP / STATE MACHINE ===================
void loop() {
    // Create string to store inputted GCode command
  String command;

  // Finite State Machine
  while (1) {
    switch (STATE) {
      case IDLE: {
        // Reset all flags in IDLE state
        error_flag= false;
        left_hit= false;
        right_hit=false;
        top_hit=false;
        bottom_hit=false;

        // Force motors to zero (fail safe)
        driveMotor(M1, E1, 0);
        driveMotor(M2, E2, 0);
        Serial.println("State Idle");
        Serial.println("Enter GCode command");

        // Wait for a gcode command via terminal
        while (Serial.available() == 0) { /* wait */ }
        command = Serial.readStringUntil('\n');
        STATE = PARSING;
        break;
      }

      case PARSING: {
        // Read and execute command from parser
        int st = Parser.ExecuteCommand(command.c_str());
        if (st == 0) { STATE = IDLE; break; }
        if (st == 1) { STATE = HOMING; break; }
        if (st == 2) {
            // If moving state validate parameters then go to MOVING state
          Serial.print("X: ");
          Serial.print(currentX);
          Serial.print("Y: ");
          Serial.println(currentY);
          if (Parser.ValidateParameters(currentX, currentY)){
            STATE = MOVING; break;
          }
          else {
            STATE = IDLE; break;
          }
        }
        // ERROR state if state is not any of the enumerated states
        STATE = ERROR;
        break;
      }

      case HOMING: {
        Serial.println("Running homing routine");
        // Set homing bool to true
        homing = true;
        
        // Run homing function
        Homing();
        Serial.println("left homing");

        // Set the global absolute X and Y positions after homing
        double currentX=(motorL.getAbsolutemm()+motorR.getAbsolutemm())/2;
        double currentY=(motorL.getAbsolutemm()-motorR.getAbsolutemm())/2;
        Serial.print("X: ");
        Serial.print(currentX);
        Serial.print("Y: ");
        Serial.println(currentY);
        
        // Check if error occurred during homing routine
        if (!error_flag) {
            STATE = IDLE;
        }
        break;
      }

      case MOVING: {
        Serial.println("Running moving routine");

        // Parser: ΔX, ΔY in mm. If your Y was inverted, keep the minus.
        float dX_mm = Parser.GetParameters()[0];
        float dY_mm = Parser.GetParameters()[1];
        Serial.print("dX mm: "); Serial.println(dX_mm);
        Serial.print("dY mm: "); Serial.println(dY_mm);

        // Convert to per-motor deltas (mm)
        float dR_mm = inputs_to_encoder_count_delta_R(dX_mm, dY_mm);
        float dL_mm = inputs_to_encoder_count_delta_L(dX_mm, dY_mm);

        Serial.print("dR mm: "); Serial.println(dR_mm);
        Serial.print("dL mm: "); Serial.println(dL_mm);

        // mm to counts for right motor
        long delta_R_ticks = (long)roundf(dR_mm * COUNTTODISTANCERATIO);
        // Find direction of right motor
        motorR.setDirection(delta_R_ticks/abs(delta_R_ticks));

        // mm to counts for left motor
        long delta_L_ticks = (long)roundf(dL_mm * COUNTTODISTANCERATIO);\
        // Find direction of left motor
        motorL.setDirection(delta_L_ticks/abs(delta_L_ticks));

        Serial.print("R Direction: "); Serial.println(delta_R_ticks/abs(delta_R_ticks));
        Serial.print("L Direction: "); Serial.println(delta_L_ticks/abs(delta_L_ticks));

        Serial.print("deltaR tick: "); Serial.println(delta_R_ticks);
        Serial.print("deltaL tick: "); Serial.println(delta_L_ticks);

        // Run PI control function
        PI_control(delta_R_ticks, delta_L_ticks);
        justfinishedhoming=false;

        // Update the absolute X and Y positions after moving
        currentX=(motorL.getAbsolutemm()+motorR.getAbsolutemm())/2;
        currentY=(motorL.getAbsolutemm()-motorR.getAbsolutemm())/2;

        // Check if error occurred during moving
        if (!error_flag){
          STATE = IDLE;
        }

        break;
      }

      case ERROR: {
        Serial.println("State Error");
        // Force to error state
        int state = 3;

        // Wait for a gcode command via terminal
        while (state != 0){
          Serial.println("Enter GCode command");
          while (Serial.available() == 0){
          }
          command = Serial.readStringUntil('\n');  // Read until newline

          // Check if the command is not M999 and if it isn't then dont execute the command
          state = Parser.ExecuteCommand(command.c_str());
          if (state == 1 | state == 2){
            Serial.println("Cannot run command from error state");
          }
        }
        STATE = IDLE;
        break;
    }

    // Default to IDLE state
      default: STATE = IDLE; break;
    }
  }
}

// =================== KINEMATICS (mm) ===================
float inputs_to_encoder_count_delta_L(float delta_X, float delta_Y) {
  return (delta_X + delta_Y);
}
float inputs_to_encoder_count_delta_R(float delta_X, float delta_Y) {
  return (delta_X - delta_Y);
}

// =================== ENCODER ISRs ===================
// Right encoder A rising: read B; assume B=HIGH means one direction.
void RENCA_ISR() {
  motorR.countTicks(motorR.getDirection());
}
// Left encoder A rising: read B; mirror right, then optionally flip once.
void LENCA_ISR() {
  motorL.countTicks(motorL.getDirection());
}

// =================== LIMIT SWITCH ISRs ===================

// The left limit switch should only change to ERROR state if it is hit not during homing routine
void left_limit_switch_hit() {
  left_now = millis();
  // Debouncing of 500ms
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
    if (!justfinishedhoming){
      if (!left_hit){
        // Force motors to zero if left limit switch is hit not during homing
        analogWrite(E1, 0);
        analogWrite(E2, 0);
      }
      // Set left_hit to true for the homing routine
      left_hit = true;

      // Check if the state is not homing since if it isn't then an error has occured
      if (!homing){
        error_flag = true;
        STATE = ERROR;
      }
    }
    left_last_time = left_now;
  }
}

// The right limit switch should always change to ERROR state if it is hit at all regardless of what state
// it is in
void right_limit_switch_hit() {
  right_now = millis();
  // Debouncing of 500ms
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
    // Force motors to zero if right limit switch is hit at all
    analogWrite(E1, 0);
    analogWrite(E2, 0);

    // Set right_hit to true to transistion to ERROR state
    right_hit = true;
    STATE = ERROR;
    error_flag = true;
  }
  right_last_time = right_now;
}

// The top limit switch should always change to ERROR state if it is hit at all regardless of what state
// it is in
void top_limit_switch_hit() {
  top_now = millis();
  // Debouncing of 500ms
  if(top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top limit switch hit");
    // Force motors to zero if top limit switch is hit at all
    analogWrite(E1, 0);
    analogWrite(E2, 0);

    // Set top_hit to true to transition to ERROR state
    top_hit = true;
    STATE = ERROR;
    error_flag = true;
  }
  top_last_time = top_now;
}

// The bottom limit switch should only change to ERROR state if it is hit not during homing routine
void bottom_limit_switch_hit() {
  bottom_now = millis();
  // Debouncing of 500ms
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Bottom limit switch hit");
  if (!justfinishedhoming){
    if (!bottom_hit){
        // Force motors to zero if left limit switch is hit not during homing
      analogWrite(E1, 0);
      analogWrite(E2, 0);
    }

    // Set bottom_hit to true for the homing routine
    bottom_hit = true;

    // If the state is not in homing and the bottom limit switch is hit then change to ERROR state
    if (!homing){
      STATE = ERROR;
      error_flag = true;
    }
  }
  }
  bottom_last_time = bottom_now;
}

// direction = 1 is top bottom homing, direction = 0 is left right homing
void back_up(int direction){
  Serial.println("in back up");
  motorL.resetEncoder();
  motorL.setDirection(1);
  // direction = 0 -> top,  direction = 1 -> right

  if (direction == 1){
    // Move to the top for the equivalent of 15mm in encoder counts
    move_top(100);
    while (motorL.convertTicksToMillimeters(motorL.getEncoderTicks()) < 15 && !error_flag){
      asm("nop");

      // Check for an error during this move
      if (error_flag){return;}
    }
  } else if (direction == 0) {
    // Move to the right for the equivalent of 15mm in encoder counts
    move_right(100);
    while (motorL.convertTicksToMillimeters(motorL.getEncoderTicks()) < 15 && !error_flag){
      asm("nop");
      // Check for error during this move
      if (error_flag){return;}
    }
  }
  // Set motors to zero
  driveMotor(M2, E2, 0);
  driveMotor(M1, E1, 0);
}

void Homing() {
  homing = true;

  // ==================Left right homing======================
    left_hit = false;

    // Check for error before moving
    if (error_flag) {return;}

    // Move to the left at a PWM of 200
    move_left(200);
    // Wait for the left limit switch to be hit or an error to occur
    while(!left_hit && !error_flag) {
      asm("nop");
      if (error_flag) {return;}
    }

    // Check if an error occurred
    if (error_flag) {return;}

    // Back up to the right for 15 mm
    back_up(0); // Right
    left_hit = false; //reset

    // Check if an error has occurred after the back up command
    if (error_flag) {return;}

    // Move left again at a PWM of 80 (almost minimum PWM value so very slow) until left limit switch is hit
    move_left(80);
    while(!left_hit && !error_flag) {
      asm("nop");
      if (error_flag) {return;}
    }
    left_hit = false;
    // =================================================================

// ====================Top bottom homing==========================
    bottom_hit = false;

    // Check for error before moving
    if (error_flag) {return;}

    // Move to the bottom at a PWM of 200
    move_bottom(200);
    while(!bottom_hit && !error_flag) {
      asm("nop");
      if (error_flag) {return;}
    }

    // Check if an error occurred
    if (error_flag) {return;}

    // Back up to the top for 15mm
    back_up(1); // Top
    bottom_hit = false; // reset
    if (error_flag) {return;}

    // Reapproach the bottom at PWM of 80 (close to minimum PWM value so very slow)
    // until bottom limit switch is hit
    move_bottom(80);
    while(!bottom_hit && !error_flag){
      asm("nop");
      if (error_flag){return;}
    }
    bottom_hit = false;
    // =========================================================

  // homing complete
  // Set absolute X and Y global variables to 0
  currentX = 0;
  currentY = 0;

  // Reset the encoder counts
  motorR.hardResetEncoder();
  motorL.hardResetEncoder();

  // Reset the homing bools
  homing = false;
  justfinishedhoming=true;
}

// =================== PI-ONLY MOVE (counts + counts/s) ===================
void PI_control(long delta_R_ref_in, long delta_L_ref_in) {
    // Reset encoders
  motorR.resetEncoder();
  motorL.resetEncoder();

  // Set the target in counts for the right and left motors (delta A = left motor and delta B = right motor)
  const long targetRcount = delta_R_ref_in;
  const long targetLcount = delta_L_ref_in;

  // Calcutate the total distance to move in counts
  const float total_distance =
      sqrtf((float)targetRcount * (float)targetRcount +
            (float)targetLcount * (float)targetLcount);

    // Checking if the distance is too small to move
  if (total_distance <= 0.5f) {
    driveMotor(M2, E2, 0);
    driveMotor(M1, E1, 0);
    Serial.println("done");
    return;
  }

  // Find the unit vectors for the left and right motors (u = v/mag(v))
  const float unitR = (float)targetRcount / total_distance;
  const float unitL = (float)targetLcount / total_distance;

  // Last R and L value in previous iteration of loop
  long  lastR = 0, lastL = 0;

  // Left and right counts per second
  float vR_cps = 0.0f, vL_cps = 0.0f;
  float vCmd_cps = 0.0f;
  float vCmdR_cps = 0.0f, vCmdL_cps = 0.0f;

  // Intergal accumulation
  float iR = 0.0f, iL = 0.0f;

  // Max. values that the PWM input to the motors can take
  const float UMAX  = 255.0f;

  // Anti-windup for the intergral
  const float I_MAX = (K_i > 0.0f) ? (255.0f / K_i) : 0.0f;

  // Previous velocity for the left and right for each previous iteration of the loop
  static float vCmdR_prev = 0.0f, vCmdL_prev = 0.0f;

  // Keep track of time
  unsigned long tNext = millis();

  // Loop runs at 100 Hz
  for (;;) {
    unsigned long now = millis();
    if ((long)(now - tNext) < 0) continue;
    tNext += (unsigned long)(1000.0f * DT);

    // Calculate the encoder ticks for the left and right motors
    long cR = motorR.getEncoderTicks();
    long cL = motorL.getEncoderTicks();

    // Check for error during the PI control movement
    if (left_hit || right_hit || top_hit || bottom_hit) {
      driveMotor(M2, E2, 0);
      driveMotor(M1, E1, 0);
      Serial.println("ABORT: limit switch hit");
      return;
    }

    // Find increments of encoder counts of left and right motors
    long dR = cR - lastR; lastR = cR;
    long dL = cL - lastL; lastL = cL;

    // Find the counts per second of the left and right motors
    vR_cps = (float)dR / DT;
    vL_cps = (float)dL / DT;

    // Find the total counts so far
    float prog_counts      = cR * unitR + cL * unitL;

    // Find the remaining counts
    float remTotal_counts  = total_distance - prog_counts;

    // Determine what the velocity should be using the trapeziodal velocity profile
    vCmd_cps = stepTrapezoidCounts(vCmd_cps, remTotal_counts);

    // Find the left and right velocity in terms of their unit vectors
    vCmdR_cps = vCmd_cps * unitR;
    vCmdL_cps = vCmd_cps * unitL;

    // Find the error in the left and right velocities in cps
    float eR = vCmdR_cps - vR_cps;
    float eL = vCmdL_cps - vL_cps;

    // Set the previous velocity for the next iteration
    vCmdR_prev = vCmdR_cps;
    vCmdL_prev = vCmdL_cps;

    // Accounting for friction
    float ffR = KVFF_R * vCmdR_cps;
    float ffL = KVFF_L * vCmdL_cps;
    if (fabs(vCmdR_cps) > 1.0f) ffR += KF_R * sgnf(vCmdR_cps);
    if (fabs(vCmdL_cps) > 1.0f) ffL += KF_L * sgnf(vCmdL_cps);

    // Find the PI control values for velocity
    float uR_pi = K_p * eR + K_i * iR;
    float uL_pi = K_p * eL + K_i * iL;

    // Find the total PI PWM values including friction
    float uR = uR_pi + ffR;
    float uL = uL_pi + ffL;

    // Limit from -255 to 255
    float uR_clamped = clampf(uR, -UMAX, UMAX);
    float uL_clamped = clampf(uL, -UMAX, UMAX);

    // simple conditional integration
    if (K_i > 0.0f) {
      if (fabs(uR - uR_clamped) < 1e-3f || sgnf(eR) != sgnf(uR)) {
        iR = clampf(iR + eR * DT, -I_MAX, I_MAX);
      }
      if (fabs(uL - uL_clamped) < 1e-3f || sgnf(eL) != sgnf(uL)) {
        iL = clampf(iL + eL * DT, -I_MAX, I_MAX);
      }
    }

    driveMotor(M1, E1, uR_clamped);  // Right
    driveMotor(M2, E2, uL_clamped);  // Left

    // Stop motors if within some tolerance of target position
    bool totalDone = (fabs(remTotal_counts) <= POS_TOL_COUNTS);
    if (totalDone) {
      Serial.println("done");
      break;
    }

    static int div = 0;
    if (++div >= 10) {
      div = 0;
      long remR_counts = targetRcount - cR;
      long remL_counts = targetLcount - cL;
      float posR_mm = (float)cR / COUNTTODISTANCERATIO;
      float posL_mm = (float)cL / COUNTTODISTANCERATIO;
      float vR_mmps = vR_cps / COUNTTODISTANCERATIO;
      float vL_mmps = vL_cps / COUNTTODISTANCERATIO;

      Serial.print("R "); Serial.print(posR_mm, 2);
      Serial.print(" mm  v "); Serial.print(vR_mmps, 1);
      Serial.print(" mm/s  rem "); Serial.print(remR_counts);
      Serial.print("  u "); Serial.print(uR_clamped, 1);
      Serial.print(" | L "); Serial.print(posL_mm, 2);
      Serial.print(" mm  v "); Serial.print(vL_mmps, 1);
      Serial.print(" mm/s  rem "); Serial.print(remL_counts);
      Serial.print("  u "); Serial.println(uL_clamped, 1);
    }
  }

  // Drive motors to zero
  driveMotor(M2, E2, 0);
  driveMotor(M1, E1, 0);
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
  digitalWrite(M2, CCW);
  analogWrite(E1, value);
  analogWrite(E2, value);
}

void move_top(int value) {
  digitalWrite(M1, CW);
  digitalWrite(M2, CCW);
  analogWrite(E1, value);
  analogWrite(E2, value);
}

void move_bottom(int value) {
  digitalWrite(M1, CCW);
  digitalWrite(M2, CW);
  analogWrite(E1, value);
  analogWrite(E2, value);
}
