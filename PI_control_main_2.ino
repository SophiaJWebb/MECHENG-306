// =================== HEADERS ===================
#include "Encoder.h"
#include "GCode.hpp"

// =================== CONFIG / DEFINES ===================
#define LIMIT_SWITCH
#define DEBOUNCE_DELAY_MS 500

// Limit-switch interrupt pins (Mega-friendly)
#define LEFT_INTERRUPT_PIN   18
#define RIGHT_INTERRUPT_PIN  19
#define TOP_INTERRUPT_PIN    21
#define BOTTOM_INTERRUPT_PIN 20

// Motor driver pins
#define E2 6
#define M2 7   // Right DIR
#define E1 5
#define M1 4   // Left  DIR

// Encoder pins
#define RENCA 2   // Right encoder A (interrupt capable)
#define RENCB 10   // Right encoder B
#define LENCB 11  // Left encoder B
#define LENCA 3  // Left encoder A (NOTE: needs an interrupt-capable pin on your board)

// =================== GAINS (PI on velocity) ===================
// Might need different gains for each motor (tune independently)
float K_p_left = 5.0f;
float K_i_left = 0.0f; // Zero for now

float K_p_right = 5.0f;
float K_i_right = 0.0f; // Zero for now

// =================== MECHANICS / UNITS ===================
// counts per mm (you provided): ~65.62 counts/mm
const float COUNTTODISTANCERATIO = 43.74f;  // counts per mm

// =================== STATE FLAGS (limit switches) ===================
volatile bool left_hit = false;
volatile bool right_hit = false;
volatile bool top_hit = false;
volatile bool bottom_hit = false;

// limit switch debouncing
unsigned long left_last_time   = 0, left_now   = 0;
unsigned long right_last_time  = 0, right_now  = 0;
unsigned long top_last_time    = 0, top_now    = 0;
unsigned long bottom_last_time = 0, bottom_now = 0;

// =================== APP STATE ===================
float currentX = 0;
float currentY = 0;

GCodeParser Parser;
Encoder motorR;   // Left
Encoder motorL;   // Right

//  Guess CW is when ENCB = LOW and CCW is when ENCB = HIGH
enum direction { CW = 0, CCW = 1 };
enum state { IDLE, PARSING, HOMING, MOVING, ERROR, CALIBRATION };

volatile direction LDIRECTION = CCW;
volatile direction RDIRECTION = CCW;

long int rel_A_count = 0;
long int rel_B_count = 0;

int rel_A = 0;
int rel_B = 0;

state STATE = IDLE;

// ========== MOTION PROFILE (counts-based; NOT feedforward) ==========
float VMAX_MMPS   = 50.0f;    // max speed (mm/s) — tune
float ACCEL_MMPS2 = 200.0f;   // accel (mm/s^2) — tune

inline float vmax_cps()   { return VMAX_MMPS   * COUNTTODISTANCERATIO; } // counts/s
inline float accel_cps2() { return ACCEL_MMPS2 * COUNTTODISTANCERATIO; } // counts/s^2

// Control-loop timing
const float LOOP_HZ = 100.0f;
const float DT      = 1.0f / LOOP_HZ;

// Stop tolerances
const long  POS_TOL_MM = 1; // mm
const float VEL_TOL_CPS    = 100.0f; // counts/s  (~0.15 mm/s)

// =================== HELPERS ===================
static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float sgnf(float x){ return (x > 0) - (x < 0); }

// Functions for the offset difference between motors
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
static inline void driveMotorLeft(bool direction, uint8_t PWM_value) {
  uint8_t pwm = clampf(abs(PWM_value), 75, 255); // mag -> PWM
  // Serial.print("U: ");
  // Serial.println(u,10);
  // Serial.print("PWM: ");
  // Serial.println(pwm);
  digitalWrite(M1, direction);
  analogWrite(E1, pwm);
}

static inline void driveMotorRight(bool direction, uint8_t PWM_value) {
  uint8_t pwm = clampf(abs(PWM_value), 58, 255); // mag -> PWM
  // Serial.print("U: ");
  // Serial.println(u,10);
  // Serial.print("PWM: ");
  // Serial.println(pwm);
  digitalWrite(M2, direction);
  analogWrite(E2, pwm);
}
// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

static inline void stopMotors() {
  analogWrite(E1, 0);
  analogWrite(E2, 0);
}

// Trapezoid in counts (remain in counts, v_cmd in counts/s)
static inline float stepTrapezoidCounts(float v_cmd_cps, float remain_counts) {
  float s    = sgnf(remain_counts);
  float v    = fabs(v_cmd_cps);    // counts/s
  float vmax = vmax_cps();         // counts/s
  float a    = accel_cps2();       // counts/s^2
  float d    = fabs(remain_counts);// counts
  float d_brake = (v*v) / (2.0f * a + 1e-9f);

  if (d <= 1.0f)            v = 0.0f;
  else if (d <= d_brake)    v = max(0.0f, v - a*DT);
  else                      v = min(vmax,   v + a*DT);

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
void PI_control(long delta_A_ref_in, long delta_B_ref_in);
float inputs_to_encoder_count_delta_A(float delta_X, float delta_Y);
float inputs_to_encoder_count_delta_B(float delta_X, float delta_Y);

// =================== OPTIONAL: DISABLE TIMER1 VELOCITY ISR ===================
// #define USE_TIMER_VEL
#ifdef USE_TIMER_VEL
ISR(TIMER1_COMPA_vect) {
  // Provide your own well-defined velocity bookkeeping here if desired.
}
#endif

// =================== SETUP ===================
void setup() {
  Serial.begin(9600);

  // Limit switches
  pinMode(LEFT_INTERRUPT_PIN,   INPUT);
  pinMode(RIGHT_INTERRUPT_PIN,  INPUT);
  pinMode(TOP_INTERRUPT_PIN,    INPUT);
  pinMode(BOTTOM_INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_PIN),   left_limit_switch_hit,   RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_PIN),  right_limit_switch_hit,  RISING);
  attachInterrupt(digitalPinToInterrupt(TOP_INTERRUPT_PIN),    top_limit_switch_hit,    RISING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_INTERRUPT_PIN), bottom_limit_switch_hit, RISING);

  // Encoders
  pinMode(RENCA, INPUT);
  pinMode(RENCB, INPUT);
  pinMode(LENCA, INPUT);
  pinMode(LENCB, INPUT);

  attachInterrupt(digitalPinToInterrupt(RENCA), RENCA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LENCA), LENCA_ISR, RISING);

  // Motor pins
  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);
  analogWrite(E1, 0);
  analogWrite(E2, 0);

#ifdef USE_TIMER_VEL
  // Timer1 init (optional)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  // Example: 1 Hz base when OCR1A=15624, prescaler 1024 @16MHz; adjust as needed
  OCR1A  = 15624;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
#endif

  interrupts();
}

// =================== LOOP / STATE MACHINE ===================
void loop() {
  String command;

  while (1) {
    switch (STATE) {
      case IDLE: {
        stopMotors();
        Serial.println("State Idle");
        Serial.println("Enter GCode command");
        while (Serial.available() == 0) { /* wait */ }
        command = Serial.readStringUntil('\n');
        STATE = PARSING;
        break;
      }

      case PARSING: {
        int st = Parser.ExecuteCommand(command.c_str());
        if (st == 0) { STATE = IDLE; break; }
        if (st == 1) { STATE = HOMING; break; }
        if (st == 2) {
          // if (Parser.ValidateParameters(currentX, currentY)) STATE = MOVING;
          // else STATE = IDLE;
          // break;
          STATE=MOVING;
          break;
        }
        STATE = ERROR; // unknown parse result
        break;
      }

      case HOMING: {
        Serial.println("Running homing routine");
        Homing();
        STATE = IDLE;
        break;
      }

      case MOVING: {
        Serial.println("Running moving routine");

        // GCode parser provides ΔX(mm), ΔY(mm)
        float dX_mm = Parser.GetParameters()[0];
        float dY_mm = Parser.GetParameters()[1];

        // Convert to each motor’s delta (in mm)
        float dR_mm = inputs_to_encoder_count_delta_A(dX_mm, dY_mm);
        float dL_mm = inputs_to_encoder_count_delta_B(dX_mm, dY_mm);

        // mm -> counts
        long delta_A_ticks = (long)roundf(dR_mm * COUNTTODISTANCERATIO);
        long delta_B_ticks = (long)roundf(dL_mm * COUNTTODISTANCERATIO);
        // G01 X0 Y-10 F10
        // PI-only move with trapezoid speed limiting
        PI_control(delta_A_ticks, delta_B_ticks);

        // Update current XY (after successful move)
        currentX += dX_mm;
        currentY += dY_mm;

        STATE = IDLE;
        break;
      }

      case ERROR: {
        Serial.println("ERROR state");
        STATE = IDLE;
        break;
      }

      default:
        STATE = IDLE;
        break;
    }
  }
}

// =================== KINEMATICS (mm) ===================
float inputs_to_encoder_count_delta_A(float delta_X, float delta_Y) {
  // left motor path (mm)
  return (delta_X + delta_Y);
}
float inputs_to_encoder_count_delta_B(float delta_X, float delta_Y) {
  // right motor path (mm)
  return (delta_X - delta_Y);
}

// =================== ENCODER ISRs ===================
// Right encoder A rising: read B to infer direction
void RENCA_ISR() {
  RDIRECTION = digitalRead(RENCB) ? CCW : CW;
  rel_B_count = RDIRECTION ? rel_B_count + 1 : rel_B_count - 1;
}
// Left encoder A rising: read B to infer direction
void LENCA_ISR() {
  LDIRECTION = digitalRead(LENCB) ? CCW : CW;
  rel_A_count = LDIRECTION ? rel_A_count - 1 : rel_A_count + 1;
}

// =================== LIMIT SWITCH ISRs ===================
void left_limit_switch_hit() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
    if (!left_hit) { analogWrite(E1, 0); analogWrite(E2, 0); }
    left_hit = true;
  }
  left_last_time = left_now;
}
void right_limit_switch_hit() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
    if (!right_hit) { analogWrite(E1, 0); analogWrite(E2, 0); }
    right_hit = true;
  }
  right_last_time = right_now;
}
void top_limit_switch_hit() {
  top_now = millis();
  if (top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top limit switch hit");
    if (!top_hit) { analogWrite(E1, 0); analogWrite(E2, 0); }
    top_hit = true;
  }
  top_last_time = top_now;
}
void bottom_limit_switch_hit() {
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Bottom limit switch hit");
    if (!bottom_hit) { analogWrite(E1, 0); analogWrite(E2, 0); }
    bottom_hit = true;
  }
  bottom_last_time = bottom_now;
}

// =================== HOMING (simple) ===================
void Homing() {
  // Example routine; adjust directions for your mechanics
  digitalWrite(M1, CCW); digitalWrite(M2, CW);
  analogWrite(E1, 200);  analogWrite(E2, 200);
  while (!bottom_hit) { /* wait */ }

  digitalWrite(M1, CW);  digitalWrite(M2, CCW);
  analogWrite(E1, 100);  analogWrite(E2, 100);
  delay(2000);

  bottom_hit = false;
  digitalWrite(M1, CCW); digitalWrite(M2, CW);
  analogWrite(E1, 100);  analogWrite(E2, 100);
  while (!bottom_hit) { /* wait */ }

  // left homing
  bottom_hit = false;
  digitalWrite(M1, CW);  digitalWrite(M2, CW);
  analogWrite(E1, 200);  analogWrite(E2, 200);
  while (!left_hit) { /* wait */ }

  digitalWrite(M1, CCW); digitalWrite(M2, CCW);
  analogWrite(E1, 100);  analogWrite(E2, 100);
  delay(2000);

  left_hit = false;
  digitalWrite(M1, CW);  digitalWrite(M2, CW);
  analogWrite(E1, 100);  analogWrite(E2, 100);
  while (!left_hit) { /* wait */ }
  left_hit = false;

  currentX = 0;
  currentY = 0;
}

double countToDistance(int count)
{
  return (count/COUNTTODISTANCERATIO);
}

int distanceToCount(float distance)
{
  return ((int)distance*COUNTTODISTANCERATIO);
}

// =================== PI-ONLY MOVE (counts + counts/s) ===================
void PI_control(long delta_A_ref_in, long delta_B_ref_in) {
  uint8_t PWM_A = 0;
  uint8_t PWM_B = 0;

  bool A_dir = CCW;
  bool B_dir = CCW;
  bool aDone = false;
  bool bDone = false;

  // Zero relative counts
  rel_A_count = 0;
  rel_B_count = 0;

  const float targetLdist = countToDistance(delta_A_ref_in);  // mm
  const float targetRdist = countToDistance(delta_B_ref_in);  // mm

  Serial.print(targetLdist);
  Serial.print(" ");
  Serial.println(targetRdist);

  long lastR = 0, lastL = 0;
  float vA_cps = 0.0f, vB_cps = 0.0f;            // measured counts/s
  float vCmdR_cps = 0.0f, vCmdL_cps = 0.0f;      // command counts/s
  float iA = 0.0f, iB = 0.0f;                    // integrators

  // Need a calculation for iA and iB if want to use them

  // Set the directions of left and right motors
  while(true) {
    A_dir = (targetLdist - rel_A) > 0 ? CCW : CW; // Check displacement needed to move
    B_dir = (targetRdist - rel_B) > 0 ? CCW : CW;
    PWM_A = K_p_left*abs(targetLdist - rel_A) + iA;
    PWM_B = K_p_right*abs(targetRdist - rel_B) + iB;

    rel_A = countToDistance(rel_A_count);
    rel_B = countToDistance(rel_B_count);

    // Break out when both delta A and delta B are within some distance of target
    aDone = abs(targetLdist - rel_A) < POS_TOL_MM ? true : false;
    bDone = abs(targetRdist - rel_B) < POS_TOL_MM ? true : false;

    driveMotorLeft(A_dir, PWM_A);
    driveMotorRight(B_dir, PWM_B);

    Serial.print("A rel (mm): ");
    Serial.print(rel_A);
    Serial.print(" ");
    Serial.print("B rel (mm): ");
    Serial.println(rel_B);

    // Serial.print("A rel count: ");
    // Serial.print(rel_A_count);
    // Serial.print(" ");
    // Serial.print("B rel count: ");
    // Serial.println(rel_B_count);

    // Serial.print("A diff (mm): ");
    // Serial.print(abs(targetLdist - rel_A));
    // Serial.print(" ");
    // Serial.print("B diff (mm): ");
    // Serial.println(abs(targetRdist - rel_B));

    // Serial.print("A PWM: ");
    // Serial.print(PWM_A);
    // Serial.print(" ");
    // Serial.print("B PWM: ");
    // Serial.println(PWM_B);

    // Serial.print("A done: ");
    // Serial.print(aDone);
    // Serial.print(" ");
    // Serial.print("B done: ");
    // Serial.println(bDone);

    if (aDone && bDone) {
      break;
    }
    delay(100);
  }

  // Stop cleanly
  stopMotors();
}
