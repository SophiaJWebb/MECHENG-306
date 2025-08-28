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
#define E2 5
#define M2 4   // Right DIR
#define E1 6
#define M1 7   // Left  DIR

// Encoder pins
#define RENCA 2   // Right encoder A (interrupt capable)
#define RENCB 10   // Right encoder B
#define LENCB 11  // Left encoder B
#define LENCA 3  // Left encoder A (NOTE: needs an interrupt-capable pin on your board)

// ======== FEEDFORWARD / FRICTION ========
float KVFF_R = 0.9f;  // PWM per (counts/s) for RIGHT  (tune)
float KVFF_L = 0.9f;  // PWM per (counts/s) for LEFT   (tune)
float KAFF_R = 0.0f;  // PWM per (counts/s^2). Start 0; optional later.
float KAFF_L = 0.0f;

float KF_R   = 15.0f; // static friction breakaway PWM RIGHT (tune 8–25)
float KF_L   = 15.0f; // static friction breakaway PWM LEFT
float MIN_PWM = 0.0f; // keep 0; we’ll add KF only when moving


// =================== GAINS (PI on velocity) ===================
float K_p = 0.2f;
float K_i = 0.5f;

// =================== MECHANICS / UNITS ===================
// counts per mm (you provided): ~65.62 counts/mm
const float COUNTTODISTANCERATIO = 65.618946f;  // counts per mm

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
const long  POS_TOL_COUNTS = 100;     // counts
const float VEL_TOL_CPS    = 100.0f; // counts/s  (~0.15 mm/s)

// =================== HELPERS ===================
static inline float clampf(float x, float lo, float hi){
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float sgnf(float x){ return (x > 0) - (x < 0); }

static inline void driveMotor(int pinDir, int pinPwm, float u){
  if (fabs(u) < 1.0f) {        // small = stop
    analogWrite(pinPwm, 0);
    return;
  }
  int dir = (u >= 0.0f) ? HIGH : LOW;
  int pwm = (int)clampf(fabs(u), 0.0f, 255.0f);
  digitalWrite(pinDir, dir);
  analogWrite(pinPwm, pwm);
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
  pinMode(RENCA, INPUT_PULLUP);
  pinMode(RENCB, INPUT_PULLUP);
  pinMode(LENCA, INPUT_PULLUP);
  pinMode(LENCB, INPUT_PULLUP);

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
        driveMotor(M1, E1, 0);
        driveMotor(M2, E2, 0);
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
        Serial.print("deltaA tick: ");
        Serial.println(delta_A_ticks);
        Serial.print("deltaB tick: ");
        Serial.println(delta_B_ticks);
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
  motorR.countTicks(RDIRECTION ? 1 : -1);
}
// Left encoder A rising: read B to infer direction
void LENCA_ISR() {
  LDIRECTION = digitalRead(LENCB) ? CCW : CW;
  motorL.countTicks(LDIRECTION ? -1 : 1);
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

// =================== PI-ONLY MOVE (counts + counts/s) ===================
void PI_control(long delta_A_ref_in, long delta_B_ref_in) {
  // Reset encoders to make "target = delta"
  motorR.resetEncoder();
  motorL.resetEncoder();

  const long targetRcount = delta_A_ref_in;  // counts
  const long targetLcount = delta_B_ref_in;  // counts

  long lastR = 0, lastL = 0;
  float vA_cps = 0.0f, vB_cps = 0.0f;        // measured velocities (counts/s)
  float vCmdR_cps = 0.0f, vCmdL_cps = 0.0f;  // commanded velocities (counts/s)
  float iR = 0.0f, iL = 0.0f;                // velocity-loop integrators

  // Back-calculation style anti-windup (simple: integrate only when not saturated)
  const float UMAX = 255.0f;
  const float I_MAX = 255.0f / max(1.0f, K_i);

  unsigned long tNext = millis();

  for (;;) {
    // Fixed 100 Hz
    unsigned long now = millis();
    if ((long)(now - tNext) < 0) continue;
    tNext += (unsigned long)(1000.0f * DT);

    // Safety
    if (left_hit || right_hit || top_hit || bottom_hit) {
      driveMotor(M1, E1, 0);
      driveMotor(M2, E2, 0);
      Serial.println("ABORT: limit switch hit");
      return;
    }

    // Encoder positions
    long cR = motorR.getEncoderTicks();
    long cL = motorL.getEncoderTicks();

    // Measured velocities (counts/s)
    long dR = cR - lastR; lastR = cR;
    long dL = cL - lastL; lastL = cL;
    vA_cps = (float)dR / DT;
    vB_cps = (float)dL / DT;

    // Remaining distance (counts)
    long remR_counts = targetRcount - cR;
    long remL_counts = targetLcount - cL;

    // Trajectory: trapezoid in counts/s (keeps both axes time-aligned)
    vCmdR_cps = stepTrapezoidCounts(vCmdR_cps, (float)remR_counts);
    vCmdL_cps = stepTrapezoidCounts(vCmdL_cps, (float)remL_counts);

    // ===== Velocity PI + Feedforward =====
    float eR = vCmdR_cps - vA_cps;  // velocity error
    float eL = vCmdL_cps - vB_cps;

    // Feedforward (velocity + optional accel + static friction to break stiction)
    // Note: we approximate accel from velocity slope of the profile
    static float vCmdR_prev = 0.0f, vCmdL_prev = 0.0f;
    float aCmdR_cps2 = (vCmdR_cps - vCmdR_prev) / DT;
    float aCmdL_cps2 = (vCmdL_cps - vCmdL_prev) / DT;
    vCmdR_prev = vCmdR_cps; vCmdL_prev = vCmdL_cps;

    float ffR = KVFF_R * vCmdR_cps + KAFF_R * aCmdR_cps2;
    float ffL = KVFF_L * vCmdL_cps + KAFF_L * aCmdL_cps2;

    // Static friction kick only when we intend to move
    if (fabs(vCmdR_cps) > 1.0f) ffR += KF_R * sgnf(vCmdR_cps);
    if (fabs(vCmdL_cps) > 1.0f) ffL += KF_L * sgnf(vCmdL_cps);

    // PI
    float uR_pi = K_p * eR + K_i * iR;
    float uL_pi = K_p * eL + K_i * iL;

    float uR = uR_pi + ffR;
    float uL = uL_pi + ffL;

    // Clamp & anti-windup (integrate only if not saturating in the same direction)
    float uR_clamped = clampf(uR, -UMAX, UMAX);
    float uL_clamped = clampf(uL, -UMAX, UMAX);
    bool satR = (uR != uR_clamped);
    bool satL = (uL != uL_clamped);

    if (!satR || (satR && sgnf(eR) != sgnf(uR))) {  // conditional integrate
      iR = clampf(iR + eR * DT, -I_MAX, I_MAX);
    }
    if (!satL || (satL && sgnf(eL) != sgnf(uL))) {
      iL = clampf(iL + eL * DT, -I_MAX, I_MAX);
    }

    // Drive motors
    driveMotor(M1, E1, uR_clamped);
    driveMotor(M2, E2, uL_clamped);

    // Stop when close AND commanded speed ~0
    bool rDone = (labs(remR_counts) <= POS_TOL_COUNTS) && (fabs(vCmdR_cps) < 1.0f);
    bool lDone = (labs(remL_counts) <= POS_TOL_COUNTS) && (fabs(vCmdL_cps) < 1.0f);
    if (rDone && lDone) {
      Serial.println("done");
      break;
    }

    // Debug print (~10 Hz)
    static int div=0;
    if (++div >= 10) { div=0;
      float posR_mm = (float)cR / COUNTTODISTANCERATIO;
      float posL_mm = (float)cL / COUNTTODISTANCERATIO;
      float vR_mmps = vA_cps / COUNTTODISTANCERATIO;
      float vL_mmps = vB_cps / COUNTTODISTANCERATIO;
      Serial.print("R ");
      Serial.print(posR_mm, 2); Serial.print(" mm  v ");
      Serial.print(vR_mmps, 1); Serial.print(" mm/s  rem ");
      Serial.print(remR_counts); Serial.print("  u ");
      Serial.print(uR_clamped, 1);
      Serial.print(" | L ");
      Serial.print(posL_mm, 2); Serial.print(" mm  v ");
      Serial.print(vL_mmps, 1); Serial.print(" mm/s  rem ");
      Serial.print(remL_counts); Serial.print("  u ");
      Serial.println(uL_clamped, 1);
    }
  }

  // Stop cleanly G01 X10 Y10 F10
  driveMotor(M1, E1, 0);
  driveMotor(M2, E2, 0);
}

