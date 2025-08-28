// =================== HEADERS ===================
#include "Encoder.h"
#include "GCode.hpp"

// =================== CONFIG / DEFINES ===================
#define LIMIT_SWITCH
#define DEBOUNCE_DELAY_MS 500

// ----- Board: Arduino MEGA external-interrupt pins: 2,3,18,19,20,21 -----
// FIX: move limit switches to real interrupt pins on MEGA
#define LEFT_INTERRUPT_PIN   18
#define RIGHT_INTERRUPT_PIN  19
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
float KVFF_R = 0.04f;
float KVFF_L = 0.04f;
float KAFF_R = 0.0f;
float KAFF_L = 0.0f;
float KF_R = 10.0f;
float KF_L = 10.0f;
float MIN_PWM = 0.0f;

// =================== GAINS (PI on velocity) ===================
float K_p = 0.05f;
float K_i = 0.02f;

// =================== MECHANICS / UNITS ===================
const float COUNTTODISTANCERATIO = 65.618946f;  // counts per mm

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
                  left_limit_switch_hit, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_PIN),
                  right_limit_switch_hit, FALLING);
  attachInterrupt(digitalPinToInterrupt(TOP_INTERRUPT_PIN),
                  top_limit_switch_hit, FALLING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_INTERRUPT_PIN),
                  bottom_limit_switch_hit, FALLING);

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
        if (st == 2) { STATE = MOVING; break; }
        STATE = ERROR;
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

        // Parser: ΔX, ΔY in mm. If your Y was inverted, keep the minus.
        float dX_mm = Parser.GetParameters()[0];
        float dY_mm = Parser.GetParameters()[1];  // keep if your frame is flipped
        Serial.print("dX mm: "); Serial.println(dX_mm);
        Serial.print("dY mm: "); Serial.println(dY_mm);

        // Convert to per-motor deltas (mm)
        float dR_mm = inputs_to_encoder_count_delta_R(dX_mm, dY_mm);
        float dL_mm = inputs_to_encoder_count_delta_L(dX_mm, dY_mm);

        Serial.print("dR mm: "); Serial.println(dR_mm);
        Serial.print("dL mm: "); Serial.println(dL_mm);

        // mm -> counts
        long delta_R_ticks = (long)roundf(dR_mm * COUNTTODISTANCERATIO);
        motorR.setDirection(delta_R_ticks/abs(delta_R_ticks));

        long delta_L_ticks = (long)roundf(dL_mm * COUNTTODISTANCERATIO);

        motorL.setDirection(delta_L_ticks/abs(delta_L_ticks));

        Serial.print("R Direction: "); Serial.println(delta_R_ticks/abs(delta_R_ticks));
        Serial.print("L Direction: "); Serial.println(delta_L_ticks/abs(delta_L_ticks));

        Serial.print("deltaR tick: "); Serial.println(delta_R_ticks);
        Serial.print("deltaL tick: "); Serial.println(delta_L_ticks);

        PI_control(delta_R_ticks, delta_L_ticks);

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

      default: STATE = IDLE; break;
    }
  }
}

// =================== KINEMATICS (mm) ===================
float inputs_to_encoder_count_delta_L(float delta_X, float delta_Y) {
  // Your chosen mapping: L = X - Y
  return (delta_X + delta_Y);
}
float inputs_to_encoder_count_delta_R(float delta_X, float delta_Y) {
  // Your chosen mapping: R = X + Y
  return (delta_X - delta_Y);
}

// =================== ENCODER ISRs ===================
// FIX: count direction directly as +/-1 to avoid CW=0/CCW=1 pitfalls.
// Right encoder A rising: read B; assume B=HIGH means one direction.
void RENCA_ISR() {
  motorR.countTicks(motorR.getDirection());
}
// Left encoder A rising: read B; mirror right, then optionally flip once.
void LENCA_ISR() {
  motorL.countTicks(motorL.getDirection());
}

// =================== LIMIT SWITCH ISRs ===================
void left_limit_switch_hit() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
    analogWrite(E1, 0); analogWrite(E2, 0);
    left_hit = true;
  }
  left_last_time = left_now;
}
void right_limit_switch_hit() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
    analogWrite(E1, 0); analogWrite(E2, 0);
    right_hit = true;
  }
  right_last_time = right_now;
}
void top_limit_switch_hit() {
  top_now = millis();
  if (top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top limit switch hit");
    analogWrite(E1, 0); analogWrite(E2, 0);
    top_hit = true;
  }
  top_last_time = top_now;
}
void bottom_limit_switch_hit() {
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Bottom limit switch hit");
    analogWrite(E1, 0); analogWrite(E2, 0);
    bottom_hit = true;
  }
  bottom_last_time = bottom_now;
}

// =================== HOMING (simple) ===================
void Homing() {
  // NOTE: adjust directions if needed for your mechanics.
  digitalWrite(M1, LOW);  // arbitrary "CCW"
  digitalWrite(M2, HIGH); // arbitrary "CW"
  analogWrite(E1, 200);
  analogWrite(E2, 200);
  while (!bottom_hit) { /* wait */ }
  analogWrite(E1, 0); analogWrite(E2, 0);

  // Nudge off
  digitalWrite(M1, HIGH);
  digitalWrite(M2, LOW);
  analogWrite(E1, 100);
  analogWrite(E2, 100);
  delay(2000);
  analogWrite(E1, 0); analogWrite(E2, 0);

  bottom_hit = false;
  digitalWrite(M1, LOW);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 100);
  analogWrite(E2, 100);
  while (!bottom_hit) { /* wait */ }
  analogWrite(E1, 0); analogWrite(E2, 0);

  // Left homing
  bottom_hit = false;
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 200);
  analogWrite(E2, 200);
  while (!left_hit) { /* wait */ }
  analogWrite(E1, 0); analogWrite(E2, 0);

  // Nudge off
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, 100);
  analogWrite(E2, 100);
  delay(2000);
  analogWrite(E1, 0); analogWrite(E2, 0);

  left_hit = false;
  digitalWrite(M1, HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 100);
  analogWrite(E2, 100);
  while (!left_hit) { /* wait */ }
  analogWrite(E1, 0); analogWrite(E2, 0);
  left_hit = false;

  currentX = 0;
  currentY = 0;
}

// =================== PI-ONLY MOVE (counts + counts/s) ===================
void PI_control(long delta_R_ref_in, long delta_L_ref_in) {
  motorR.resetEncoder();
  motorL.resetEncoder();

  const long targetRcount = delta_R_ref_in;
  const long targetLcount = delta_L_ref_in;

  const float total_distance =
      sqrtf((float)targetRcount * (float)targetRcount +
            (float)targetLcount * (float)targetLcount);

  if (total_distance <= 0.5f) {
    driveMotor(M2, E2, 0);
    driveMotor(M1, E1, 0);
    Serial.println("done");
    return;
  }

  const float unitR = (float)targetRcount / total_distance;
  const float unitL = (float)targetLcount / total_distance;

  long  lastR = 0, lastL = 0;
  float vR_cps = 0.0f, vL_cps = 0.0f;
  float vCmd_cps = 0.0f;
  float vCmdR_cps = 0.0f, vCmdL_cps = 0.0f;
  float iR = 0.0f, iL = 0.0f;

  const float UMAX  = 255.0f;
  const float I_MAX = (K_i > 0.0f) ? (255.0f / K_i) : 0.0f;

  static float vCmdR_prev = 0.0f, vCmdL_prev = 0.0f;

  unsigned long tNext = millis();

  for (;;) {
    unsigned long now = millis();
    if ((long)(now - tNext) < 0) continue;
    tNext += (unsigned long)(1000.0f * DT);

    if (left_hit || right_hit || top_hit || bottom_hit) {
      driveMotor(M2, E2, 0);
      driveMotor(M1, E1, 0);
      Serial.println("ABORT: limit switch hit");
      return;
    }

    long cR = motorR.getEncoderTicks();
    long cL = motorL.getEncoderTicks();

    long dR = cR - lastR; lastR = cR;
    long dL = cL - lastL; lastL = cL;
    vR_cps = (float)dR / DT;
    vL_cps = (float)dL / DT;

    float prog_counts      = cR * unitR + cL * unitL;
    float remTotal_counts  = total_distance - prog_counts;

    vCmd_cps = stepTrapezoidCounts(vCmd_cps, remTotal_counts);

    vCmdR_cps = vCmd_cps * unitR;
    vCmdL_cps = vCmd_cps * unitL;

    float eR = vCmdR_cps - vR_cps;
    float eL = vCmdL_cps - vL_cps;

    float aCmdR_cps2 = (vCmdR_cps - vCmdR_prev) / DT;
    float aCmdL_cps2 = (vCmdL_cps - vCmdL_prev) / DT;
    vCmdR_prev = vCmdR_cps;
    vCmdL_prev = vCmdL_cps;

    float ffR = KVFF_R * vCmdR_cps + KAFF_R * aCmdR_cps2;
    float ffL = KVFF_L * vCmdL_cps + KAFF_L * aCmdL_cps2;
    if (fabs(vCmdR_cps) > 1.0f) ffR += KF_R * sgnf(vCmdR_cps);
    if (fabs(vCmdL_cps) > 1.0f) ffL += KF_L * sgnf(vCmdL_cps);

    float uR_pi = K_p * eR + K_i * iR;
    float uL_pi = K_p * eL + K_i * iL;

    float uR = uR_pi + ffR;
    float uL = uL_pi + ffL;

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

  driveMotor(M2, E2, 0);
  driveMotor(M1, E1, 0);
}
