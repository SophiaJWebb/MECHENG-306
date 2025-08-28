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

// =================== GAINS (PI on velocity) ===================
float K_p = 0.5f;
float K_i = 1.0f;

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
  int dir = (u >= 0.0f) ? HIGH : LOW;           // sign -> DIR
  int pwm = (int)clampf(fabs(u), 70.0f, 255.0f); // mag -> PWM
  // Serial.print("U: ");
  // Serial.println(u,10);
  // Serial.print("PWM: ");
  // Serial.println(pwm);
  digitalWrite(pinDir, dir);
  analogWrite(pinPwm, pwm);
  if (u==0){analogWrite(pinPwm, 0);}
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
  // Zero relative counts
  motorR.resetEncoder();
  motorL.resetEncoder();

  const long targetRcount = delta_A_ref_in;  // counts
  const long targetLcount = delta_B_ref_in;  // counts

  long lastR = 0, lastL = 0;
  float vA_cps = 0.0f, vB_cps = 0.0f;            // measured counts/s
  float vCmdR_cps = 0.0f, vCmdL_cps = 0.0f;      // command counts/s
  float iA = 0.0f, iB = 0.0f;                    // integrators

  // Anti-windup so K_i * i <= 255
  const float I_MAX = 255.0f / max(1.0f, K_i);

  unsigned long tNext = millis();

  for (;;) {
    // fixed-rate loop @ 100 Hz
    unsigned long now = millis();
    if ((long)(now - tNext) < 0) continue;
    tNext += (unsigned long)(1000.0f * DT);

    // Safety: limit switches
    if (left_hit || right_hit || top_hit || bottom_hit) {
      driveMotor(M1, E1, 0);
      driveMotor(M2, E2, 0);
      Serial.println("ABORT: limit switch hit");
      return;
    }

    // Read encoders
    long cR= motorR.getEncoderTicks();
    long cL= motorL.getEncoderTicks();

    long dR = cR- lastR;
    long dL = cL- lastL;
    lastR = cR; lastL = cL;

    // counts/s
    vA_cps = (float)dR / DT;
    vB_cps = (float)dL / DT;

    // Remaining counts
    long remR_counts = (targetRcount - cR);
    long remL_counts = (targetLcount - cL);
    

    // // Trapezoid setpoints (counts/s)
    // vCmdR_cps = stepTrapezoidCounts(vCmdR_cps, (float)remR_counts);
    // vCmdL_cps = stepTrapezoidCounts(vCmdL_cps, (float)remL_counts);

    // // PI on velocity (counts/s)
    // float eA = vCmdR_cps - vA_cps;
    // float eB = vCmdL_cps - vB_cps;

    iA+= remR_counts*DT;
    iB+= remR_counts*DT;
    // iA += eA * DT;  iA = clampf(iA, -I_MAX, I_MAX);
    // iB += eB * DT;  iB = clampf(iB, -I_MAX, I_MAX);

    float uA = clampf(K_p*remR_counts + K_i*iA, -255.0f, 255.0f);
    float uB = clampf(K_p*remL_counts + K_i*iB, -255.0f, 255.0f);

    // Drive motors
    driveMotor(M1, E1, uA);
    driveMotor(M2, E2, uB);

    // Stop when close *and* slow
    bool aDone = (labs(remR_counts) <= POS_TOL_COUNTS);// && (fabs(vA_cps) <= VEL_TOL_CPS);
    bool bDone = (labs(remL_counts) <= POS_TOL_COUNTS);//&& (fabs(vB_cps) <= VEL_TOL_CPS);
    if (aDone && bDone) 
    {
      Serial.println("done");
      driveMotor(M1, E1, 0);
      driveMotor(M2, E2, 0);
      break;
    }

    // Optional status @ ~5 Hz (mm & mm/s)
    static int div=0;
    if (++div >= (int)(LOOP_HZ/100)) { div=0;
      float posA_mm = (float)cR/ COUNTTODISTANCERATIO;
      float posB_mm = (float)cL/ COUNTTODISTANCERATIO;
      float vA_mmps = vA_cps / COUNTTODISTANCERATIO;
      float vB_mmps = vB_cps / COUNTTODISTANCERATIO;
      Serial.print("R "); Serial.print(posA_mm, 2); Serial.print(" mm  v ");
      Serial.print(vA_mmps, 1); Serial.print(" mm/s ");
      Serial.print(remR_counts); Serial.print(" counts remaining ");
      Serial.print(uA, 2); Serial.print(" PWM");
      Serial.print("  |  L ");
      Serial.print(posB_mm, 2); Serial.print(" mm  v ");
      Serial.print(vB_mmps, 1); Serial.print(" mm/s ");
      Serial.print(remL_counts); Serial.print(" counts remaining ");
      Serial.print(uB, 2); Serial.println(" PWM");
    } //G01 X10 Y-40 F1
  }

  // Stop cleanly
  driveMotor(M1, E1, 0);
  driveMotor(M2, E2, 0);
}
