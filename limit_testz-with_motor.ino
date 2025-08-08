//#define DEBUG // Comment out to remove serial output
// Attached to MEGA interrupt pins
#define LEFT_PIN 18
#define TOP_PIN 21
#define RIGHT_PIN 19
#define BOTTOM_PIN 20
#define DEBOUNCE_DELAY_MS 500

int long left_last_time = 0;
int long left_now = 0;
int long top_last_time = 0;
int long top_now = 0;

int long right_last_time = 0;
int long right_now = 0;
int long bottom_last_time = 0;
int long bottom_now = 0;

//motor set up 
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

const int CCW  = HIGH;
const int CW = LOW;

bool m1direction=HIGH;
bool m2direction=HIGH;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN), LEFT_ISR, RISING); // Left pin interrupt
  attachInterrupt(digitalPinToInterrupt(TOP_PIN), TOP_ISR, RISING); // Top pin interrupt
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN), RIGHT_ISR, RISING); // Right pin interrupt
  attachInterrupt(digitalPinToInterrupt(BOTTOM_PIN), BOTTOM_ISR, RISING); // Bottom pin interrupt

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
}

int repeat = 0;
int value= 80;

void loop()
{
  repeat++;
  if (repeat<5)
  {
    m1direction=CW; // M1 CW M2 CCW means TOP M1&M2 CW means Right
    m2direction=CCW; // M2 is left motor M1 is right
  } else
  {
    m1direction=CCW;
    m2direction=CW;
  }
  if (repeat>=9)
  {
    repeat=0;
  }
  
  digitalWrite(M1,m1direction);
  digitalWrite(M2,m2direction);
  analogWrite(E1, value); //PWM Speed Control
  analogWrite(E2, value); //PWM Speed Control
  delay(1000);
}


void LEFT_ISR() {
  analogWrite(E1, 0); //PWM Speed Control
  analogWrite(E2, 0); //PWM Speed Control
  value = 0;
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
  }
  left_last_time = millis();
}

void TOP_ISR() {
  analogWrite(E1, 0); //PWM Speed Control
  analogWrite(E2, 0); //PWM Speed Control
  value = 0;
  top_now = millis();
  if (top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top limit switch hit");
  }
  top_last_time = millis();
}

void RIGHT_ISR() {
  analogWrite(E1, 0); //PWM Speed Control
  analogWrite(E2, 0); //PWM Speed Control
  value = 0;
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
  }
  right_last_time = millis();
}

void BOTTOM_ISR() {
  analogWrite(E1, 0); //PWM Speed Control
  analogWrite(E2, 0); //PWM Speed Control
  value = 0;
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Bottom limit switch hit");
  }
  bottom_last_time = millis();
}
