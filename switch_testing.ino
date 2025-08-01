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

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN), LEFT_ISR, RISING); // Left pin interrupt
  attachInterrupt(digitalPinToInterrupt(TOP_PIN), TOP_ISR, RISING); // Top pin interrupt
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN), RIGHT_ISR, RISING); // Right pin interrupt
  attachInterrupt(digitalPinToInterrupt(BOTTOM_PIN), BOTTOM_ISR, RISING); // Bottom pin interrupt
}

void loop() {
  #ifdef DEBUG
    Serial.print(left);
    Serial.print(" ");
    Serial.print(right);
    Serial.print(" ");
    Serial.print(top);
    Serial.print(" ");
    Serial.println(bottom);
  #endif

  delay(10);
}

void LEFT_ISR() {
  left_now = millis();
  if (left_now - left_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left limit switch hit");
  }
  left_last_time = millis();
}

void TOP_ISR() {
  top_now = millis();
  if (top_now - top_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top limit switch hit");
  }
  top_last_time = millis();
}

void RIGHT_ISR() {
  right_now = millis();
  if (right_now - right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Right limit switch hit");
  }
  right_last_time = millis();
}

void BOTTOM_ISR() {
  bottom_now = millis();
  if (bottom_now - bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Bottom limit switch hit");
  }
  bottom_last_time = millis();
}
