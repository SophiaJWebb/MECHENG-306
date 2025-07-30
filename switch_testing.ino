//#define DEBUG // Comment out to remove serial output
// Attached to MEGA interrupt pins
#define LEFT_RIGHT_PIN 2
#define TOP_BOTTOM_PIN 3
#define DEBOUNCE_DELAY_MS 500

int long left_right_last_time = 0;
int long left_right_now = 0;
int long top_bottom_last_time = 0;
int long top_bottom_now = 0;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LEFT_RIGHT_PIN), LEFT_RIGHT_ISR, RISING); // Left pin interrupt
  attachInterrupt(digitalPinToInterrupt(TOP_BOTTOM_PIN), TOP_BOTTOM_ISR, RISING); // Top pin interrupt
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

void LEFT_RIGHT_ISR() {
  left_right_now = millis();
  if (left_right_now - left_right_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Left or right limit switch hit");
  }
  left_right_last_time = millis();
}

void TOP_BOTTOM_ISR() {
  top_bottom_now = millis();
  if (top_bottom_now - top_bottom_last_time > DEBOUNCE_DELAY_MS) {
    Serial.println("Top or bottom limit switch hit");
  }
  top_bottom_last_time = millis();
}
