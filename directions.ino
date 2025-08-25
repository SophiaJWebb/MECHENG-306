#define LIMIT_SWITCH
#define DEBOUNCE_DELAY_MS 500

#define LEFT_INTERRUPT_PIN 18
#define RIGHT_INTERRUPT_PIN 19
#define TOP_INTERRUPT_PIN 21
#define BOTTOM_INTERRUPT_PIN 20

//motor set up 
#define E1 5
#define M1 4
#define E2 6
#define M2 7

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

void setup() {
  pinMode(LEFT_INTERRUPT_PIN, INPUT);
  pinMode(RIGHT_INTERRUPT_PIN, INPUT);
  pinMode(TOP_INTERRUPT_PIN, INPUT);
  pinMode(BOTTOM_INTERRUPT_PIN, INPUT);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);

  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(LEFT_INTERRUPT_PIN), left_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_INTERRUPT_PIN), right_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(TOP_INTERRUPT_PIN), top_limit_switch_hit, RISING);
  attachInterrupt(digitalPinToInterrupt(BOTTOM_INTERRUPT_PIN), bottom_limit_switch_hit, RISING);
}

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

enum direction {
  CW,
  CCW
};

void loop() {
  //put your main code here, to run repeatedly:
  //Left
  digitalWrite(M1, CW);
  analogWrite(E1, 100);
  digitalWrite(M2, CW);
  analogWrite(E2, 100);
  delay(2000);
  // Right
  digitalWrite(M1, CCW);
  analogWrite(E1, 100);
  digitalWrite(M2, CCW);
  analogWrite(E2, 100);
  delay(2000);
  // // // up
  digitalWrite(M1, CCW);
  analogWrite(E1, 100);
  digitalWrite(M2, CW);
  analogWrite(E2, 100);
  delay(2000);
  // // // Down
  digitalWrite(M1, CW);
  analogWrite(E1, 100);
  digitalWrite(M2, CCW);
  analogWrite(E2, 100);
  delay(2000);

}