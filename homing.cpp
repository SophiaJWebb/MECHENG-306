#define LEFT_INTERRUPT_PIN 18
#define RIGHT_INTERRUPT_PIN 19
#define TOP_INTERRUPT_PIN 21
#define BOTTOM_INTERRUPT_PIN 20

//motor set up 
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

const int CCW  = HIGH;
const int CW = LOW;

Homing() {
    digitalWrite(M1,CCW);
  digitalWrite(M2,CW);
  analogWrite(E1, 200); //PWM Speed Control
  analogWrite(E2, 200); //PWM Speed Control
  while(!hit){
    Serial.println("hit: ");
    Serial.print(hit);
  }
  digitalWrite(M1,CW);
  digitalWrite(M2,CCW);
  analogWrite(E1, value); //PWM Speed Control
  analogWrite(E2, value);
  delay(2000);
  hit = false;
  digitalWrite(M1,CCW);
  digitalWrite(M2,CW);
  analogWrite(E1, value); //PWM Speed Control
  analogWrite(E2, value); //PWM Speed Control
  while(!hit){
    Serial.println("moving");
  }
//left homing
  hit = false;
  digitalWrite(M1,CW);
  digitalWrite(M2,CW);
  analogWrite(E1, 200); //PWM Speed Control
  analogWrite(E2, 200); //PWM Speed Control
  while(!hit){
    Serial.println("hit: ");
    Serial.print(hit);
  }
  digitalWrite(M1,CCW);
  digitalWrite(M2,CCW);
  analogWrite(E1, value); //PWM Speed Control
  analogWrite(E2, value);
  delay(2000);
  hit = false;
  digitalWrite(M1,CW);
  digitalWrite(M2,CW);
  analogWrite(E1, value); //PWM Speed Control
  analogWrite(E2, value); //PWM Speed Control
  while(!hit){
    Serial.println("moving");
  }
  hit = false;
}
