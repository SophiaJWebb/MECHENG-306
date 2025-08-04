//Arduino PWM Speed Control：
#define DEBUG // Comment out to remove serial
float GEARRATIO = 171.79;
int E1 = 5;
int M1 = 4;

int M1HALL1 = 2;
int M2HALL1 = 3;
// int M1HALL2 = 10; TBC
// int M2HALL2 = 11; TBC
int long m1count = 0;
int long m2count = 0;

int m1positive=true;
bool m1direction=HIGH;

int m2positive=true;
bool m2direction=HIGH;

const int CCW  = HIGH;
const int CW = LOW;

float m1position;
float m2position;


void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(M1HALL1, INPUT);
  pinMode(M2HALL1, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1HALL1), M1HALLONE, RISING);
}
void loop()
{
  value= 255;
  m1direction=CW;
  digitalWrite(M1,m1direction);
  analogWrite(E1, 255); //PWM Speed Control
  m1position=m1count/GEARRATIO;

  #ifdef DEBUG
  Serial.print("Count: ");
  Serial.print(position);
  Serial.print(" ");
  Serial.print("Direction: ");
  Serial.println(m1positive);
  delay(10);  
  #endif // Debug
  }
}



void m1counting(){
  if (m1positive) {
    m1count++;
  } else{
    m1count--;
  }
}

void m2counting(){
  if (m2positive) {
    m2count++;
  } else{
    m2count--;
  }
}

void M1HALLONE(){
    if (digitalRead(M1HALL2)) {
      m1positive=true;

  } else {
      m1positive=false;
  }
  m1counting();
}

void M2HALLONE(){
    if (digitalRead(M2HALL2)) {
      m2positive=true;

  } else {
      m2positive=false;
  }
  m1counting();
}
