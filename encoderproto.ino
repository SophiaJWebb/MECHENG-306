//Arduino PWM Speed Control：
#define DEBUG // Comment out to remove serial
double GEARRATIO = 171.79;
double COUNTTODISTANCERATIO = 65.618946 // For 1mm 65.61 counts are needed
int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

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

double m1position=0; //in mm
double m2position=0; //in mm


void setup()
{
  Serial.begin(9600);

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(M1HALL2, INPUT);
  pinMode(M2HALL2, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1HALL1), M1HALLONE, RISING);
  attachInterrupt(digitalPinToInterrupt(M2HALL1), M2HALLONE, RISING);

}
void loop()
{
  value= 255;
  m1direction=CW;
  m2direction=CW;
  digitalWrite(M1,m1direction);
  digitalWrite(M2,m2direction);
  analogWrite(E1, 255); //PWM Speed Control
  analogWrite(E1, 255); //PWM Speed Control
  m1position=countToDistance(m1count);
  m2position=countToDistance(m1count);

  #ifdef DEBUG
  Serial.print("M1Count: ");
  Serial.print(m1position);
  Serial.print(" ");
  Serial.print("M1Direction: ");
  Serial.println(m1positive);

  Serial.print("M2Count: ");
  Serial.print(m2position);
  Serial.print(" ");
  Serial.print("M2Direction: ");
  Serial.println(m2positive);
  delay(10);  
  #endif // Debug
  }
}

double countToDistance(int count)
{
return (count/COUNTTODISTANCERATIO)
}

double distanceToCount(int distance)
{
return (distance*COUNTTODISTANCERATIO)
}

void m1counting()
{
  if (m1positive) {
    m1count++;
  } else{
    m1count--;
  }
}

void m2counting()
{
  if (m2positive) {
    m2count++;
  } else{
    m2count--;
  }
}

void M1HALLONE()
{
    if (digitalRead(M1HALL2)) {
      m1positive=true;

  } else {
      m1positive=false;
  }
  m1counting();
}

void M2HALLONE()
{
    if (digitalRead(M2HALL2)) {
      m2positive=true;

  } else {
      m2positive=false;
  }
  m1counting();
}
