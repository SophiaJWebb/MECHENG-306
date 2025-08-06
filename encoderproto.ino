
//Arduino PWM Speed Control：
//#define DEBUG // Comment out to remove serial
double VELOCITYDELAY = 1; //1/24 of a second try 1/8 if too fast
int CTCTIMER = VELOCITYDELAY*15624; //Every 1 second 15624 
double GEARRATIO = 171.79;
double COUNTTODISTANCERATIO = 65.618946; // For 1mm 65.61 counts are needed

double time = 0;

double m1distancetravelled = 0;
double m2distancetravelled = 0;
double m1velocity;
double m2velocity;

int E1 = 5;
int M1 = 4;
int E2 = 6;
int M2 = 7;

int M1HALL1 = 2;
int M2HALL1 = 3;
int M1HALL2 = 10; //TBC
int M2HALL2 = 11; //TBC

int long m1count = 0;
int long m2count = 0;

int m1positive=true;
bool m1direction=HIGH;

int m2positive=true;
bool m2direction=HIGH;

const int CCW  = HIGH;
const int CW = LOW;

double m1position=0; //in mm
double m1lastposition=0;
double m2position=0; //in mm
double m2lastposition=0;

double relxposition=0;
double relyposition=0;

int repeat =0;

void setup()
{
  Serial.begin(9600);
  noInterrupts();           // disable all interrupts

  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(M1HALL2, INPUT);
  pinMode(M2HALL2, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1HALL1), M1HALLONE, RISING);
  attachInterrupt(digitalPinToInterrupt(M2HALL1), M2HALLONE, RISING);

  
  // initialize timer1 
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = CTCTIMER;            // 15624 compare match register 16MHz/1024/1Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10); // 1024 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

}
void loop()
{
  repeat++;
  if (repeat<5)
  {
    m1direction=CW; // M1 CW M2 CCW means TOP M1&M2 CW means Right
    m2direction=CCW; // M2 is left motor M1 is right
  } else
  {
    m1direction=CW;
    m2direction=CCW;
  }
  if (repeat>=9)
  {
    repeat=0;
  }
  int value= 80;
  digitalWrite(M1,m1direction);
  digitalWrite(M2,m2direction);
  analogWrite(E1, value); //PWM Speed Control
  analogWrite(E2, value); //PWM Speed Control

  Serial.print("XVel: ");
  Serial.print(XVelocity(m1distancetravelled, m2distancetravelled));
  Serial.print(" ");
  Serial.print("YVel: ");
  Serial.println(YVelocity(m1distancetravelled, m2distancetravelled));

  #ifdef DEBUG
  Serial.print("M1position: ");
  Serial.print(m1position);
  Serial.print(" ");
  Serial.print("M1Lastposition: ");
  Serial.print(m1lastposition);
  Serial.print(" ");
  Serial.print("M1Speed: ");
  Serial.print(m1velocity);
  Serial.print(" ");
  Serial.print("M1Direction: ");
  Serial.println(m1positive);

  Serial.print("M2position: ");
  Serial.print(m2position);
  Serial.print(" ");
  Serial.print("M2Lastposition: ");
  Serial.print(m2lastposition);
  Serial.print(" ");
  Serial.print("M2Speed: ");
  Serial.print(m2velocity);
  Serial.print(" ");
  Serial.print("M2Direction: ");
  Serial.println(m2positive);
  #endif // Debug
  delay(1000);  
}

ISR(TIMER1_COMPA_vect)
{
  time += VELOCITYDELAY;
  m1distancetravelled=m1position-m1lastposition;
  m1lastposition=m1position;
  m2distancetravelled=m2position-m2lastposition;
  m2lastposition=m2position;
  m1velocity=m1distancetravelled/VELOCITYDELAY;
  m2velocity=m2distancetravelled/VELOCITYDELAY;
}

double XVelocity(double m1distancetravelled, double m2distancetravelled)
{
  relxposition=(m1distancetravelled+m2distancetravelled)/2;
  return relxposition/VELOCITYDELAY;
}

double YVelocity(double m1distancetravelled, double m2distancetravelled)
{
  relyposition=(m1distancetravelled-m2distancetravelled)/2;
  return relyposition/VELOCITYDELAY;
}

double countToDistance(int count)
{
return (count/COUNTTODISTANCERATIO);
}

double distanceToCount(int distance)
{
return (distance*COUNTTODISTANCERATIO);
}

void m1counting()
{
  if (m1positive) {
    m1count++;
  } else{
    m1count--;
  }
  m1position=countToDistance(m1count);
}

void m2counting()
{
  if (m2positive) {
    m2count++;
  } else{
    m2count--;
  }
  m2position=countToDistance(m2count);
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
  m2counting();
}
