//Arduino PWM Speed Control：
#define DEBUG // Comment out to remove serial
float GEARRATIO = 171.79;
int E1 = 5;
int M1 = 4;

int HALL1 = 20;
int HALL2 = 21;
int long count = 0;
int hallOneLit = 0;
int hallTwoLit = 0;
int positive=true;
bool direction=HIGH;
const int CCW  = HIGH;
const int CW = LOW;
float position;


void setup()
{
  Serial.begin(9600);
  pinMode(M1, OUTPUT);
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  attachInterrupt(digitalPinToInterrupt(HALL1), HALLONE, RISING);
}
void loop()
{
  for (int value = 0; value <= 255; value++) {
  if (value<128){
    direction=CCW;
  } else{
    direction=CW;
  }
  digitalWrite(M1,direction);
  analogWrite(E1, 255); //PWM Speed Control
  position=count/GEARRATIO;

  #ifdef DEBUG
  Serial.print("Count: ");
  Serial.print(position);
  Serial.print(" ");
  Serial.print("Direction: ");
  Serial.println(positive);
  delay(10);  
  #endif // Debug
  }
}



void counting(){
  if (positive) {
    count++;
  } else{
    count--;
  }
}

void HALLONE(){
    if (digitalRead(HALL2)) {
      positive=true;

  } else {
      positive=false;
  }
  counting();
}
