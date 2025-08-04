// State definitions
enum State {
  IDLE,
  HOMING,
  CALIBRATION,
  MOVING,
  ERROR
};

State currentState = IDLE;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read buttons (active LOW)

  switch (currentState) {
    case IDLE:

      break;

    case HOMING:

      break;
    case CALIBRATION:

      break;
    case MOVING:

      break;  
    case ERROR:

      break;                
  }
}
