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
    //Move down
    DigitalWrite(M1, CCW);
    DigitalWrite(M2, CW);
    AnalogueWrite(E1, 100);  // Can increase speed later
    AnalogueWrite(E2, 100);
    // move until bottom switch is pressed
    while (!bottom_hit){
    }
    bottom_hit = false;
    DigitalWrite(M1, CW);
    DigitalWrite(M2, CCW);
    AnalogueWrite(E1, 70);  
    AnalogueWrite(E2, 70);
    delay(1000); // change from delay 
    //back up until encoder count is tbd 
    DigitalWrite(M1, CCW);
    DigitalWrite(M2, CW);
    AnalogueWrite(E1, 70);  
    AnalogueWrite(E2, 70);
    while (!bottom_hit){
    }
    bottom_hit = false;
    // Set absolute_x 0

    //move left
    DigitalWrite(M1, CCW);
    DigitalWrite(M2, CCW);
    AnalogueWrite(E1, 100);  // Can increase speed later
    AnalogueWrite(E2, 100);
    // move until left switch is pressed
    while (!left_hit){
    }
    left_hit = false;
    DigitalWrite(M1, CCW);
    DigitalWrite(M2, CCW);
    AnalogueWrite(E1, 70);  
    AnalogueWrite(E2, 70);
    delay(1000); // change from delay 
    //back up until encoder count is tbd 
    DigitalWrite(M1, CCW);
    DigitalWrite(M2, CCW);
    AnalogueWrite(E1, 70);  
    AnalogueWrite(E2, 70);
    while (!left_hit){
    }
    bottom_hit = false;
}
