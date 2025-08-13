#include "GCode.hpp"

#define IDLE 0
#define HOMING 1
#define MOVING 2
#define ERROR 3
#define CALIBRATION 4

GCodeParser Parser;

int currentState = IDLE;
float absolute_x = -1;
float absolute_y = -1;
float change_in_x = 0;
float change_in_y = 0;
String command;  // Arduino String type
const float* parameters = nullptr;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial to connect
  }
  //Serial.println("Ready for GCode commands...");
}

void loop() {
  switch (currentState) {
    case IDLE: {
      Serial.println("State Idle");
      while (Serial.available() > 0){
        //Serial.println("Enter GCode command");
        command = Serial.readStringUntil('\n');  // Read until newline
        int State = Parser.ExecuteCommand(command.c_str());
        if (State == 1) {
          currentState = HOMING;
        } else if (State == 2) {
          currentState = MOVING;
        }
        break;
      }
    }

    case HOMING: {
      Serial.println("State Idle");
      //Serial.println("Homing in progress...");
      currentState = IDLE; // Reset to IDLE after homing
      break;
    }

    case MOVING: {
      parameters = Parser.GetParameters();
      change_in_x = parameters[0];
      change_in_y = parameters[1];
      bool valid = Parser.ValidateParameters(absolute_x, absolute_y);
      if (!valid) {
        Serial.println("Invalid parameters entered: board limits exceeded.");
        currentState = IDLE; // Switch to error state
      } else {
        Serial.println("Moving...");
        // Run move function here
        currentState = IDLE; // Reset to IDLE after moving
      }
      break;
    }

    case ERROR: {
      Serial.println("Error occurred.");
      currentState = IDLE;
      break;
    }

    case CALIBRATION: {
      Serial.println("Calibration in progress...");
      currentState = IDLE;
      break;
    }
  }
}