#include "GCode.hpp"
//#include encoder 
//#include
#include <iostream>
using namespace std;

#define IDLE 0
#define HOMING 1
#define MOVING 2
#define ERROR 3
#define CALIBRATION 4

void setup() {
  //Serial.begin(9600);

}

int main() {
    // Read buttons (active LOW)
    GCodeParser Parser;
    bool running = true;
    int currentState = IDLE;
  
    float absolute_x = -1;
    float absolute_y = -1;
    float change_in_x = 0;
    float change_in_y = 0;
    string command;
    const float* parameters = nullptr;

    while (running) {
        switch (currentState) {
            case IDLE: {
                cout << "Enter GCode command: ";
                std::getline(std::cin, command);
                int State = Parser.ExecuteCommand(command);
                if (State == 1) {
                    currentState = HOMING;
                } else if (State == 2) {
                    currentState = MOVING;
                }
                break;
            }
            case HOMING: {
                std::cout << "Homing in progress..." << std::endl;
                currentState = IDLE; // Reset to IDLE after homing 
                break;
            }

            case MOVING: {
                parameters = Parser.GetParameters();
                change_in_x = parameters[0];
                change_in_y = parameters[1];
                bool valid = Parser.ValidateParameters(absolute_x, absolute_y);
                if (!valid) {
                    std::cout << "Invalid parameters entered: board limits exceeded." << std::endl;
                    currentState = IDLE; // Switch to error state
                    break;
                }
                else {
                cout << "Moving: " << endl;
                // run move function 
                currentState = IDLE; // Reset to IDLE after moving
                break;
                }
                break;
            }  
            case ERROR: {
                std::cout << "Error occurred." << std::endl;
                break;
            }
            case CALIBRATION: {
                std::cout << "Calibration in progress..." << std::endl;
                break;
            }
        }
    }
}