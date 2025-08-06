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
                change_in_x = parameters[0] - absolute_x;
                change_in_y = parameters[1] - absolute_y;
                cout << "Moving: " << endl;
                // run move function 
                currentState = IDLE; // Reset to IDLE after moving
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