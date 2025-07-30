#include "GCode.hpp"
#include <iostream>
using namespace std;

void GCode::CaseCapitalize() {
    for (char& c : command) {
        c = toupper(c);
    }
}

void GCode::execute() {
    CaseCapitalize();

    if (command.substr(0, 3) == "G28") {
        // Run homing routine
        cout << "Executing homing routine for G28 command." << endl;

    } else if (command.substr(0, 2) == "G1") {
        // Identify X and Y commands
        size_t xPos = command.find('X');
        size_t yPos = command.find('Y');
        
        if (xPos != std::string::npos && yPos != std::string::npos) {
            size_t xEnd = command.find(' ', xPos);
            std::string xCoord = command.substr(xPos + 1,
                (xEnd == std::string::npos ? command.size() : xEnd) - xPos - 1);
            size_t yEnd = command.find(' ', yPos);
            std::string yCoord = command.substr(yPos + 1,
                (yEnd == std::string::npos ? command.size() : yEnd) - yPos - 1);

            //convert coordinates to float
            float x = std::stof(xCoord);
            float y = std::stof(yCoord);

            // Execute G1 command with coordinates
            cout << "Executing G1 command with coordinates: X=" 
                 << xCoord << ", Y=" << yCoord << endl;

        } else {
            cout << "Invalid G1 command. Missing X or Y coordinates." << endl;
        }
    } else {
        cout << "Unknown GCode command: " << command << endl;
    }
}

void GCode::print() const {
    std::cout << "GCode: " << command << std::endl;
}