#include "GCode.hpp"
#include <vector>
#include <iostream>
using namespace std;

GCodeParser::GCodeParser() {}

int GCodeParser::ExecuteCommand(const std::string& cmd) {
    command = cmd;
    std::vector<std::string> tokens = tokenize(command);
    CaseCapitalize();
    return parameterExtraction(tokens);;
}

std::vector<std::string> GCodeParser::tokenize(std::string& cmd) {
    //implement
    std::vector<std::string> tokens;
    std::string buffer;
    for (int i = 0; i < cmd.length(); i++){
        if (cmd[i] == ' ' /*|| cmd[i] == '\t') { */ ){
            if (!buffer.empty()) {
                tokens.push_back(buffer);
                buffer.clear();
            }
        } else {
            buffer += cmd[i];
        }
    }
    if (!buffer.empty()) {
        tokens.push_back(buffer);
    }
    return tokens;
}

int GCodeParser::parameterExtraction(std::vector<std::string>& tokens) {
    //implement
    if (tokens[0] == "G28") {
        std::cout << "Changing to homing state for G28 command." << std::endl;
        return 1; // Change to homing state
    }
    else if (tokens[0] == "M999") {
        //enter homing state ??
        std::cout << "Entering homing state for M999 command." << std::endl;
        return 1; // Change to homing state
    }
    else if (tokens[0] == "G01"){
        std::cout << "Executing G01 command." << std::endl;
        parameters[0] = 0.0f; // Reset X
        parameters[1] = 0.0f; // Reset Y
        for (int i = 1; i< tokens.size(); i++){
            string token = tokens[i];
            // Check for X, Y, Z parameters
            if (token[0] == 'X'){
                string xValue = token.substr(1);
                parameters[0] = std::stof(xValue);
            } else if (token[0] == 'Y'){
                string yValue = token.substr(1);
                parameters[1] = std::stof(yValue);
            } else if (token[0] == 'F'){
                string fValue = token.substr(1);
                parameters[2] = std::stof(fValue);
            } else if (token[0] == ';') {
                // Ignore comments
                continue;
            }
            else{
                std::cout << "Unknown parameter: " << token << std::endl;
                return 0;
            }
        }
        cout << "Parameters extracted: " << endl;
        cout << "X: " << parameters[0] << ", Y: " << parameters[1]
        << ", F: " << parameters[2] << endl;
        // position validity check 
        // switch to moving state and enter parameters
        return 2; // Change to moving state
    }
    else {
        std::cout << "Unknown command: " << tokens[0] << std::endl;
        return 0;
    }
     return 0;
}

void GCodeParser::CaseCapitalize() {
    for (char& c : command) {
        c = toupper(c);
    }
}

bool GCodeParser::ValidateParameters(float currentX, float currentY) {
    // Implement validation logic for parameters
    if ((parameters[0] + currentX) > 150 || (parameters[1] + currentY) > 150 || parameters[1] + currentY < 0 || parameters[0] + currentX < 0) {
        return false; // Parameters exceed board limits
    }
    return true; // Parameters are valid
}