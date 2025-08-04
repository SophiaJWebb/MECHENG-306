#include "GCode.hpp"
#include <vector>
#include <iostream>
using namespace std;

GCodeParser::GCodeParser() {}

void GCodeParser::ExecuteCommand(const std::string& cmd) {
    command = cmd;
    std::vector<std::string> tokens = tokenize(command);
    cout << "tokenisation done: " << endl;
    parameterExtraction(tokens);
    return;
}

std::vector<std::string> GCodeParser::tokenize(std::string& cmd) {
    //implement
    std::vector<std::string> tokens;
    std::string buffer;
    for (int i = 0; i < cmd.length(); i++){
        if (cmd[i] == ' ' /*|| cmd[i] == '\t') { */ ){
            if (!buffer.empty()) {
                cout << "Adding to tokens: " << buffer << endl;
                tokens.push_back(buffer);
                buffer.clear();
            }
        } else {
            buffer += cmd[i];
        }
    }
    if (!buffer.empty()) {
        cout << "Adding last token: " << buffer << endl;
        tokens.push_back(buffer);
    }
    return tokens;
}

void GCodeParser::parameterExtraction(std::vector<std::string>& tokens) {
    //implement
    if (tokens[0] == "G28") {
        // homing routine
        std::cout << "Executing homing routine for G28 command." << std::endl;
    }
    else if (tokens[0] == "M999") {
        //error reset
        std::cout << "Resetting error state for M999 command." << std::endl;
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
            }
        }
    }
    else {
        std::cout << "Unknown command: " << tokens[0] << std::endl;
        return;
    }
    cout << "Parameters extracted: " << endl;
    cout << "X: " << parameters[0] << ", Y: " << parameters[1]
    << ", F: " << parameters[2] << endl;
     return;
}

void GCodeParser::CaseCapitalize() {
    for (char& c : command) {
        c = toupper(c);
    }
}
