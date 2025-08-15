#include "GCode.hpp"
#include <Arduino.h>

GCodeParser::GCodeParser() {}

void GCodeParser::CaseCapitalize() {
    command.toUpperCase(); // Arduino String method
}

int GCodeParser::ExecuteCommand(const String& cmd) {
    command = cmd;
    String tokens[10]; // fixed-size array for tokens
    int tokenCount = 0;
    tokenize(command, tokens, tokenCount);
    CaseCapitalize();
    return parameterExtraction(tokens, tokenCount);
}

void GCodeParser::tokenize(const String& cmd, String tokens[], int& tokenCount) {
    tokenCount = 0;
    String buffer = "";
    for (unsigned int i = 0; i < cmd.length(); i++) {
        char c = cmd.charAt(i);
        if (c == ' ') {
            if (buffer.length() > 0) {
                tokens[tokenCount++] = buffer;
                buffer = "";
            }
        } else {
            buffer += c;
        }
    }
    if (buffer.length() > 0) {
        tokens[tokenCount++] = buffer;
    }
}

int GCodeParser::parameterExtraction(String tokens[], int tokenCount) {
    if (tokens[0] == "G28") {
        Serial.println("Changing to homing state for G28 command.");
        return 1; // HOMING
    }
    else if (tokens[0] == "M999") {
        Serial.println("Entering homing state for M999 command.");
        return 1; // HOMING
    }
    else if (tokens[0] == "G01") {
        Serial.println("Executing G01 command.");
        parameters[0] = 0.0f; // Reset X
        parameters[1] = 0.0f; // Reset Y
        for (int i = 1; i < tokenCount; i++) {
            String token = tokens[i];
            if (token.charAt(0) == 'X') {
                parameters[0] = token.substring(1).toFloat();
            } 
            else if (token.charAt(0) == 'Y') {
                parameters[1] = token.substring(1).toFloat();
            } 
            else if (token.charAt(0) == 'F') {
                int speed = token.substring(1).toFloat();
                if (speed == 0){
                    Serial.print("Invalid speed entry: "); Serial.print(tokens[i]);
                }
                
                if (speed < 3000){
                    parameters[2] = speed;
                }
                else {
                    parameters[2] = 3000;
                }
                
            } 
            else if (token.charAt(0) == ';') {
                continue; // ignore comment
            }
            else {
                Serial.print("Unknown parameter: ");
                Serial.println(token);
                return 0;
            }
        }
        Serial.println("Parameters extracted:");
        Serial.print("X: "); Serial.print(parameters[0]);
        Serial.print(", Y: "); Serial.print(parameters[1]);
        Serial.print(", F: "); Serial.println(parameters[2]);
        return 2; // MOVING
    }
    else {
        Serial.print("Unknown command: ");
        Serial.println(tokens[0]);
        return 0;
    }
}

bool GCodeParser::ValidateParameters(float currentX, float currentY) {
    if ((parameters[0] + currentX) > 150 || 
        (parameters[1] + currentY) > 150 || 
        (parameters[1] + currentY) < 0 || 
        (parameters[0] + currentX) < 0) {
        return false;
    }
    return true;
}