#include "GCode.hpp"
#include <Arduino.h>

GCodeParser::GCodeParser() {}

void GCodeParser::CaseCapitalize() {
    command.toUpperCase(); // Arduino String method
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

int GCodeParser::extractCommand(String tokens[], int tokenCount) {
    // if G28 command return homing state 
    if (tokens[0] == "G28") {
        return 1; // HOMING
    }
    // if M999 run homing command 
    else if (tokens[0] == "M999") {
        return 1; // HOMING
    }
    // if G01, extract parameters
    else if (tokens[0] == "G01") {
        extractParameters(tokens, tokenCount);
        // Serial.println("Parameters extracted:");
        // Serial.print("X: "); Serial.print(parameters[0]);
        // Serial.print(", Y: "); Serial.print(parameters[1]);
        // Serial.print(", F: "); Serial.println(parameters[2]);
        if (invalidCommand){
            return 0; // return to IDLE
        }
        return 2; // MOVING
    }
    else {
        Serial.print("Unknown command: ");
        Serial.println(tokens[0]);
        return 0; //return to IDLE
    }
    return 0;
}

void GCodeParser::extractParameters(String tokens[], int tokenCount){
    parameters[0] = 0.0f; // Reset X
    parameters[1] = 0.0f; // Reset Y
    previousFeedrate = parameters[2]; // capture lass feedrate 

    for (int i = 1; i < tokenCount; i++) {
            String token = tokens[i];
            if (token.charAt(0) == 'X') {
                parameters[0] = token.substring(1).toFloat();
            } 
            else if (token.charAt(0) == 'Y') {
                parameters[1] = token.substring(1).toFloat();
            } 
            else if (token.charAt(0) == 'F') {
                parameters[2] = token.substring(1).toFloat();                
            } 
            else if (token.charAt(0) == ';') {
                continue; // ignore comment
            }
            else {
                Serial.print("Unknown parameter: ");
                Serial.println(token);
                invalidCommand = true;
                return;
            }
        }
    return;
}

bool GCodeParser::ValidateParameters(float currentX, float currentY) {
    // validate x and y distances
    if ((parameters[0] + currentX) > 150 || 
        (parameters[1] + currentY) > 150 || 
        (parameters[1] + currentY) < 0 || 
        (parameters[0] + currentX) < 0) {
        Serial.println("Distance entered exceeds limits");
        invalidCommand = true;
    }
    // validate speed 
    if (parameters[2] == 0){
        Serial.print("Invalid speed entry: "); Serial.print(tokens[i]);
        parameters[2] = previousFeedrate; //return feedrate to old one
        invalidCommand = true;
    }
    //cap speed
    if (parameters[2] > 3000){
        parameters[2] = 3000;
    }
    if (invalidCommand){
        return false;
    }
    return true;
}

int GCodeParser::ExecuteCommand(const String& cmd) {
    invalidCommand = false;
    command = cmd;
    String tokens[10]; // fixed-size array for tokens
    int tokenCount = 0;
    CaseCapitalize();
    tokenize(command, tokens, tokenCount);
    return extractCommand(tokens, tokenCount);
}