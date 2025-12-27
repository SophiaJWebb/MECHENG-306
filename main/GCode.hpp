#ifndef GCODE_H
#define GCODE_H

#include <Arduino.h> // Gives you String, Serial, etc.

// Optional: you can still use vectors if you have enough memory and include ArduinoSTL
//#include <vector>

class GCodeParser {
  private:
    String command;
    int feedrate = 1000; // Default feedrate
    float parameters[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z parameters

  public:
    GCodeParser();

    int ExecuteCommand(const String& cmd);
    void CaseCapitalize();
    void execute();
    // If you don't have std::vector, you can return an array or work directly on the String
    void tokenize(const String& cmd, String tokens[], int& tokenCount);
    int parameterExtraction(String tokens[], int tokenCount);
    const float* GetParameters() const { return parameters; }
    bool ValidateParameters(float currentX, float currentY);
};

#endif // GCODE_H