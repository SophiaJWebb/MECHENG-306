#include <Arduino.h> // Gives you String, Serial, etc.

#ifndef GCode 
class GCodeParser {
  private:
    float parameters[3] = {0.0f, 0.0f, 1000.0f}; // X, Y, Z parameters
    bool invalidCommand = false;

  public:
    GCodeParser();

    int ExecuteCommand(const String& cmd);
    void CaseCapitalize();
    void tokenize(const String& cmd, String tokens[], int& tokenCount);
    int extractCommand(String tokens[], int tokenCount);
    void extractParameters(String tokens[], int tokenCount);
    bool ValidateParameters(float currentX, float currentY);
    const float* GetParameters() const { return parameters; }
};


#endif // GCODE_H

