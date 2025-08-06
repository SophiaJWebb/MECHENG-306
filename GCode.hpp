#ifndef GCODE_H
#define GCODE_H

#include <iostream>
#include <vector>
#include <string>

class GCodeParser{
    private:
        std::string command;
        int feedrate = 1000; // Default feedrate
        float parameters[3] = {0.0f, 0.0f, 0.0f}; // X, Y, Z parameters
    public:
        GCodeParser();
        int ExecuteCommand(const std::string& cmd);
        void CaseCapitalize();
        void execute();
        std::vector<std::string> tokenize(std::string& cmd);
        int parameterExtraction(std::vector<std::string>& tokens);
        const float* GetParameters() const { return parameters; }
        bool ValidateParameters(float currentX, float currentY);

};

#endif // GCODE_H