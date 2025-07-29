#ifndef GCODE_H
#define GCODE_H

#include <iostream>
#include <string>

class GCode {
    private:
        std::string command;
    public:
        GCode(const std::string& cmd) : command(cmd) {}
        std::string GetCommand() const { return command; }
        void execute();
        
        // for testing purposes
        void print() const;
};

#endif // GCODE_H