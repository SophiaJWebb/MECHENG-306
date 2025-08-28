#ifndef ENCODER_H
#define ENCODER_H

#pragma once

class Encoder
{
public:
    Encoder();
    void countTicks(int amount);
    void resetEncoder();
    long getEncoderTicks() const;
    double getMillimeters() const;
    double convertTicksToMillimeters(long ticks);
    long convertMillimetersToTicks(double millimeters);
    void setLastEncoderTicks(long ticks);
    long getLastEncoderTicks() const;
    
    ~Encoder();


private:
    long absoluteEncoderTicks_;
    long encoderTicks_;
    double millimeters_;
    const double millimetersToTicks_ = 65.618946; // For 1mm 65.61 ticks are needed
    long lastEncoderTicks_;
};

#endif