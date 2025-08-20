#include "Encoder.h"
#include <Arduino.h>

Encoder::Encoder()
{
    encoderTicks_ = 0;
    millimeters_ = 0.0;
    lastEncoderTicks_ = 0;
}

Encoder::~Encoder()
{

}

void Encoder::resetEncoder()
{
    encoderTicks_ = 0;
    millimeters_ = 0.0;
    lastEncoderTicks_ = 0;
}

long Encoder::getLastEncoderTicks() const
{
    return lastEncoderTicks_;
}

void Encoder::setLastEncoderTicks(long ticks)
{
    lastEncoderTicks_ = ticks;
}

long Encoder::convertMillimetersToTicks(double millimeters)
{
    long convertedTicks = millimeters * millimetersToTicks_;
    return convertedTicks;
}

double Encoder::convertTicksToMillimeters(long ticks)
{
    double convertedMillimeters = ticks / millimetersToTicks_;
    return convertedMillimeters;
}

void Encoder::countTicks()
{
    encoderTicks_++;
    millimeters_ = convertTicksToMillimeters(encoderTicks_);
}

long Encoder::getEncoderTicks() const
{
    return encoderTicks_;
}

double Encoder::getMillimeters() const
{
    return millimeters_;
}

