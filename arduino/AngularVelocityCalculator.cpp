#include "AngularVelocityCalculator.h"

angularVelocityCalculator::angularVelocityCalculator(
    uint8_t pin1,
    uint8_t pin2,
    float freq,
    float cpr
) : Encoder(pin1, pin2), updateFrequency(freq), countsPerRotation(cpr)
{
    angularVelocity = 0.0f;

    memset(encoderReadingsArray, 0, 5*sizeof(float);
}

inline void angularVelocityCalculator::pushReading(float val)
{
    memmove(&encoderReadingsArray[1], &encoderReadingsArray[0], 4*sizeof(float);
    
    encoderReadingsArray[0] = (float) val;
}

inline void angularVelocityCalculator::calcAngularVelocity()
{
    angularVelocity = updateFrequency * (
        25.0f * encoderReadingsArray[0] -
        48.0f * encoderReadingsArray[1] +
        36.0f * encoderReadingsArray[2] -
        16.0f * encoderReadingsArray[3] +
        3.0f  * encoderReadingsArray[4]
    ) / (12.0f * countsPerRotation);
}

inline void angularVelocityCalculator::updateAngularVelocity()
{
    pushReading(read());
    calcAngularVelocity();
}

inline float angularVelocityCalculator::getAngularVelocity()
{
    return angularVelocity;
}