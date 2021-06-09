#include "Angular_Velocity_Calculator.h"

angularVelocityCalculator::angularVelocityCalculator(
    uint8_t pin1,
    uint8_t pin2,
    float32_t freq,
    flaot32_t cpr
) : Encoder(pin1, pin2), updateFrequency(freq), countsPerRotation(cpr)
{
    angularVelocity = 0.0f;

    memset(encoderReadingsArray, 0, 20);
}

inline void angularVelocityCalculator::pushReading(float32_t val)
{
    memmove(&encoderReadingsArray[1], &encoderReadingsArray[0], 16);
    
    encoderReadingsArray[0] = (float32_t) val;
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

float32_t angularVelocityCalculator::getAngularVelocity()
{
    return angularVelocity;
}