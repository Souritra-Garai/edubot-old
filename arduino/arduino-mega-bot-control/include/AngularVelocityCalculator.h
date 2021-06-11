/* 
    This library is intended to calculate the angular velocity of a shaft using
    a rotary encoder. It derives from the Encoder class in the Encoder.h library
    by by Paul Stoffregen at https://www.pjrc.com/teensy/td_libs_Encoder.html
    
    The rotary encoder provides angular position of the shaft. The initial position 
    of the shaft, when the system boots up, is assumed to be zero. The Encoder type
    object increments / decrements the position, a 32-bit integer, in the range 
    -2147483648 to 2147483647, upon receiving a quadrature rotation from the rotary
    encoder.

    This library defines angularVelocityCalculator class. It determines the angular
    velocity at the current time step by using last 5 encoder position readings and
    a 5th order scheme to estimate the first derivative. 
*/

#ifndef __VELOCITY_CALC__
#define __VELOCITY_CALC__

#include "Arduino.h"
#include "Encoder.h"

class angularVelocityCalculator : protected Encoder
{
    private:
        
        float angularVelocity;
        float encoderReadingsArray[5];

        const float updateFrequency;
        const float countsPerRotation;
        
        inline void pushReading(float newReading) __attribute__((always_inline));
        inline void calcAngularVelocity() __attribute__((always_inline));

    public:
        
        angularVelocityCalculator(
            uint8_t encoderPin1,
            uint8_t encoderPin2,
            float updateFrequency,
            float countsPerRotation
        );

        inline void updateAngularVelocity() __attribute__((always_inline));

        inline float getAngularVelocity() __attribute__((always_inline));
};


angularVelocityCalculator::angularVelocityCalculator(
    uint8_t pin1,
    uint8_t pin2,
    float freq,
    float cpr
) : Encoder(pin1, pin2), updateFrequency(freq), countsPerRotation(cpr)
{
    angularVelocity = 0.0f;

    memset(encoderReadingsArray, 0, 5*sizeof(float));
}

void angularVelocityCalculator::pushReading(float val)
{
    memmove(&encoderReadingsArray[1], &encoderReadingsArray[0], 4*sizeof(float));
    
    encoderReadingsArray[0] = val;
}

void angularVelocityCalculator::calcAngularVelocity()
{
    angularVelocity = updateFrequency * (
        25.0f * encoderReadingsArray[0] -
        48.0f * encoderReadingsArray[1] +
        36.0f * encoderReadingsArray[2] -
        16.0f * encoderReadingsArray[3] +
        3.0f  * encoderReadingsArray[4]
    ) / (12.0f * countsPerRotation);
}

void angularVelocityCalculator::updateAngularVelocity()
{
    pushReading((float) read());
    calcAngularVelocity();
}

float angularVelocityCalculator::getAngularVelocity()
{
    return angularVelocity;
}

#endif