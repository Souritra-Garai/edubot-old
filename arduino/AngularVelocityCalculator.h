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
        
        void pushReading(float newReading);
        void calcAngularVelocity();

    public:
        
        angularVelocityCalculator(
            uint8_t encoderPin1,
            uint8_t encoderPin2,
            float updateFrequency,
            float countsPerRotation
        );

        void updateAngularVelocity();

        float getAngularVelocity();
};

#endif