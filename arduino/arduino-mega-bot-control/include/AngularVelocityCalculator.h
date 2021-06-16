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
    a 5th order scheme to estimate the first derivative with respect to time. 
*/

#ifndef __VELOCITY_CALC__
#define __VELOCITY_CALC__

#include "Arduino.h"

// This library is required for blocking interrupts
// while copying volatile variables in main execution
// path. Link - https://www.nongnu.org/avr-libc/user-manual/group__util__atomic.html
#include <util/atomic.h>

// This library contains the definition of Encoder class
#include "Encoder.h"

// Definition of class angularVelocityCalculator
// It is derived from Encoder class
class AngularVelocityCalculator : protected Encoder
{
    private:
        
        // This array of 5 elements stores last 5 positions
        // of the shaft read from the encoder at a certain
        // interval
        volatile float encoderReadingsArray[5];

        // The coefficients to the 5 position values
        // in the expression of the 5th order velocity estimation
        // scheme
        const float coefficient0;
        const float coefficient1;
        const float coefficient2;
        const float coefficient3;
        const float coefficient4;

    public:
        
        // Constructor
        // encoderPin1 - Pin no. to which Pin A of encoder is connected
        // encoderPin2 - Pin no. to which Pin B of encoder is connected
        // updateFrequency - Frequency at which new position value is read from the encoder
        // countsPerRotation - No. of quadrature turn counts of encoder to complete one shaft rotation
        AngularVelocityCalculator(
            uint8_t encoderPin1,
            uint8_t encoderPin2,
            float updateFrequency,
            float countsPerRotation
        );

        // This function is called at the specified frequency
        // to capture a new position reading of the encoder
        inline void updateAngularVelocity() __attribute__((always_inline));
        // The post-fix __attribute__((always_inline)) ensures
        // the function is definitely inlined by compiler

        // Returns latest angular velocity in radians per second
        inline float getAngularVelocity();
};

AngularVelocityCalculator::AngularVelocityCalculator(
    uint8_t encoderPin1,
    uint8_t encoderPin2,
    float updateFrequency,
    float countsPerRotation
) : Encoder(encoderPin1, encoderPin2),
    coefficient0(25.0f * updateFrequency * 2.0f * PI / (12.0f * countsPerRotation)),
    coefficient1(- 48.0f * updateFrequency * 2.0f * PI / (12.0f * countsPerRotation)),
    coefficient2(36.0f * updateFrequency * 2.0f * PI / (12.0f * countsPerRotation)),
    coefficient3(- 16.0f * updateFrequency * 2.0f * PI / (12.0f * countsPerRotation)),
    coefficient4(3.0f * updateFrequency * 2.0f * PI / (12.0f * countsPerRotation))
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        encoderReadingsArray[0] = 0;
        encoderReadingsArray[1] = 0;
        encoderReadingsArray[2] = 0;
        encoderReadingsArray[3] = 0;
        encoderReadingsArray[4] = 0;
    }
}

void AngularVelocityCalculator::updateAngularVelocity()
{
    // memmove(&encoderReadingsArray[1], &encoderReadingsArray[0], 4*sizeof(float));
    encoderReadingsArray[4] = encoderReadingsArray[3];
    encoderReadingsArray[3] = encoderReadingsArray[2];
    encoderReadingsArray[2] = encoderReadingsArray[1];
    encoderReadingsArray[1] = encoderReadingsArray[0];
    encoderReadingsArray[0] = read();
}

float AngularVelocityCalculator::getAngularVelocity()
{
    float angularVelocity;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        angularVelocity =
            coefficient0 * encoderReadingsArray[0] +
            coefficient1 * encoderReadingsArray[1] +
            coefficient2 * encoderReadingsArray[2] +
            coefficient3 * encoderReadingsArray[3] +
            coefficient4 * encoderReadingsArray[4];
    }
    return angularVelocity;
}

#endif