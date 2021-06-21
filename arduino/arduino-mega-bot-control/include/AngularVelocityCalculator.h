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
class AngularVelocityCalculator : public Encoder
{
    private:
        
        // This array of 5 elements stores last 5 positions
        // of the shaft read from the encoder at a certain
        // interval
        volatile float encoder_readings_array_[5];
        // Since the values will be updated at a very high frequency
        // making the array volatile tells the compiler not to optimize
        // the reading from array task. Usually a variable that is used
        // frequently, is stored in a cache close to the processor. But
        // here our variable is updated more frequently than it is read.
        // Hence, it needs to be read from memory every time.


        // Time interval after which a new encoder position
        // value is inserted to
        const float time_period_;

        // Number of counts of change in encoder position to
        // complete one full rotation of the wheel
        const float counts_per_rotation_;

    public:
        
        // Constructor
        // encoderPin1 - Pin no. to which Pin A of encoder is connected
        // encoderPin2 - Pin no. to which Pin B of encoder is connected
        // updateFrequency - Frequency at which new position value is read from the encoder
        // countsPerRotation - No. of quadrature turn counts of encoder to complete one shaft rotation
        AngularVelocityCalculator(
            uint8_t encoder_pin_1,
            uint8_t encoder_pin_2,
            float update_frequency,
            float counts_per_rotation
        );

        // This function is called at the specified frequency
        // to capture a new position reading of the encoder
        inline void updateAngularVelocity() __attribute__((always_inline));
        // The post-fix __attribute__((always_inline)) ensures
        // the function is definitely inlined by compiler.
        // Executes under 15 us

        // Sets the value of the passed float variable angularVelocity
        // to latest angular velocity in radians per second
        inline void getAngularVelocity(float angular_velocity) __attribute__((always_inline));
        // Executes under 15 us
};

/*============================================================================================================*/

// The definition of the member functions of this class is done in the header file itself
// since compiler could not inline functions that are defined and declared in separate files.
// Inlining the functions are absolutely necessary as funciton call overheads cost up to 100 us (ATmega2560)
// whereas executing the tasks inside the functions take up to 10 us

// Constructor
AngularVelocityCalculator::AngularVelocityCalculator(
    uint8_t encoder_pin_1,
    uint8_t encoder_pin_2,
    float update_frequency,
    float counts_per_rotation
) : // Initialize the base Encoder class
    // and constant variables in the mem initialization list
    Encoder(encoder_pin_1, encoder_pin_2),
    time_period_(1 / update_frequency),
    counts_per_rotation_(counts_per_rotation)
{
    // Initialize all the elements in the 
    // encoder readings array to zero
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Atomic block prevents any interrupts from
        // stopping the initializing task
        encoder_readings_array_[0] = 0.0f;
        encoder_readings_array_[1] = 0.0f;
        encoder_readings_array_[2] = 0.0f;
        encoder_readings_array_[3] = 0.0f;
        encoder_readings_array_[4] = 0.0f;
        // Atomic restorestate will restore the status
        // of interrupts to whatever it was before it stopped
        // all interrupts, i.e. enabled / disabled.
    }
}

// Read the shaft position from the encoder
// and place it at the front of the encoder readings array
// while shifting other elements one place back
void AngularVelocityCalculator::updateAngularVelocity()
{
    // memmove(&encoderReadingsArray[1], &encoderReadingsArray[0], 4*sizeof(float));
    // As encoder_readings_array_ is declared as volatile,
    // memmove function cannot be used
    encoder_readings_array_[4] = encoder_readings_array_[3];
    encoder_readings_array_[3] = encoder_readings_array_[2];
    encoder_readings_array_[2] = encoder_readings_array_[1];
    encoder_readings_array_[1] = encoder_readings_array_[0];
    encoder_readings_array_[0] = (float) read();
}

// Calculate and set the angular velocity from  
// five encoder readings using a fifth order scheme
void AngularVelocityCalculator::getAngularVelocity(float angular_velocity)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Atomic block prevents any interrupts from
        // stopping the initializing task
        
        // Estimate angular velocity using 5th order scheme
        angular_velocity = 2 * PI * (
            25.0f * encoder_readings_array_[0] -
            48.0f * encoder_readings_array_[1] +
            36.0f * encoder_readings_array_[2] -
            16.0f * encoder_readings_array_[3] +
            3.0f  * encoder_readings_array_[4]
        ) / (12.0f * time_period_ * counts_per_rotation_);

        // Bharg's Formula
        // angular_velocity = 2 * PI * (
        //     -200.0f * encoder_readings_array_[0] -
        //     -100.0f * encoder_readings_array_[1] +
        //     0.0f    * encoder_readings_array_[2] -
        //     100.0f  * encoder_readings_array_[3] +
        //     200.0f  * encoder_readings_array_[4]
        // ) / 560.0f;
        
        // // For debug, simple 1st Order backward scheme
        // // to estimate velocity
        // angular_velocity = 60.0f * (float) (
        //     encoder_readings_array_[0] -
        //     encoder_readings_array_[1]
        // ) / (time_period_ * counts_per_rotation_);
        
        Serial.print("Time Period:\t");
        Serial.println(time_period_, 10);
        Serial.print("Counts per Rotation:\t");
        Serial.println(counts_per_rotation_);
        Serial.println(encoder_readings_array_[0]);
        Serial.println(encoder_readings_array_[1]);
        Serial.println(encoder_readings_array_[2]);
        Serial.println(encoder_readings_array_[3]);
        Serial.println(encoder_readings_array_[4]);

        // Atomic restorestate will restore the status
        // of interrupts to whatever it was before it stopped
        // all interrupts, i.e. enabled / disabled.
    }
}

#endif