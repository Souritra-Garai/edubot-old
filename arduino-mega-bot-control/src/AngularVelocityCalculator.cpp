#include "AngularVelocityCalculator.h"

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