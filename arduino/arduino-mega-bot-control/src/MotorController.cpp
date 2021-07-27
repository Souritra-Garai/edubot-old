#include "MotorController.h"

// Constructor
MotorController::MotorController(
    uint8_t direction_pin,
    uint8_t encoder_pin_1,
    uint8_t encoder_pin_2,
    float update_frequency,
    float counts_per_rotation
) : // Call base class constructors
    AngularVelocityCalculator(
        encoder_pin_1,
        encoder_pin_2,
        update_frequency,
        counts_per_rotation
    ), PID(update_frequency),
    // Initialize const variables
    direction_pin_(direction_pin)
{
    // Initialize angular_velocity and PID output to zero
    angular_velocity_ = 0;
    PID_output_ = 0;

    // Initialise the motors in stopping position
    stopMotor();
}

void MotorController::stopMotor()
{
    // Disable PID control
    PID_control_enable_ = false;
    // Set duty cyle for PWM output to 0
    PID_output_ = 0;
}

void MotorController::enablePIDControl()
{
    // Reset PID
    reset();
    // Enable PID
    PID_control_enable_ = true;
}

float MotorController::getMotorAngularVelocity()
{
    // Return the last computed copy of angular velocity
    return angular_velocity_;
}