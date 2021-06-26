/*
    This library defines a class to control the
    motor giving the PWM value and its direction.
*/

#ifndef __MOTOR_CONTROLLER__
#define __MOTOR_CONTROLLER__

// This library contains the definition of functions
// used for updating and extracting the value of angular
// velocity. 
#include "AngularVelocityCalculator.h"

// This library contains a class to implement Proportional
// Integra, and Differential control (PID).
#include "PID.h"

// Class to update PWM values onto Motor-connected Arduino
// pin.
class MotorController : public AngularVelocityCalculator, public PID
{
    private:

        // Arduino pins for PWM value and direction
        const uint8_t PWM_pin_;
        const uint8_t direction_pin_;

        // Variable for updated angular velocity 
        float angular_velocity_;
        float pid_output_;
        // Vaariable for controller output 

    public :

        // Update the PWM values 
        inline void spinMotor() __attribute__((always_inline));

        float angVel();
        float pidOut();

        // Constructor
        // PWMpin - Pin no. on which absolute value of PWM is to be written
        // directionPin - Pin no. on which sign (HIGH or LOW) of PWM value is to be written
        // encoderPin1 - Pin no. to which Pin A of encoder is connected
        // encoderPin2 - Pin no. to which Pin B of encoder is connected
        // updateFrequency - Frequency at which new position value is read from the encoder
        // countsPerRotation - No. of quadrature turn counts of encoder to complete one shaft rotation
        MotorController(
            uint8_t PWM_pin,
            uint8_t direction_pin,
            uint8_t encoder_pin_1,
            uint8_t encoder_pin_2,
            float update_frequency,
            float counts_per_rotation
        );

};

// Update and extract the angular velocity and calculate the pid
// control values for the same. 
// Write the PWM value and direction
void MotorController::spinMotor()
{
    // Update the angular velocity as per the new 
    // captured position of the encoder.
    updateAngularVelocity();
    getAngularVelocity(angular_velocity_);
    
    pid_output_ = round(getControllerOutput(angular_velocity_));

    // Write a HIGH or LOW value to the direction pin
    digitalWrite(direction_pin_, pid_output_>0?HIGH:LOW);

    // Write the absolute value of PWM value on the PWM
    // pin.
    analogWrite(PWM_pin_, min(abs(pid_output_),0xFF));
}

float MotorController::angVel()
{
    return angular_velocity_;
}

float MotorController::pidOut()
{
    return pid_output_;
}

// Constructor
MotorController::MotorController(
    uint8_t PWM_pin,
    uint8_t direction_pin,
    uint8_t encoder_pin_1,
    uint8_t encoder_pin_2,
    float update_frequency,
    float counts_per_rotation
) : // Mem initialization list
    // Calling the constructor of base classes
AngularVelocityCalculator(
    encoder_pin_1,
    encoder_pin_2,
    update_frequency,
    counts_per_rotation
), 
PID(update_frequency), 
// Initialize constant variables
PWM_pin_(PWM_pin),
direction_pin_(direction_pin)
{   
    // Initilize all the private variables to zero
    angular_velocity_ = 0;
    pid_output_ = 0;
}

#endif