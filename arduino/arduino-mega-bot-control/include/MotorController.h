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
class MotorController : public AngularVelocityCalculator, PID
{
    private:

        // Arduino pins for PWM value and direction
        uint8_t PWM_pin_;
        uint8_t direction_pin_;

        // Variable for updated angular velocity 
        float angular_velocity_;
        float pid_output_;
        // Vaariable for controller output 

    public :

        // Update the PWM values 
        void spin_motor()
        {
            // Update the angular velocity as per the new 
            // captured position of the encoder.
            updateAngularVelocity();
            angular_velocity_ = getAngularVelocity();
            
            pid_output_ = getControllerOutput(angular_velocity_);

            // Write a HIGH or LOW value to the direction pin
            digitalWrite(direction_pin_, pid_output_>0?HIGH:LOW);

            // Write the absolute value of PWM value on the PWM
            // pin.
            analogWrite(PWM_pin_, abs(pid_output_));
        };

};

#endif