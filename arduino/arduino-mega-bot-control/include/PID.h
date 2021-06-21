/*
    This library defines a class to implement 
    Proportional Integral and Differential control.
*/

#ifndef __PID__
#define __PID__

#include <Arduino.h>

// Class to implement PID control
// It is assumed that the control ouput is calculated at
// a fixed frequency
class PID
{
    private:
    
        // Gains for PID control
        float proportional_gain_;
        float integral_gain_;
        float differential_gain_;
        
        // Variables for storing error in the last iteration
        // and integral of error
        float last_error_;
        float integral_error_;
        
        // Variables for storing the set point value and
        // the value of the state variable in the last iteration
        float target_state_value_;
        float last_state_value_;
        // value of state variable in the last iteration is required
        // to determine derivative for differential control

        // Time period in which PID control is executed once
        const float time_period_;

    public:

        // Constructor
        // update_frequency - the frequency at which PID control is excuted
        PID(float update_frequency);

        // Function to modify PID gains for the controller
        // proportional_gain - gain for proportional control
        // integral_gain - gain for integral control
        // differential_gain - gain for differential control
        void setPIDGains(
            float proportional_gain,
            float integral_gain,
            float differential_gain
        );

        // Returns the PID control output
        // current_state_value - current value of the state variable
        inline float getControllerOutput(float current_state_value)  __attribute__((always_inline));
        // The post-fix __attribute__((always_inline)) ensures
        // the function is definitely inlined by compiler.

        float getError()
        {
            return last_error_;
        }
        // Change the set point
        // target_state_value - 
        void setTargetStateValue(float target_state_value);

        // Reset last stored error and integral error to zero
        void reset();
};

/*=====================================================================================================*/

// The inline functions need to be defined and declared in the same file.

// Evaluate and return the PID control output
// Also update the integral error, last error and last state value
float PID::getControllerOutput(float current_state_value)
{
    // Evaluate the error at the current iteration
    float current_error = target_state_value_ - current_state_value;
    
    // Evaluate the integral of error from time t = 0
    // using trapezoidal rule
    integral_error_ += 0.5 * (last_error_ + current_error) * time_period_;

    // Evaluate the controller output by multiplying
    // gains with corresponding errors
    float controller_output = 
        proportional_gain_  * current_error +
        integral_gain_      * integral_error_ +
        differential_gain_  * (last_state_value_ - current_state_value) / time_period_;
    // derivative for differential control is determined
    // by taking backward difference of the state variable

    // Update last value of state variable and error
    last_state_value_ = current_state_value;
    last_error_ = current_error;

    // return controller output
    return controller_output;
}

#endif