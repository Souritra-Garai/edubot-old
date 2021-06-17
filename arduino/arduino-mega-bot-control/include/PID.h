#ifndef __PID__
#define __PID__

#include <Arduino.h>

class PID
{
    private:
    
        float proportional_gain_;
        float integral_gain_;
        float differential_gain_;
        
        float last_error_;
        float integral_error_;
        
        float target_state_value_;
        float last_state_value_;

        const float time_period_;

    public:

        PID(float update_frequency);

        void setPIDGains(
            float proportional_gain,
            float integral_gain,
            float differential_gain
        );

        inline float getControllerOutput(float current_state_value)  __attribute__((always_inline));
        
        void setPIDStateTargetValue(float target_state_value);

        void reset();
};

/*=====================================================================================================*/

float PID::getControllerOutput(float current_state_value)
{
    float current_error = target_state_value_ - current_state_value;
    
    integral_error_ += 0.5 * (last_error_ + current_error) * time_period_;

    float controller_output = 
        proportional_gain_  * current_error +
        integral_gain_      * integral_error_ +
        differential_gain_  * (last_state_value_ - current_state_value) / time_period_;

    last_state_value_ = current_state_value;
    last_error_ = current_error;

    return controller_output;
}

#endif