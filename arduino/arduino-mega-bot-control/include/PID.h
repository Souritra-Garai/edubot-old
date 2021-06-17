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

        float time_period_;

    public:

        PID();
        PID(
            float proportional_gain,
            float integral_gain,
            float differential_gain
        );

        void setPIDGains(
            float proportional_gain,
            float integral_gain,
            float differential_gain
        );

        
};

PID::PID(/* args */)
{
    
}

PID::~PID()
{
}

#endif