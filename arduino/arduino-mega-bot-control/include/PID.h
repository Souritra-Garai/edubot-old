#ifndef __PID__
#define __PID__

#include <Arduino.h>

class PID
{
    private:
    
        float proportionalGain;
        float integralGain;
        float differentialGain;
        
        float lastError;
        float IntegralError;
        float lastStateValue;

        float lastCallTime;

    public:
        PID(/* args */);
        ~PID();
};

PID::PID(/* args */)
{
    
}

PID::~PID()
{
}

#endif