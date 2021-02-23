#ifndef __PID_CALC__
#define __PID_CALC__

#include <ctime>

class PID_Calculator
{
    private:

        float Kp, Ki, Kd;

        float last_error, integral;
        clock_t last_call_time;

    public:
        
        PID_Calculator();
        PID_Calculator(float Kp_val, float Ki_val, float Kd_val);

        void set_Kp(float val);
        void set_Ki(float val);
        void set_Kd(float val);

        void set_PID_gains(float K_p, float K_i, float K_d);

        float get_Kp();
        float get_Ki();
        float get_Kd();

        void reset();

        float calc_effort(float error);
};


#endif