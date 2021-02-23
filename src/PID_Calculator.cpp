#include "PID_Calculator.hpp"

PID_Calculator::PID_Calculator() 
{
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;

    reset();
}

PID_Calculator::PID_Calculator(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;

    reset();
}

void PID_Calculator::reset()
{
    integral = 0.0;
    last_error = 0.0;
    last_call_time = clock();
}

void PID_Calculator::set_Kp(float val) { Kp = val; }
void PID_Calculator::set_Ki(float val) { Ki = val; }
void PID_Calculator::set_Kd(float val) { Kd = val; }

void PID_Calculator::set_PID_gains(float p, float i, float d)
{
    Kp = p;
    Ki = i;
    Kd = d;
}

float PID_Calculator::get_Kp() { return Kp; }
float PID_Calculator::get_Ki() { return Kp; }
float PID_Calculator::get_Kd() { return Kp; }


float PID_Calculator::calc_effort(float error)
{
    clock_t current_time = clock();

    float Dt = float(current_time - last_call_time) / CLOCKS_PER_SEC;
    
    last_call_time = current_time;

    integral += Ki * 0.5 * (error + last_error) * Dt;

    float retval  = Kp * error + integral + Kd * (error - last_error) / Dt;

    last_error = error;

    return retval;
}
