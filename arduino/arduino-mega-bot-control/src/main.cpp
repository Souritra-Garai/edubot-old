#include "AngularVelocityCalculator.h"
// #include "AngularVelocityCalculator.cpp"

#define SERIAL_PRINT_TIME_PERIOD 2000
#define VEL_CALC_FREQ 8000

AngularVelocityCalculator encoderShaft(2, 3, VEL_CALC_FREQ, 30);

long int lastSerialPrintTime;
long double lastVelCalcTime;

double i;
double time_period_sum;
double time_period_sqr_sum;

double start_time;
double vel_update_duration;

void setup()
{
  Serial.begin(9600);
  lastSerialPrintTime = millis();
  lastVelCalcTime = micros();

  i = 0;
  time_period_sum = 0;
  time_period_sqr_sum = 0;
  
  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  
  if (millis() - lastSerialPrintTime > SERIAL_PRINT_TIME_PERIOD)
  {
    double average_time_period = time_period_sum / i;
    
    Serial.println("Average Time Period (us) : ");
    Serial.println(average_time_period);

    Serial.println("Standard Deviation (us) : ");
    Serial.println(sqrt(time_period_sqr_sum/i - average_time_period*average_time_period));

    // Serial.println(encoderShaft.read());
    lastSerialPrintTime = millis();
  }

  if (micros() - lastVelCalcTime > 1/VEL_CALC_FREQ)
  {
    start_time = micros();
    encoderShaft.updateAngularVelocity();
    vel_update_duration = micros() - start_time;

    time_period_sum += vel_update_duration;
    time_period_sqr_sum += vel_update_duration * vel_update_duration;
    i += 1;

    lastVelCalcTime = micros();
  }
}