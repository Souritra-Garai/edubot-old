/**
 * @file VelocityCalculationTimeTest.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This is an example code to measure the time taken for the
 * updateAngularVelocity and getAngularVelocity class methods of
 * AngularVelocityCalculator class to execute.
 * @version 0.1
 * @date 2021-07-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "AngularVelocityCalculator.h"

#define SERIAL_PRINT_TIME_PERIOD 2000
#define VELOCITY_GET_TIME_PERIOD 5000
#define VELOCITY_UPDATE_TIME_PERIOD 50

float angular_velocity;

AngularVelocityCalculator encoder_shaft(21, 19, 1 / VELOCITY_UPDATE_TIME_PERIOD, 560);

long int last_serial_print_time;
long int last_velocity_get_time;
long int last_velocity_update_time;

double velocity_get_num_observations;
double velocity_get_time_period_sum;
double velocity_get_time_period_sqr_sum;

double velocity_update_num_observations;
double velocity_update_time_period_sum;
double velocity_update_time_period_sqr_sum;

double start_time;
double duration;

void setup()
{
  Serial.begin(115200);
  last_serial_print_time = millis();
  last_velocity_get_time = micros();
  last_velocity_update_time = micros();

  velocity_get_num_observations = 0;
  velocity_get_time_period_sum = 0;
  velocity_get_time_period_sqr_sum = 0;

  velocity_update_num_observations = 0;
  velocity_update_time_period_sum = 0;
  velocity_update_time_period_sqr_sum = 0;
  
  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  
  if (millis() - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
  {
    double mean_velocity_update_time = velocity_update_time_period_sum / velocity_update_num_observations;
    double std_dev_velocity_update_time = sqrt(velocity_update_time_period_sqr_sum / velocity_update_num_observations - mean_velocity_update_time * mean_velocity_update_time);

    double mean_velocity_get_time = velocity_get_time_period_sum / velocity_get_num_observations;
    double std_dev_velocity_get_time = sqrt(velocity_get_time_period_sqr_sum / velocity_get_num_observations - mean_velocity_get_time * mean_velocity_get_time);

    Serial.print("Time taken for Velocity Update :\tMean\t");
    Serial.print(mean_velocity_update_time);
    Serial.print("\tStd Dev\t");
    Serial.println(std_dev_velocity_update_time);

    Serial.print("Time taken for Velocity Get :\t\tMean\t");
    Serial.print(mean_velocity_get_time);
    Serial.print("\tStd Dev\t");
    Serial.println(std_dev_velocity_get_time);    
    
    last_serial_print_time = millis();
  }

  if (micros() - last_velocity_update_time > VELOCITY_UPDATE_TIME_PERIOD)
  {
    start_time = micros();
    encoder_shaft.updateAngularVelocity();
    duration = micros() - start_time;

    velocity_update_time_period_sum += duration;
    velocity_update_time_period_sqr_sum += duration * duration;
    velocity_update_num_observations += 1;

    last_velocity_update_time = micros();
  }

  if (micros() - last_velocity_get_time > VELOCITY_GET_TIME_PERIOD)
  {
    start_time = micros();
    encoder_shaft.getAngularVelocity(angular_velocity);
    duration = micros() - start_time;

    velocity_get_time_period_sum += duration;
    velocity_get_time_period_sqr_sum += duration * duration;
    velocity_get_num_observations += 1;

    last_velocity_get_time = micros();
  }
}