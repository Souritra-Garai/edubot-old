/**
 * @file SpinMotorExample.cpp
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in)
 * @brief Example for running PID controls for a target velocity.
 * The angular velocity is updated at a rate of 50Hz.  PID output
 * and angular velocity is printed at every 2000 ms rate.
 * @version 0.1
 * @date 2021-06-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "MotorController.h"
#include "PhaseCorrect16BitPWM.h"

#define DIRECTION_PIN 22
#define ENCODER_PIN_A 18
#define ENCODER_PIN_B 20

#define SERIAL_PRINT_TIME_PERIOD 2000

#define VELOCITY_UPDATE_FREQUENCY 50 // Hz

#define ENCODER_COUNTS_PER_ROTATION 840

// Time interval at which anything is printed to serial monitor.
#define PRINT_TIME_PERIOD 2000 // ms

MotorController motor_controller(
  DIRECTION_PIN,
  ENCODER_PIN_A,
  ENCODER_PIN_B,
  VELOCITY_UPDATE_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);

long int last_serial_print_time;
long int last_PID_set_time;
long int last_update_time;

double velocity_get_num_observations;
double PID_set_time_period_sum;
double PID_set_time_period_sqr_sum;

double velocity_update_num_observations;
double motor_controller_spin_time_period_sum;
double motor_controller_spin_time_period_sqr_sum;

double start_time;
double duration;

void setup()
{
  Serial.begin(115200);

  Timer1PhaseCorrectPWM::clearTimerSettings();
  Timer1PhaseCorrectPWM::setupTimer();
  Timer1PhaseCorrectPWM::setupChannelA();

  last_serial_print_time = millis();
  last_PID_set_time = micros();
  last_update_time = micros();

  velocity_get_num_observations = 0;
  PID_set_time_period_sum = 0;
  PID_set_time_period_sqr_sum = 0;

  velocity_update_num_observations = 0;
  motor_controller_spin_time_period_sum = 0;
  motor_controller_spin_time_period_sqr_sum = 0;

  motor_controller.setPIDGains(5, 120, 0);

  motor_controller.setTargetStateValue(-10);

  Serial.println("Arduino Initialized successfully");

  motor_controller.enablePIDControl();
}

void loop()
{
  if (millis() - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
  {
    double mean_motor_spin_time = motor_controller_spin_time_period_sum / velocity_update_num_observations;
    double std_dev_motor_spin_time = sqrt(motor_controller_spin_time_period_sqr_sum / velocity_update_num_observations - mean_motor_spin_time * mean_motor_spin_time);

    double mean_PID_set_time = PID_set_time_period_sum / velocity_get_num_observations;
    double std_dev_PID_set_time = sqrt(PID_set_time_period_sqr_sum / velocity_get_num_observations - mean_PID_set_time * mean_PID_set_time);

    Serial.print("Time taken for Motor Spin :\tMean\t");
    Serial.print(mean_motor_spin_time);
    Serial.print("\tStd Dev\t");
    Serial.println(std_dev_motor_spin_time);

    Serial.print("Time taken for PID set :\tMean\t");
    Serial.print(mean_PID_set_time);
    Serial.print("\tStd Dev\t");
    Serial.println(std_dev_PID_set_time);    
    
    last_serial_print_time = millis();
  }

  if (millis() - last_update_time > 20)
  {
    start_time = micros();
    motor_controller.spinMotor();
    duration = micros() - start_time;

    motor_controller_spin_time_period_sum += duration;
    motor_controller_spin_time_period_sqr_sum += duration * duration;
    velocity_update_num_observations += 1;

    start_time = micros();
    Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA(motor_controller.getPIDControlOutput());
    duration = micros() - start_time;

    PID_set_time_period_sum += duration;
    PID_set_time_period_sqr_sum += duration * duration;
    velocity_get_num_observations += 1;
    
    last_update_time = millis();
  }
}
