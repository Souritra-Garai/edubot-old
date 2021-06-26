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

#define PWM_PIN 11
#define DIRECTION_PIN 22
#define ENCODER_PIN_A 18
#define ENCODER_PIN_B 20

/**
 * @brief Frequency at which velocity is updated
 * 
 */
#define VELOCITY_UPDATE_FREQUENCY 50 // Hz

#define ENCODER_COUNTS_PER_ROTATION 840

/**
 * @brief Time interval at which anything is printed to serial monitor.

 * 
 */
#define PRINT_TIME_PERIOD 2000 // ms

/**
 * @brief Our object for motor controller
 * 
 * @return MotorController 
 */
MotorController motor_controller(
  PWM_PIN,
  DIRECTION_PIN,
  ENCODER_PIN_A,
  ENCODER_PIN_B,
  VELOCITY_UPDATE_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);

/**
 * @brief Variable to store the last time anything was
 * printed through the serial port in millisecond.
 * 
 */
long int last_print_time;

/**
 * @brief Variable to store the last time the angular 
 * velocity was updated in millisecond.
 * 
 */
long int last_vel_update_time;

void initialize_timer_2();

void setup()
{
  /**
   * @brief Initialise Serial Comm
   * 
   */
  Serial.begin(9600);

  last_vel_update_time = millis();
  last_print_time = millis();

  /**
   * @brief Set PID gains to P=15, I=120 and D=0 
   * 
   */
  motor_controller.setPIDGains(15, 120, 0);

  /**
   * @brief Set target angular velocity
   * 
   */
  motor_controller.setTargetStateValue(10);

  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  if (millis() - last_vel_update_time > 1000/VELOCITY_UPDATE_FREQUENCY)
  {
    motor_controller.spinMotor();

    last_vel_update_time = millis();
  }

  if (millis() - last_print_time > PRINT_TIME_PERIOD)
  {
    Serial.print("Velocity:\t");
    Serial.println(motor_controller.angVel());

    Serial.print("PID output:\t");
    Serial.println(motor_controller.pidOut());

    Serial.print("Error: ");
    Serial.println(motor_controller.getError());

    last_print_time = millis();
  }
}