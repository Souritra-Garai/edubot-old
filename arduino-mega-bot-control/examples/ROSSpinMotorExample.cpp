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

#include <Arduino.h>

#include <ros.h>
#include <std_srvs/Trigger.h>

#include "ROSMotorController.h"
#include "PhaseCorrect16BitPWM.h"

#define DIRECTION_PIN 22
#define ENCODER_PIN_A 18
#define ENCODER_PIN_B 20

#define VELOCITY_UPDATE_FREQUENCY 50 // Hz

#define ENCODER_COUNTS_PER_ROTATION 840

// Time interval at which anything is printed to serial monitor.
#define ROS_CYCLE_TIME_PERIOD 100 // ms

ROSMotorController motor_controller(
  "edubot",
  DIRECTION_PIN,
  ENCODER_PIN_A,
  ENCODER_PIN_B,
  VELOCITY_UPDATE_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);

ros::NodeHandle node_handle;

void stopBotServiceCallback(const std_srvs::TriggerRequest&, std_srvs::TriggerResponse&);
void moveBotServiceCallback(const std_srvs::TriggerRequest&, std_srvs::TriggerResponse&);

ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
  stopBotServiceServer("edubot/stop_bot", &stopBotServiceCallback);
ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse>
  moveBotServiceServer("edubot/move_bot", &moveBotServiceCallback);

// Variable to store the last time anything was
// printed through the serial port in millisecond.
long int prev_ros_cylce_time;

void initializeTimer3();

void setup()
{
  Timer1PhaseCorrectPWM::clearTimerSettings();
  Timer1PhaseCorrectPWM::setupTimer();
  Timer1PhaseCorrectPWM::setupChannelA();

  node_handle.initNode();
  node_handle.loginfo("Arduino node initialised");

  motor_controller.initialize_publishers_and_subscribers(node_handle);
  motor_controller.setPIDGains(5, 120, 0);

  node_handle.advertiseService(stopBotServiceServer);
  node_handle.advertiseService(moveBotServiceServer);

  initializeTimer3();

  node_handle.loginfo("Arduino Initialized successfully");
  prev_ros_cylce_time = millis();
}

void loop()
{
  if (millis() - prev_ros_cylce_time > ROS_CYCLE_TIME_PERIOD)
  {
    motor_controller.publish();

    node_handle.spinOnce();

    prev_ros_cylce_time = millis();
  }
}

void initializeTimer3()
{
  // stop interrupts
  cli();

  // Clear Timer/Counter Control Resgisters
  TCCR3A &= 0x00;
  TCCR3B &= 0x00;
  TCCR3C &= 0x00;
  // Clear Timer/Counter Register
  TCNT3 &= 0x0000;
  
  // Set Timer1 to interrupt at 50Hz

  // ATmega2560 clock frequency - 16MHz
  // Prescalers available for timers - 1, 8, 64, 256, 1024
  // Timer clock frequency = ATmega2560 clock frequency / Prescaler
  
  // To have interrupts with better frequency resolution timer
  // will be operated in Clear Timer on Compare Match (CTC) mode
  // TCNTx will count from 0x00 to the value in OCRnA register
  // Frequency of Interrupt = Timer clock frequency / Number of TCNTn Counts before it resets to 0x00

  // TCNTx will be compared to OCRnx in each clock cycle
  // Upon compare match, interrupt is fired
  // For 50Hz, TCNTx needs - 
  //   - 40000 counts at 8 prescaler
  //   - 5000 counts at 64 prescaler
  //   - 1250 counts at 256 prescaler
  //   - 312.5 counts at 1024 prescaler
  
  // Turn on CTC mode
  TCCR3B |= (0x01 << WGM32);
  // Set Prescaler to 256
  TCCR3B |= (0x01 << CS32);
  // Set compare match register (OCR3A) to 1250-1
  OCR3A = 0x04E1;
  // Enable interrupt upon compare match of OCR3A
  TIMSK3 |= (0x01 << OCIE3A);

  // allow interrupts
  sei();
}

ISR(TIMER3_COMPA_vect)
{
  motor_controller.updateAngularVelocity();
  motor_controller.spinMotor();
  Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA(motor_controller.getPIDControlOutput());
}

void stopBotServiceCallback(const std_srvs::TriggerRequest&, std_srvs::TriggerResponse& response)
{
  motor_controller.stopMotor();
  response.success = true;
  response.message = "Bot stopped successfully";
}

void moveBotServiceCallback(const std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response)
{
  motor_controller.enablePIDControl();
  response.success = true;
  response.message = "Bot velocity PID control enabled";
}