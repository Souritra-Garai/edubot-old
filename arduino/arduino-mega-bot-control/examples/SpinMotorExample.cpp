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

void initializeTimer3();

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

  initializeTimer3();

  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  // if (millis() - last_vel_update_time > 1000/VELOCITY_UPDATE_FREQUENCY)
  // {
  //   motor_controller.spinMotor();

  //   last_vel_update_time = millis();
  // }

  if (millis() - last_print_time > PRINT_TIME_PERIOD)
  {
    Serial.print("Velocity:\t");
    Serial.println(motor_controller.getMotorAngularVelocity());

    Serial.print("PID output:\t");
    Serial.println(motor_controller.getPIDControlOutput());

    Serial.print("Error: ");
    Serial.println(motor_controller.getError());

    last_print_time = millis();
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
  // Set compare match register (OCR3A) to 249
  OCR3A = 0x04E1;
  // Enable interrupt upon compare match of OCR3A
  TIMSK3 |= (0x01 << OCIE3A);

  // allow interrupts
  sei();
}

ISR(TIMER3_COMPA_vect)
{
  motor_controller.spinMotor();
}