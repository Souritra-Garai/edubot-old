#include "MotorController.h"

#define PWM_PIN 11
#define DIRECTION_PIN 22
#define ENCODER_PIN_A 18
#define ENCODER_PIN_B 20

#define VELOCITY_UPDATE_FREQUENCY 488 // Hz
#define ENCODER_COUNTS_PER_ROTATION 840

#define PRINT_TIME_PERIOD 2000 // ms

MotorController motor_controller(
  PWM_PIN,
  DIRECTION_PIN,
  ENCODER_PIN_A,
  ENCODER_PIN_B,
  VELOCITY_UPDATE_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);


long int last_print_time;
float velocity_update_duration_sum;
// Variable to store when the last velocity update 
// was performed in microsecond
float last_velocity_update_time;
// Variable to fetch and store the time during
// velocity update
float current_velocity_update_time;
// Variable to store the number of velocity updates 
// that are performed
float number_velocity_updates;

void initialize_timer_2();

void setup()
{
  Serial.begin(9600);

  last_print_time = millis();

  velocity_update_duration_sum = 0;
  current_velocity_update_time = 0;
  last_velocity_update_time = 0;
  number_velocity_updates = 0;

  motor_controller.setPIDGains(5, 0, 0);

  motor_controller.setTargetStateValue(10);

  initialize_timer_2();

  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  if (millis() - last_print_time > PRINT_TIME_PERIOD)
  {
    // motor_controller.spinMotor();
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      Serial.print("Velocity:\t");
      Serial.println(motor_controller.angVel());

      Serial.print("PID output:\t");
      Serial.println(motor_controller.pidOut());
      Serial.println(motor_controller.getError());

      Serial.print("Average Velocity Update Period:\t");
      Serial.println(velocity_update_duration_sum / number_velocity_updates);
    }

    last_print_time = millis();
  }
}

void initialize_timer_2()
{
  // stop interrupts
  cli();
  
  // Set timer2 to interrupt at 8kHz

  // ATmega2560 clock frequency - 16MHz
  // Prescalers available for timers - 1, 8, 64, 256, 1024
  // Timer clock frequency = ATmega2560 clock frequency / Prescaler
  
  // To have interrupts with better frequency resolution timer
  // will be operated in Clear Timer on Compare Match (CTC) mode
  // TCNTx will count from 0x00 to the value in OCRnA register
  // Frequency of Interrupt = Timer clock frequency / Number of TCNTn Counts before it resets to 0x00

  // TCNTx will be compared to OCRnx in each clock cycle
  // Upon compare match, interrupt is fired
  // For 1kHz, TCNTx needs - 
  //   - 250 counts at 64 prescaler
  
  // Turn on CTC mode
  TCCR2A |= (0x01 << WGM21);
  // Set Prescaler to 1024
  TCCR2B |= (0x01 << CS22);
  TCCR2B |= (0x01 << CS20);
  // Set compare match register (OCR2A) to 
  OCR2A = 0xF0;
  // Enable interrupt upon compare match of OCR2A
  TIMSK2 |= (0x01 << OCIE2A);

  // allow interrupts
  sei();
}

ISR(TIMER2_COMPA_vect)
{
  current_velocity_update_time = micros();
  velocity_update_duration_sum += current_velocity_update_time - last_velocity_update_time;
  last_velocity_update_time = current_velocity_update_time;
  number_velocity_updates += 1;
  motor_controller.spinMotor();
}