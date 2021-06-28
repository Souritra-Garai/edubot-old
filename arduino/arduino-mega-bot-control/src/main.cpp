#include "MotorController.h"

#define PWM_PIN 11
#define DIRECTION_PIN 22
#define ENCODER_PIN_A 18
#define ENCODER_PIN_B 20

#define VELOCITY_UPDATE_FREQUENCY 50 // Hz
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
long int last_vel_update_time;

void initialize_timer_2();

void setup()
{
  Serial.begin(9600);

  // initialize_timer_2();
  
  last_vel_update_time = millis();
  last_print_time = millis();

  motor_controller.setPIDGains(15, 120, 0);

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

void initialize_timer_2()
{
  // stop interrupts
  cli();

  // Clear Timer/Counter Control Resgisters
  TCCR2A &= 0x00;
  TCCR2B &= 0x00;
  // Clear Timer/Counter Register
  TCNT2 &= 0x00;
  
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
  // For 8kHz, TCNTx needs - 
  //   - 250 counts at 8 prescaler
  //   - 31.25 counts at 64 prescaler
  
  // Turn on CTC mode
  TCCR2A |= (0x01 << WGM21);
  // Set Prescaler to 8
  TCCR2B |= (0x01 << CS22);
  TCCR2B |= (0x01 << CS20);
  // Set compare match register (OCR2A) to 249
  OCR2A = 0xF9;
  // Enable interrupt upon compare match of OCR2A
  TIMSK2 |= (0x01 << OCIE2A);

  // allow interrupts
  sei();
}

ISR(TIMER2_COMPA_vect)
{
  motor_controller.spinMotor();
}