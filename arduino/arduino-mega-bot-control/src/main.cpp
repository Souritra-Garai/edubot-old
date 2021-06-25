/* 
  This is an example code to estimate angular velocity 
  of a rotating shaft with an encoder attached to it,
  using the AngularVelocityCalculator class.

  The angular velocity is updated at frequency of 8 kHz,
  using timer 2 interrupt. It is printed to serial monitor
  at a much slower rate of 1Hz.
*/

#include "AngularVelocityCalculator.h"

// Time interval after which anything is
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 2000

#define ENCODER_PIN_A 19
#define ENCODER_PIN_B 21

#define VELOCITY_UPDATE_FREQUENCY 1E3 // Hz

#define ENCODER_COUNTS_PER_ROTATION 840

// Our object to estimate the angular velocity of encoder shaft
AngularVelocityCalculator encoder_shaft(
  ENCODER_PIN_A,
  ENCODER_PIN_B,
  VELOCITY_UPDATE_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);

// To store the last time anything was
// printed through serial port in millisecond
long int last_serial_print_time;

// To verify the time period of velocity update
// Variable to store the sum of durations in microseconds
// after which velocity is updated
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

// Declaration of function for initializing Timer 2
// interrupts
void initialize_timer_2();

void setup()
{
  // Initialize Serial Comm
  Serial.begin(115200);

  // Initialize global variables

  last_serial_print_time = millis();

  velocity_update_duration_sum = 0;
  current_velocity_update_time = 0;
  last_velocity_update_time = 0;
  number_velocity_updates = 0;

  // Initialize timer 2 for interrupts
  initialize_timer_2();
  
  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  if (millis() - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
  {
    float angular_velocity;
    encoder_shaft.getAngularVelocity(angular_velocity);
    Serial.print("Velocity:\t");
    Serial.println(angular_velocity, 5);
    
    Serial.print("Encoder shaft position:\t");
    Serial.println(encoder_shaft.read());

    Serial.print("Average Velocity Update Period:\t");
    Serial.println(velocity_update_duration_sum / number_velocity_updates);

    last_serial_print_time = millis();
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
  // Set compare match register (OCR2A) to 249
  OCR2A = 0xF9;
  // Enable interrupt upon compare match of OCR2A
  TIMSK2 |= (0x01 << OCIE2A);

  // allow interrupts
  sei();
}

ISR(TIMER2_COMPA_vect)
{
  current_velocity_update_time = micros();
  encoder_shaft.updateAngularVelocity();
  velocity_update_duration_sum += current_velocity_update_time - last_velocity_update_time;
  last_velocity_update_time = current_velocity_update_time;
  number_velocity_updates += 1;
}