/* 
  This is an example code to estimate angular velocity 
  of a rotating shaft with an encoder attached to it,
  using the AngularVelocityCalculator class.

  The angular velocity is updated at frequency of 8 kHz,
  using timer 2 interrupt. It is printed to serial monitor
  at a much slower rate of 1Hz.
*/

#include "AngularVelocityCalculator.h"

// Time interval after which angular velocity is
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 1000

AngularVelocityCalculator encoder_shaft(2, 4, 8E3, 840);

long int last_time;

float duration_sum = 0;
float last_update_time = 0;
float current_time = 0;
float num = 0;

void initialize_timer_2();

void setup()
{
  Serial.begin(9600);
  last_time = millis();

  initialize_timer_2();
  
  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  if (millis() - last_time > SERIAL_PRINT_TIME_PERIOD)
  {
    Serial.print("Velocity:\t");
    Serial.println(encoder_shaft.getAngularVelocity(), 5);
    
    Serial.print("Encoder shaft position:\t");
    Serial.println(encoder_shaft.read());

    Serial.print("Average Velocity Update Period:\t");
    Serial.println(duration_sum / num);

    last_time = millis();
  }
}

void initialize_timer_2()
{
  // stop interrupts
  cli();

  // set timer2 interrupt at 8kHz
  
  TCCR2A = 0; // set entire TCCR2A register to 0
  TCCR2B = 0; // same for TCCR2B
  TCNT2 = 0; // initialize counter value to 0
  
  // set compare match register for 8khz increments
  OCR2A = 249; // = (16*10^6) / (8000*8) - 1 (must be <256)
  
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS22); 
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);

  // allow interrupts
  sei();
}

ISR(TIMER2_COMPA_vect)
{
  current_time = micros();
  duration_sum += current_time - last_update_time;
  last_update_time = current_time;
  num += 1;

  encoder_shaft.updateAngularVelocity();
}