#include "AngularVelocityCalculator.h"
// #include "AngularVelocityCalculator.cpp"

#define SERIAL_PRINT_TIME_PERIOD 1000

angularVelocityCalculator encoderShaft(2, 3, 8000, 30);

long int lastTime;

void initializeTimer2();

void setup()
{
  Serial.begin(9600);
  lastTime = millis();

  initializeTimer2();
  
  Serial.println("Arduino Initialized successfully");
}

void loop()
{
  if (millis() - lastTime > SERIAL_PRINT_TIME_PERIOD)
  {
    Serial.println(encoderShaft.getAngularVelocity());
    // Serial.println(encoderShaft.read());
    lastTime = millis();
  }
}

void initializeTimer2()
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

  //allow interrupts
  sei();
}

ISR(TIMER2_COMPA_vect)
{
  encoderShaft.updateAngularVelocity();
}