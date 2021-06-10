#include <AngularVelocityCalculator.h>

#define SERIAL_PRINT_TIME_PERIOD 1000

angularVelocityCalculator WheelShaft(2, 3, 8000, 30);

long int lastTime;

void setup()
{
    Serial.begin(9600);
    lastTime = millis();
}

void loop()
{
    if (millis() - lastTime > SERIAL_PRINT_TIME_PERIOD)
    {
        Serial.println(WheelShaft.getAngularVelocity());
    }
}

ISR(TIMER2_COMPA_vect)
{
    LeftWheelShaft.updateAngularVelocity();
}