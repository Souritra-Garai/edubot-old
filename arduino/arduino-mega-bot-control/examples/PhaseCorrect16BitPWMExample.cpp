/**
 * @file PWM16bitPhaseCorrectExample.cpp
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in)
 * @brief Example for setting PWM over two channels 
 * with the help of a single library.
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "PhaseCorrect16BitPWM.h"

void setup() {
   
    Serial.begin(115200);

    Timer1PhaseCorrectPWM::clearTimerSettings();
    Timer1PhaseCorrectPWM::setupTimer();
    Timer1PhaseCorrectPWM::setupChannelA();
}

void loop() {

    for (int i = 0; i <= MAX_PWM_DUTY_CYCLE_INPUT; i+=50)
    {
        Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA(i);
        delay(10);

        Serial.print("Current PWM Input: ");
        Serial.println(i);
    }
}