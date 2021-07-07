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

/**
 * @brief Object for 16 bit PWM
 * 
 */
Timer1PhaseCorrectPWM pwm_16bit;

void setup() {
   
    Serial.begin(115200);

    pwm_16bit.setupChannelC();
}

void loop() {

    for (int i = 0; i < 0x10000; i+=50)
    {
        pwm_16bit.setDutyCyclePWMChannelC(i);
        delay(10);
    }
}