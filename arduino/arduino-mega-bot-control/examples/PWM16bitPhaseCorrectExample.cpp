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

#include "PhaseCorrect16bitPWM.h"

/**
 * @brief Object for 16 bit PWM
 * 
 */
PhaseCorrectPWM pwm_16bit;

void setup() {
    /**
     * @brief Initialize serial comm
     * 
     */
    Serial.begin(9600);

    /**
     * @brief Pin 12 is set as output pin
     * 
     */
    pinMode(12, OUTPUT);
}
 
void loop() {

    /**
     * @brief Set OCR1B to 100
     * 
     * Argument is taken as uint16_t
     * Corresponds to analogWrite()
     */
    pwm_16bit.setPWMChannelB(100);

    Serial.println("Loop terminated");
}