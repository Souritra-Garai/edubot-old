/**
 * @file PhaseCorrect16bitPWM.h
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in)
 * @brief This library defines a class to set 16 bit PWM values
 * into two channels of Timer1 - A and B.
 * @version 0.1
 * @date 2021-07-04
 * 
 * @copyright Copyright (c) 2021
 * 
 * 16 bit PWM. Phase and frequency correct.
 */

#ifndef __PHASE_CORRECT_16_BIT_PWM__
#define __PHASE_CORRECT_16_BIT_PWM__

#include "Arduino.h"

/**
 * @brief Class to set PWM values with its own analogWrite() 
 * function for 16 bit.
 * 
 */
class PhaseCorrectPWM
{
    public:

        // Update PWM values in channel A
        inline void setPWMChannelA(uint16_t dutyCycle) __attribute__((always_inline));
        // Update PWM values in channel B
        inline void setPWMChannelB(uint16_t dutyCycle) __attribute__((always_inline));
        
        PhaseCorrectPWM();

};

/**
 * @brief Set value to OCR1A register 
 * 
 * @param val // PWM value to be written
 */
void PhaseCorrectPWM::setPWMChannelA(uint16_t val)
{
    OCR1A = val;
}

/**
 * @brief Set value to OCR1B register
 * 
 * @param val // PWM value to be written
 */
void PhaseCorrectPWM::setPWMChannelB(uint16_t val)
{
    OCR1B = val;
}

/**
 * @brief Construct a new PhaseCorrectPWM:: PhaseCorrectPWM object
 * 
 */
PhaseCorrectPWM::PhaseCorrectPWM()
{
    // Clear OC1A and OC1B on compare match while up-counting and
    // Set OC1A and OC1B on compare match while down-counting
    TCCR1A |= 0x01 << COM1A1;
    TCCR1A |= 0x01 << COM1B1;
    
    // No Prescaler implies 16MHz PWM frequency
    TCCR1B |= 0x01 << CS10;

    // Using Phase Correct PWM with ICRn as TOP
    TCCR1B |= 0x01 << WGM13; // Mode 8

    // TOP value
    ICR1 = 0xFFFF;
}

#endif