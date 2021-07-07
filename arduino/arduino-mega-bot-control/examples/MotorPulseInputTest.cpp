/**
 * @file main.cpp
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in)
 * @brief This program records the pulse response of a Motor
 * 
 * It uses the AngularVelocityCalculator class to calculate the
 * angular velocity of the rotating motor shaft. And uses the PhaseCorrect16BitPWM class
 * to provide input for the motor controller
 * 
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Required for the AngularVelocityCalculator class
#include "AngularVelocityCalculator.h"

// Required for the PhaseCorrect16BitPWM class
#include "PhaseCorrect16BitPWM.h"

/**
 * @brief Maximum value of the pulse PWM signal
 */
#define MAX_VALUE 0xFFFF

/**
 * @brief Angular velocity calculator object
 * 
 * Encoder pins A-18, B-20
 * Update Frequency - 50 Hz
 * Counts per rotation - 840
 */
AngularVelocityCalculator ang_vel_calc(18, 20, 50, 840);

/**
 * @brief Object for Phase Correct 16 bit PWM
 */
Timer1PhaseCorrectPWM pwm_16;

/**
 * @brief Array for input PWM signal
 */
float PWM_signal[250];

/**
 * @brief Array for output angular velocity
 */
float ang_vel_array[250];

/**
 * @brief Iterator
 */
int i = 0;

void initializeTimer3();

void setup()
{
    // Initialize serial comm
    Serial.begin(115200);

    // Setting the pulse PWM signal
    for (int j=0; j<250; j+=1)
    {
        if (j<100 || j>150)
        {
            PWM_signal[j] = 0;
        }
        else
        {
            PWM_signal[j] = MAX_VALUE;
        }
    }

    pwm_16.setupChannelB();

    initializeTimer3();

    Serial.println("Arduino initialized successfully");
}

void loop()
{
    if (i==250)
    {
        for (int j = 0; j<250; j++)
        {
            Serial.print(j*20);
            Serial.print("\t");
            Serial.print(PWM_signal[j]);
            Serial.print("\t");
            Serial.println(ang_vel_array[j]);
        }
        i++;
    }
}

void initializeTimer3()
{
    // stop interrupts
    cli();

    // Clear Timer/Counter Control Resgisters
    TCCR3A &= 0x00;
    TCCR3B &= 0x00;
    TCCR3C &= 0x00;
    // Clear Timer/Counter Register
    TCNT3 &= 0x0000;
    
    // Set Timer1 to interrupt at 50Hz

    // ATmega2560 clock frequency - 16MHz
    // Prescalers available for timers - 1, 8, 64, 256, 1024
    // Timer clock frequency = ATmega2560 clock frequency / Prescaler
    
    // To have interrupts with better frequency resolution timer
    // will be operated in Clear Timer on Compare Match (CTC) mode
    // TCNTx will count from 0x00 to the value in OCRnA register
    // Frequency of Interrupt = Timer clock frequency / Number of TCNTn Counts before it resets to 0x00

    // TCNTx will be compared to OCRnx in each clock cycle
    // Upon compare match, interrupt is fired
    // For 50Hz, TCNTx needs - 
    //   - 40000 counts at 8 prescaler
    //   - 5000 counts at 64 prescaler
    //   - 1250 counts at 256 prescaler
    //   - 312.5 counts at 1024 prescaler
    
    // Turn on CTC mode
    TCCR3B |= (0x01 << WGM32);
    // Set Prescaler to 256
    TCCR3B |= (0x01 << CS32);
    // Set compare match register (OCR3A) to 249
    OCR3A = 0x04E1;
    // Enable interrupt upon compare match of OCR3A
    TIMSK3 |= (0x01 << OCIE3A);

    // allow interrupts
    sei();
}

ISR(TIMER3_COMPA_vect)
{
    if (i<250)
    {
        ang_vel_calc.updateAngularVelocity();
        ang_vel_calc.getAngularVelocity(ang_vel_array[i]);
        pwm_16.setDutyCyclePWMChannelB(PWM_signal[i]);
        i++;
    }
}