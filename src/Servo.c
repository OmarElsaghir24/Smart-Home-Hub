/*
 * Servo.c
 *
 *  Created on: Oct 1, 2025
 *      Author: Omar Elsaghir
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include "Servo.h"

#define SERVO_MASK 32   // PC5 = bit 5
#define SERVO_NEUTRAL_US 1500

void initServo(void)
{
    // Enable PWM0 and Port C
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure PC5 as M0PWM7
    GPIO_PORTC_AFSEL_R |= SERVO_MASK;
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC5_M;
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_M0PWM7;
    GPIO_PORTC_DEN_R   |= SERVO_MASK;
    GPIO_PORTC_AMSEL_R &= ~SERVO_MASK;

    // Reset PWM0
    SYSCTL_SRPWM_R |= SYSCTL_SRPWM_R0;
    SYSCTL_SRPWM_R &= ~SYSCTL_SRPWM_R0;

    // Disable generator 3 during setup
    PWM0_3_CTL_R = 0;

    // Configure Generator 3, output B (PWM7) actions:
    PWM0_3_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;

    // ======== Period and pulse width calculations at 40 MHz =========
    // 50 Hz = 20 ms period → ticks = 40 MHz * 0.02 = 800,000
    PWM0_3_LOAD_R = 800000 - 1;

    // Neutral (1.5 ms pulse) → ticks = 40 MHz * 0.0015 = 60,000
    PWM0_3_CMPB_R = 800000 - 60000;

    // Enable generator 3
    PWM0_3_CTL_R = PWM_0_CTL_ENABLE;

    // Enable PWM7 output
    PWM0_ENABLE_R |= PWM_ENABLE_PWM7EN;
}

// Set servo angle between 0° and 180°
void setServoAngle(float angle)
{
    // Convert angle to pulse width in microseconds (1000–2000 µs)
    float pulse_us = 1000.0 + (angle / 180.0f) * 1000.0;

    // Convert microseconds to ticks at 40 MHz
    int ticks = (int)(pulse_us * 40);

    // Update compare register (inverted because count-down mode)
    PWM0_3_CMPB_R = 800000 - ticks;
}
