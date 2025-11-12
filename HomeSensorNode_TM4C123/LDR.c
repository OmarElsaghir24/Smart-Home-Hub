/*
 * LDR.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Omar Elsaghir
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "gpio.h"
#include "adc0.h"
#include "uart0.h"

#define LDR PORTE, 0
char str[100];

void initLDR(void)
{

    enablePort(PORTE);

    // Set the Light Dependent Resistor as an input to AIN0
    selectPinAnalogInput(LDR);

    // Initializes analog to digital converter
    initAdc0Ss3();
    // Use AIN0 input with N=4 hardware sampling
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(2);
}

