/*
 * BME280.h
 *
 *  Created on: Sep 6, 2025
 *      Author: Omar Elsaghir
 */

#ifndef BME280_H_
#define BME280_H_

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

// Structures to hold calibration data
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_CalibData;

BME280_CalibData calib;

void readBME280Raw(int32_t *temp_raw, int32_t *press_raw, int32_t *hum_raw);
void initBME280(void);
void readBME280Calibration(void);
float compensateTemperature(int32_t adc_T);
float compensatePressure(int32_t adc_P);
float compensateHumidity(int32_t adc_H);
//void bme280_selftest(void);

#endif /* BME280_H_ */
