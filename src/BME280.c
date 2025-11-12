/*
 * BME280.c
 *
 *  Created on: Sep 6, 2025
 *      Author: Omar Elsaghir
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "i2c0.h"
#include "uart0.h"
#include "BME280.h"

// Defines
#define BME280_ADDRESS 0x76

// Global variables
int32_t t_fine;
char str[100];

void readBME280Raw(int32_t *temp_raw, int32_t *press_raw, int32_t *hum_raw)
{
    uint8_t data[8];
    // Pressure MSB(0xF7), Pressure LSB(0xF8), Pressure XLSB(0xF9)
    // Temp MSB(0xFA), Temp LSB(0xFB), Temp XLSB(0xFC)
    // Humidity MSB(0xFD), Humidity LSB(0xFE)
    readI2c0Registers(BME280_ADDRESS, 0xF7, data, 8);

    *press_raw = (int32_t)((((uint32_t)data[0]) << 12) | (((uint32_t)data[1]) << 4) | (data[2] >> 4));
    *temp_raw  = (int32_t)((((uint32_t)data[3]) << 12) | (((uint32_t)data[4]) << 4) | (data[5] >> 4));
    *hum_raw   = (int32_t)(((uint32_t)data[6] << 8) | (uint32_t)data[7]);
}

void initBME280(void)
{

    // Initialize I2C
    initI2c0();

    // Step 1: Reset the device
   writeI2c0Register(BME280_ADDRESS, 0xE0, 0xB6);  // reset command
   _delay_cycles(800000); // wait ~2-5 ms (depends on your clock)

   // Step 2: Configure humidity oversampling (ctrl_hum, reg 0xF2)
   writeI2c0Register(BME280_ADDRESS, 0xF2, 0x01);  // oversampling x1

   // Step 3: Configure temperature/pressure oversampling and mode (ctrl_meas, reg 0xF4)
   // bits: [7:5] temp oversampling, [4:2] pressure oversampling, [1:0] mode
   // Example: temp x1, press x1, mode = normal (11)
   writeI2c0Register(BME280_ADDRESS, 0xF4, 0x27);  // 001 001 11b = 0x27

   // Step 4: Configure config register (filter and standby, reg 0xF5)
   // Example: standby 1000 ms, filter off
   writeI2c0Register(BME280_ADDRESS, 0xF5, 0xA0);  // 1010 0000b

   // At this point the BME280 is running in normal mode
}

void readBME280Calibration(void)
{
    uint8_t data[26];

    // Read temp + pressure calibration
    readI2c0Registers(BME280_ADDRESS, 0x88, data, 26);
    calib.dig_T1 = (data[1] << 8) | data[0];
    calib.dig_T2 = (data[3] << 8) | data[2];
    calib.dig_T3 = (data[5] << 8) | data[4];
    calib.dig_P1 = (data[7] << 8) | data[6];
    calib.dig_P2 = (data[9] << 8) | data[8];
    calib.dig_P3 = (data[11] << 8) | data[10];
    calib.dig_P4 = (data[13] << 8) | data[12];
    calib.dig_P5 = (data[15] << 8) | data[14];
    calib.dig_P6 = (data[17] << 8) | data[16];
    calib.dig_P7 = (data[19] << 8) | data[18];
    calib.dig_P8 = (data[21] << 8) | data[20];
    calib.dig_P9 = (data[23] << 8) | data[22];
    calib.dig_H1 = data[25];

    // Read humidity calibration
    uint8_t data2[7];
    readI2c0Registers(BME280_ADDRESS, 0xE1, data2, 7);
    calib.dig_H2 = (data2[1] << 8) | data2[0];
    calib.dig_H3 = data2[2];
    calib.dig_H4 = (data2[3] << 4) | (data2[4] & 0x0F);
    calib.dig_H5 = (data2[5] << 4) | (data2[4] >> 4);
    calib.dig_H6 = (int8_t)data2[6];
}

float compensateTemperature(int32_t adc_T)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
             ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
             ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0; // Celsius
}

float compensatePressure(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) +
           ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (var1 == 0) return 0; // avoid divide by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    return (float)p / 256.0; // Pa
}

float compensateHumidity(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) -
                    (((int32_t)calib.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                    (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                    (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + 32768)) >> 10) + 2097152) *
                    ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                    ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    float H = (v_x1_u32r >> 12);
    return H / 1024.0; // %RH
}

