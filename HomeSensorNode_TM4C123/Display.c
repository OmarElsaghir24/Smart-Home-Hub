/*
 * Display.c
 *
 *  Created on: Nov 4, 2025
 *      Author: Omar Elsaghir
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "Display.h"
#include "tm4c123gh6pm.h"
#include "OLED.h"
#include "icons.h"
#include "user_created_tasks.h"

// State Defines
#define TEMPERATURE     0
#define HUMIDITY        1
#define PRESSURE        2
#define BRIGHTNESS      3

// Variables
uint8_t state;
uint8_t count = 0;
char buff[100];

void display_sequence(Sensor_Data data)
{
    switch(state)
    {
        case TEMPERATURE:
            ssd1306_drawBitmapXy(16, 16, thermometer_icon, 16, 16, BMP_ROW_MSB_FIRST);
            snprintf(buff, sizeof(buff), "%.2f", data.temperature);
            ssd1306_printAtSize(2, 32, buff, 2);
            ssd1306_printAtSize(2, 96, "C", 1);

            ssd1306_setCursor(5, 0);
            snprintf(buff, sizeof(buff), " Pressure:%.2f hPa", data.pressure);
            ssd1306_print(buff);

            ssd1306_setCursor(6, 0);
            snprintf(buff, sizeof(buff), " Humidity: %.2f %%", data.humidity);
            ssd1306_print(buff);
            if(count == 5)
            {
                state = HUMIDITY;
                count = 0;
                ssd1306_clearDisplay();
            }
            break;
        case HUMIDITY:
            ssd1306_drawBitmapXy(16, 16, humidity_icon, 16, 16, BMP_ROW_MSB_FIRST);
            snprintf(buff, sizeof(buff), "%.2f", data.humidity);
            ssd1306_printAtSize(2, 32, buff, 2);
            ssd1306_printAtSize(2, 96, "%", 1);

            ssd1306_setCursor(5, 0);
            snprintf(buff, sizeof(buff), " Temperature:%.2f C", data.temperature);
            ssd1306_print(buff);

            ssd1306_setCursor(6, 0);
            snprintf(buff, sizeof(buff), " Pressure:%.2f hPa", data.pressure);
            ssd1306_print(buff);

            if(count == 5)
            {
                state = PRESSURE;
                count = 0;
                ssd1306_clearDisplay();
            }
            break;
        case PRESSURE:
            ssd1306_drawBitmapXy(8, 16, pressure_icon, 16, 16, BMP_ROW_MSB_FIRST);
            snprintf(buff, sizeof(buff), "%.2f", data.pressure);
            ssd1306_printAtSize(2, 24, buff, 2);
            ssd1306_printAtSize(2, 108, "hPa", 1);

            ssd1306_setCursor(5, 0);
            snprintf(buff, sizeof(buff), " Temperature:%.2f C", data.temperature);
            ssd1306_print(buff);

            ssd1306_setCursor(6, 0);
            snprintf(buff, sizeof(buff), " Brightness: %d %%", data.brightness);
            ssd1306_print(buff);
            if(count == 5)
            {
                state = BRIGHTNESS;
                count = 0;
                ssd1306_clearDisplay();
            }
            break;
        case BRIGHTNESS:
            ssd1306_drawBitmapXy(32, 8, sun_icon, 16, 16, BMP_ROW_MSB_FIRST);
            snprintf(buff, sizeof(buff), "%d %%", data.brightness);
            ssd1306_printAtSize(1, 56, buff, 2);

            ssd1306_setCursor(5, 0);
            snprintf(buff, sizeof(buff), " Temperature: %.2f C", data.temperature);
            ssd1306_print(buff);

            ssd1306_setCursor(6, 0);
            snprintf(buff, sizeof(buff), " Humidity: %.2f %%", data.humidity);
            ssd1306_print(buff);
            if(count == 5)
            {
                state = TEMPERATURE;
                count = 0;
                ssd1306_clearDisplay();
            }
            break;

    }

    count++;

}


