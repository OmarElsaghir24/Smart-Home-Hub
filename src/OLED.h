/*
 * OLED.h
 *
 *  Created on: Aug 26, 2025
 *      Author: Omar Elsaghir
 */

#ifndef OLED_H_
#define OLED_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Supported bitmap encodings
typedef enum {
    BMP_SSD1306_PAGE = 0,   // each byte = 8 vertical pixels; page-major (what SSD1306 RAM uses)
    BMP_ROW_MSB_FIRST = 1,  // row-major; in each byte, bit7=leftmost pixel (image2cpp default)
    BMP_ROW_LSB_FIRST = 2   // row-major; in each byte, bit0=leftmost pixel
} BitmapFormat;

void ssd1306_command(uint8_t cmd);
char* convertIntToString(uint32_t num, char str[]);
int32_t convertStringToInt(char str[]);
void init_ssd1306(void);
void ssd1306_setCursor(uint8_t page, uint8_t column);
void ssd1306_data(uint8_t *data, uint8_t size);
void ssd1306_drawChar(char c);
void ssd1306_print(const char *str);
void ssd1306_clearDisplay();
void ssd1306_drawBitmap(const uint8_t *bitmap, uint8_t width, uint8_t height);
void ssd1306_displayBitmap(const uint8_t *bitmap);
void ssd1306_drawBitmapAt(const uint8_t *bitmap, uint8_t x, uint8_t y, uint8_t width, uint8_t height);
void ssd1306_printAt(uint8_t page, uint8_t column, const char *str);
void ssd1306_drawCharScaledAt(uint8_t page, uint8_t col, char c, uint8_t size);
void ssd1306_printAtSize(uint8_t page, uint8_t col, const char *str, uint8_t size);
void ssd1306_drawBitmapXY(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height);
static inline uint8_t getPixel(const uint8_t *bmp, uint16_t width, uint16_t height, uint16_t sx, uint16_t sy, BitmapFormat fmt);
void ssd1306_drawBitmapXy(uint8_t x, uint8_t y,const uint8_t *bitmap, uint8_t width, uint8_t height, BitmapFormat fmt);

#endif /* OLED_H_ */
