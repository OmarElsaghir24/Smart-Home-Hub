/*
 * OLED.c
 *
 *  Created on: Aug 26, 2025
 *      Author: Omar Elsaghir
 */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "i2c2.h"
#include "OLED.h"

#define OLED_ADDRESS 0x3C // SSD1306 I2C address (might also be 0x3D)

// 96 character 5x7 bitmaps based on ISO-646 (BCT IRV extensions)
const uint8_t charGen[100][5] = {
    // Codes 32-127
    // Space ! " # $ % & ' ( ) * + , - . /
    {0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x4F, 0x00, 0x00},
    {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},
    {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x55, 0x22, 0x40},
    {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00},
    {0x14, 0x08, 0x3E, 0x08, 0x14},
    {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00},
    {0x08, 0x08, 0x08, 0x08, 0x08},
    {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02},
    // 0-9
    {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39},
    {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E},
    // : ; < = > ? @
    {0x00, 0x36, 0x36, 0x00, 0x00},
    {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00},
    {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08},
    {0x02, 0x01, 0x51, 0x09, 0x3E},
    {0x32, 0x49, 0x79, 0x41, 0x3E},
    // A-Z
    {0x7E, 0x11, 0x11, 0x11, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36},
    {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C},
    {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x49, 0x49, 0x3A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F},
    {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01},
    {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x04, 0x08, 0x10, 0x7F},
    {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06},
    {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01},
    {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F},
    {0x3F, 0x40, 0x70, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x07, 0x08, 0x70, 0x08, 0x07},
    {0x61, 0x51, 0x49, 0x45, 0x43},
    // [ \ ] ^ _ `
    {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20},
    {0x00, 0x41, 0x41, 0x7F, 0x00},
    {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00},
    // a-z
    {0x20, 0x54, 0x54, 0x54, 0x78},
    {0x7F, 0x44, 0x44, 0x44, 0x38},
    {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18},
    {0x08, 0x7E, 0x09, 0x01, 0x02},
    {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78},
    {0x00, 0x44, 0x7D, 0x40, 0x00},
    {0x20, 0x40, 0x44, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00},
    {0x7C, 0x04, 0x18, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78},
    {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08},
    {0x08, 0x14, 0x14, 0x18, 0x7C},
    {0x7C, 0x08, 0x04, 0x04, 0x08},
    {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20},
    {0x3C, 0x40, 0x40, 0x20, 0x7C},
    {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x20, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44},
    {0x0C, 0x50, 0x50, 0x50, 0x3C},
    {0x44, 0x64, 0x54, 0x4C, 0x44},
    // { | } ~ cc
    {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00},
    {0x0C, 0x04, 0x1C, 0x10, 0x18},
    {0x00, 0x00, 0x00, 0x00, 0x00},
    // Custom assignments beyond ISO646
    // Codes 128+: right arrow, left arrow, degree sign
    {0x08, 0x08, 0x2A, 0x1C, 0x08},
    {0x08, 0x1C, 0x2A, 0x08, 0x08},
    {0x07, 0x05, 0x07, 0x00, 0x00},
};

void ssd1306_command(uint8_t cmd)
{
    writeI2c2Registers(OLED_ADDRESS, 0x00, &cmd, 1); // 0x00 = command
}

char* convertIntToString(uint32_t num, char str[])
{
    uint8_t len = 0, i = 0;
    uint64_t mod = 10;

    while ((num % mod) != num)
    {
        mod *= 10;
        len++;
    }

    for (i = 0; i <= len; i++)
        str[i] = (num % mod / (mod /= 10) + 48);

    str[len + 1] = NULL;

    return str;
}

int32_t convertStringToInt(char str[])
{
    int8_t len = 0, i = 0;
    uint32_t num = 0, tens = 1;

    while (*str != NULL)
    {
        len++;
        str++;
    }

    str -= len;
    tens = 10 * (len - 1);

    for (i = 0; i < len; i++)
    {
        num += (str[i] - 48) * tens;
        tens = tens / 10;
    }
    return num;
}


void init_ssd1306(void)
{
    int i;
    const uint8_t init_sequence[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Set display clock divide
        0xA8, 0x3F, // Set multiplex (height-1)
        0xD3, 0x00, // Set display offset
        0x40,       // Set start line
        0x8D, 0x14, // Charge pump
        0x20, 0x00, // Memory mode (horizontal addressing)
        0xA1,       // Segment remap
        0xC8,       // COM output scan direction
        0xDA, 0x12, // COM pins config
        0x81, 0xCF, // Contrast
        0xD9, 0xF1, // Pre-charge
        0xDB, 0x40, // VCOM detect
        0xA4,       // Resume display from RAM
        0xA6,       // Normal display (not inverted)
        0xAF        // Display ON
    };
    for (i = 0; i < sizeof(init_sequence); i++) {
        ssd1306_command(init_sequence[i]);
    }
}

uint8_t cursorPage = 0;
uint8_t cursorCol = 0;

void ssd1306_setCursor(uint8_t page, uint8_t column)
{
    cursorPage = page;
    cursorCol = column;
    ssd1306_command(0xB0 | (page & 0x07));
    ssd1306_command(0x00 | (column & 0x0F));
    ssd1306_command(0x10 | (column >> 4));
}

void ssd1306_data(uint8_t *data, uint8_t size)
{
    writeI2c2Registers(OLED_ADDRESS, 0x40, data, size); // 0x40 = data
}

void ssd1306_drawChar(char c)
{
    if (c < 32 || c > 126) c = ' ';
    ssd1306_data((uint8_t*)charGen[c - 32], 5);
    uint8_t space = 0x00;
    ssd1306_data(&space, 1); // 1-pixel spacing
}

void ssd1306_print(const char *str)
{
    while (*str) {
        ssd1306_drawChar(*str++);
        cursorCol += 6; // Each character is 5px + 1 space

        if (cursorCol > 120) { // Auto-wrap
            cursorCol = 0;
            cursorPage++;
            if (cursorPage > 7) cursorPage = 0;
            ssd1306_setCursor(cursorPage, cursorCol);
        }
    }
}

void ssd1306_clearDisplay()
{
    uint8_t page, col;
    for (page = 0; page < 8; page++) {
        ssd1306_setCursor(page, 0); // Move to start of page
        for (col = 0; col < 128; col++) {
            uint8_t zero = 0x00;
            ssd1306_data(&zero, 1); // Write blank byte
        }
    }
    ssd1306_setCursor(0, 0); // Reset cursor to top left
}

void ssd1306_drawBitmap(const uint8_t *bitmap, uint8_t width, uint8_t height)
{
    uint8_t page;
    uint8_t pages = height / 8;

    for (page = 0; page < pages; page++) {
        ssd1306_setCursor(page, 0);
        const uint8_t *page_data = bitmap + (page * width);
        ssd1306_data((uint8_t *)page_data, width);
    }
}

void ssd1306_displayBitmap(const uint8_t *bitmap)
{
    uint8_t page;
    for (page = 0; page < 8; page++) {
        ssd1306_setCursor(page, 0);
        ssd1306_data((uint8_t*)(bitmap + page * 128), 128);
    }
}

void ssd1306_drawBitmapAt(const uint8_t *bitmap, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint8_t page, pages = height / 8;

    for (page = 0; page < pages; page++) {
        ssd1306_setCursor((y / 8) + page, x);  // Set start position
        const uint8_t *page_data = bitmap + (page * width);
        ssd1306_data((uint8_t *)page_data, width);
    }
}

void ssd1306_printAt(uint8_t page, uint8_t column, const char *str)
{
    ssd1306_setCursor(page, column);  // Set starting position
    cursorPage = page;
    cursorCol = column;

    while (*str) {
        ssd1306_drawChar(*str++);
        cursorCol += 6; // 5px font + 1px space

        if (cursorCol > 120) { // Auto-wrap if needed
            cursorCol = 0;
            cursorPage++;
            if (cursorPage > 7) cursorPage = 0;
            ssd1306_setCursor(cursorPage, cursorCol);
        }
    }
}

// Draw a single scaled character at (page, col)
void ssd1306_drawCharScaledAt(uint8_t page, uint8_t col, char c, uint8_t size)
{
    uint8_t p, i, sx, y;
    if (size == 0) size = 1;
    if (c < 32 || c > 126) c = ' ';

    const uint8_t *glyph = (const uint8_t*)charGen[c - 32]; // 5 bytes, 8 bits tall
    uint8_t pages_used = (uint8_t)((8 * size + 7) / 8);     // how many 8px bands tall

    // For each 8-pixel-high page band of the scaled char
    for (p = 0; p < pages_used; p++)
    {
        uint8_t target_page = page + p;
        if (target_page > 7) break; // clip if it would run off the display

        // Start at the correct page/column for this band
        ssd1306_setCursor(target_page, col);

        // For each of the 5 source columns
        for (i = 0; i < 5; i++)
        {
            uint8_t src_col = glyph[i];

            // Horizontal scaling: write this source column 'size' times
            for (sx = 0; sx < size; sx++)
            {
                uint8_t outByte = 0;

                // Build the 8 bits that live in this page band
                for (y = 0; y < 8; y++)
                {
                    // y_total is the scaled y within the whole character
                    uint16_t y_total = (uint16_t)p * 8 + y;
                    // Map back to the original (unscaled) bit row 0..7
                    uint8_t src_bit = (uint8_t)(y_total / size);

                    if (src_bit < 8 && (src_col & (1 << src_bit)))
                        outByte |= (1 << y);
                }

                ssd1306_data(&outByte, 1);
            }
        }

        // Add spacing column(s), scaled horizontally
        for (sx = 0; sx < size; sx++)
        {
            uint8_t zero = 0x00;
            ssd1306_data(&zero, 1);
        }
    }
}

// Print a string at (page, col) with integer scale 'size'
void ssd1306_printAtSize(uint8_t page, uint8_t col, const char *str, uint8_t size)
{
    if (size == 0) size = 1;
    uint8_t char_w = (uint8_t)(6 * size);                  // 5px + 1px space, scaled
    uint8_t char_pages = (uint8_t)((8 * size + 7) / 8);    // vertical pages per char

    while (*str)
    {
        // Simple wrap to next band if it would exceed width
        if (col + char_w > 128)
        {
            col = 0;
            page += char_pages;
            if (page > 7) break; // no more room
        }

        ssd1306_drawCharScaledAt(page, col, *str, size);
        col += char_w;
        str++;
    }
}

// Draw any monochrome bitmap at (x, y)
void ssd1306_drawBitmapXY(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t width, uint8_t height)
{
    uint8_t page, col;
    uint8_t pageStart = y / 8;    // Starting page
    uint8_t bitOffset = y % 8;    // Pixel offset inside page
    uint8_t pages = (height + 7) / 8;  // How many pages bitmap spans

    for (page = 0; page < pages; page++)
    {
        ssd1306_setCursor(pageStart + page, x);

        for (col = 0; col < width; col++)
        {
            // Take one byte (8 vertical pixels) from the bitmap
            uint8_t data = bitmap[page * width + col];

            // If bitmap not aligned with page, shift it
            uint8_t shifted = data << bitOffset;

            ssd1306_data(&shifted, 1);
        }
    }
}

// Get 0/1 pixel from source bitmap at (sx, sy) for the given format
static inline uint8_t getPixel(
    const uint8_t *bmp, uint16_t width, uint16_t height,
    uint16_t sx, uint16_t sy, BitmapFormat fmt)
{
    if (sx >= width || sy >= height) return 0;

    if (fmt == BMP_SSD1306_PAGE) {
        // page-major: first 'width' bytes = rows y 0..7, next 'width' bytes = y 8..15, etc.
        uint16_t sp = sy >> 3;         // source page index
        uint8_t  bit = 1u << (sy & 7); // bit inside that page byte
        uint16_t idx = sp * width + sx;
        return (bmp[idx] & bit) ? 1 : 0;
    } else {
        // row-major: bytes pack 8 horizontal pixels per row
        uint16_t stride = (width + 7) >> 3;   // bytes per row
        uint16_t byteIndex = sy * stride + (sx >> 3);
        uint8_t  inByte    = bmp[byteIndex];
        if (fmt == BMP_ROW_MSB_FIRST) {
            uint8_t mask = 0x80u >> (sx & 7);
            return (inByte & mask) ? 1 : 0;
        } else { // BMP_ROW_LSB_FIRST
            uint8_t mask = 1u << (sx & 7);
            return (inByte & mask) ? 1 : 0;
        }
    }
}

// Universal draw: renders any (width x height) bitmap at top-left (x,y)
void ssd1306_drawBitmapXy(uint8_t x, uint8_t y,const uint8_t *bitmap, uint8_t width, uint8_t height, BitmapFormat fmt)
{
    if (!bitmap) return;
    if (x >= 128 || y >= 64) return;

    // Clip width/height to screen
    uint8_t drawW = width;
    if ((uint16_t)x + drawW > 128) drawW = 128 - x;

    uint8_t firstPage = y >> 3;
    uint8_t lastPage  = (uint8_t)((y + height - 1) >> 3);
    if (lastPage > 7) lastPage = 7;

    // Build one page line at a time and send in a single I2C burst
    uint8_t lineBuf[128];  // max screen width
    uint8_t dp, dx, b;

    for (dp = firstPage; dp <= lastPage; dp++) {
        uint16_t pageY0 = (uint16_t)dp * 8;   // display y of bit0 in this page

        // Compose bytes for this display page
        for (dx = 0; dx < drawW; dx++) {
            uint8_t outByte = 0;
            uint16_t sx = dx;                 // source x inside the bitmap (we’ll clip via getPixel)

            for (b = 0; b < 8; b++) {
                uint16_t yd = pageY0 + b;     // absolute display y for this bit
                int16_t  sy = (int16_t)yd - (int16_t)y; // source y relative to bitmap
                if (sy >= 0 && sy < height) {
                    if (getPixel(bitmap, width, height, sx, (uint16_t)sy, fmt))
                        outByte |= (1u << b);
                }
            }
            lineBuf[dx] = outByte;
        }

        // Send the composed line to the display at (x, page)
        ssd1306_setCursor(dp, x);
        ssd1306_data(lineBuf, drawW);
    }
}
