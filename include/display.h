#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

/* ================== DISPLAY CONFIG ================== */

#define LCD_HEIGHT 412
#define LCD_WIDTH  960

#define ROW_COUNT        4
#define CHARACTER_COUNT  26

#define CHARACTER_WIDTH  32
#define CHARACTER_HEIGHT 64

#define ROW_HEIGHT (LCD_HEIGHT / ROW_COUNT)

/* ================== COLORS ================== */

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF

/* ================== GLOBAL BUFFERS ================== */

extern uint16_t framebuffer[LCD_HEIGHT][LCD_WIDTH];
extern char row_buffer[ROW_COUNT][CHARACTER_COUNT + 1];

/* ================== GLYPH LUT ================== */

extern const uint16_t glyph_lut[128];

/* ================== DRAWING FUNCTIONS ================== */

void set_pixel(int x, int y, uint16_t color);
void draw_bitmap(int x, int y, char c);

void draw_row(int row, const char *message);
void update_row(int row, const char *text);
void clear_row(int row);

#endif