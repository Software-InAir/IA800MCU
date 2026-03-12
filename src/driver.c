#include "stm32f0xx.h"
#include "lvglstub.h"
#include <string.h>



#define LCD_HEIGHT 412
#define LCD_WIDTH 960

#define ROW_COUNT 4
#define CHARACTER_COUNT 26

#define CHARACTER_WIDTH 32
#define CHARACTER_HEIGHT 64

#define ROW_HEIGHT (LCD_HEIGHT/ROW_COUNT)

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF

uint16_t framebuffer[LCD_HEIGHT][LCD_WIDTH];
char row_buffer[ROW_COUNT][CHARACTER_COUNT + 1];

void set_pixel(int,int,uint16_t);
void draw_bitmap(int,int,char);
void draw_row(int, const char*);
void update_row(int, const char*);
void clear_row(int);

const uint16_t glyph_lut[128] = {

['!'] = 5,
['%'] = 6,
['*'] = 9,
['-'] = 10,

['/'] = 11,
['0'] = 12,
['1'] = 13,
['2'] = 14,
['3'] = 15,
['4'] = 16,
['5'] = 17,
['6'] = 18,
['7'] = 19,
['8'] = 20,
['9'] = 21,
[':'] = 22,

['<'] = 23,
['='] = 24,
['>'] = 25,
['?'] = 26,
['@'] = 27,

['A'] = 28,
['B'] = 29,
['C'] = 30,
['D'] = 31,
['E'] = 32,
['F'] = 33,
['G'] = 34,
['H'] = 35,
['I'] = 36,
['J'] = 37,
['K'] = 38,
['L'] = 39,
['M'] = 40,
['N'] = 41,
['O'] = 42,
['P'] = 43,
['Q'] = 44,
['R'] = 45,
['S'] = 46,
['T'] = 47,
['U'] = 48,
['V'] = 49,
['W'] = 50,
['X'] = 51,
['Y'] = 52,
['Z'] = 53,

['['] = 54,
[']'] = 55,
['^'] = 56,

['a'] = 57,
['b'] = 58,
['c'] = 59,
['d'] = 60,
['e'] = 61,
['f'] = 62,
['g'] = 63,
['h'] = 64,
['i'] = 65,
['j'] = 66,
['k'] = 67,
['l'] = 68,
['m'] = 69,
['n'] = 70,
['o'] = 71,
['p'] = 72,
['q'] = 73,
['r'] = 74,
['s'] = 75,
['t'] = 76,
['u'] = 77,
['v'] = 78,
['w'] = 79,
['x'] = 80,
['y'] = 81,
['z'] = 82,
};

int main(void){

}

void set_pixel(int x, int y, uint16_t color)
{
    if(x < 0 || x >= 0){
        return;
    }
    if(y < 0 || y >= 0){
        return;
    }

    framebuffer[y][x] = color;
}

void draw_bitmap(int x, int y, char c)
{
    uint16_t glyph_index = glyph_lut[(uint8_t)c];

    if(!glyph_index)
        return;

    const lv_font_fmt_txt_glyph_dsc_t *g = &glyph_dsc[glyph_index];
    const uint8_t *bitmap = &glyph_bitmap[g->bitmap_index];

    int bit_index = 0;

    for(int row = 0; row < g->box_h; row++)
    {
        int py = y + row;

        for(int col = 0; col < g->box_w; col++)
        {
            uint8_t byte = bitmap[bit_index >> 3];
            uint8_t bit  = (byte >> (7 - (bit_index & 7))) & 1;

            if(bit)
                framebuffer[py][x + col] = COLOR_WHITE;

            bit_index++;
        }
    }
}

void draw_row(int row, const char* message){

    if(row >= ROW_COUNT){
        return;
    }

    int y = row * ROW_HEIGHT;
    int cursor_x = 0;

    for(int i = 0; message[i] != '\0' && i < CHARACTER_COUNT; i++)
    {
        draw_bitmap(cursor_x, y, message[i]);

        cursor_x += CHARACTER_WIDTH;
    }
}

void update_row(int row, const char *text)
{
    if(row >= ROW_COUNT)
        return;

    clear_row(row);

    strncpy(row_buffer[row], text, CHARACTER_COUNT);
    row_buffer[row][CHARACTER_COUNT] = '\0';

    draw_row(row, row_buffer[row]);
}

void clear_row(int row)
{
    int y0 = row * ROW_HEIGHT;

    for(int y = y0; y < y0 + ROW_HEIGHT; y++)
    {
        for(int x = 0; x < LCD_WIDTH; x++)
        {
            framebuffer[y][x] = COLOR_BLACK;
        }
    }

    memset(row_buffer[row], ' ', CHARACTER_COUNT);
    row_buffer[row][CHARACTER_COUNT] = '\0';
}


