#ifndef LVGLSTUB_H
#define LVGLSTUB_H

#include <stdint.h>

/* ================== GLYPH DESCRIPTOR ================== */
/* Matches LVGL font generator output (what your font.c expects) */

typedef struct {
    uint32_t bitmap_index;   /* Index into glyph_bitmap[] */
    uint16_t adv_w;          /* Advance width (can ignore for now) */
    uint8_t box_w;           /* Glyph width in pixels */
    uint8_t box_h;           /* Glyph height in pixels */
    int8_t ofs_x;            /* X offset (can ignore initially) */
    int8_t ofs_y;            /* Y offset (can ignore initially) */
} lv_font_fmt_txt_glyph_dsc_t;

/* ================== FONT DATA ================== */
/* These are defined in your font.c */

extern const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[];
extern const uint8_t glyph_bitmap[];

#endif