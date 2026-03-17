#include "stm32h723xx.h"
#include "display.h"
#include "capture.h"
#include "decoder.h"
#include "history.h"
#include "classifier.h"
#include "uart.h"
#include "config.h"



/* ================== CONFIG ================== */

// Panel timing bring up guesses

#define FB ((uint32_t)0x24000000)

// Horizontal
#define H_SYNC   10
#define H_BACK   20
#define H_ACTIVE 960
#define H_FRONT  10

// Vertical
#define V_SYNC   2
#define V_BACK   4
#define V_ACTIVE 412
#define V_FRONT  2

/* =======================================================================
   ===================== DISPLAY BRING-UP MANUAL =========================
   =======================================================================

   Use: set BRINGUP_STAGE in config.h and step through 1 → 7 sequentially.

   -----------------------------------------------------------------------
   STAGE 0 - CAPTURE TRUTH
   -----------------------------------------------------------------------
   EXPECT:
     - Terminal output of buffer capture from ID-800
   
    BE PREPARED FOR:
     - Nothing
     - Garbled garbage


     
   -----------------------------------------------------------------------
   STAGE 1 — SOLID COLOR (GREEN)
   -----------------------------------------------------------------------
   EXPECT:
     - Entire screen solid green

   FAILURES:
     - Black screen:
         → LTDC not initialized
         → wrong framebuffer address
         → panel timing wrong
         → GPIO/LTDC pins not configured

     - Flickering / unstable color:
         → bad sync timing (HSYNC/VSYNC)
         → pixel clock incorrect

     - Wrong color:
         → RGB565 format mismatch
         → byte order issue

   -----------------------------------------------------------------------
   STAGE 2 — PIXEL / BOUNDS TEST
   -----------------------------------------------------------------------
   EXPECT:
     - One white pixel at (100,100)
     - Red pixel top-left
     - Green pixel bottom-right

   FAILURES:
     - Pixel in wrong location:
         → stride incorrect (width mismatch)
         → X/Y swapped

     - Mirrored or flipped:
         → LTDC polarity / orientation issue

     - Lines instead of pixels:
         → memory alignment or color format issue

   -----------------------------------------------------------------------
   STAGE 3 — SINGLE GLYPH ('A')
   -----------------------------------------------------------------------
   EXPECT:
     - Clear, properly shaped 'A' at (100,100)

   FAILURES:
     - Nothing:
         → glyph_lut mapping wrong
         → glyph index = 0

     - Garbled shape:
         → bit unpacking wrong
         → bitmap pointer incorrect

     - Shifted or clipped:
         → box_w / box_h mismatch
         → coordinate math error

   -----------------------------------------------------------------------
   STAGE 4 — ROW RENDERING
   -----------------------------------------------------------------------
   EXPECT:
     - "HELLO WORLD 123" on row 0
     - Proper spacing, no overlap

   FAILURES:
     - Overlapping characters:
         → CHARACTER_WIDTH incorrect

     - Too much spacing:
         → incorrect cursor increment

     - Row misplaced vertically:
         → ROW_HEIGHT calculation wrong

     - Truncated string:
         → CHARACTER_COUNT too small

   -----------------------------------------------------------------------
   STAGE 5 — UPDATE_ROW() (NO FLICKER)
   -----------------------------------------------------------------------
   EXPECT:
     - Row only updates when content changes
     - No flicker on unchanged rows

   FAILURES:
     - Flicker every loop:
         → missing strcmp/strncmp check

     - Old text remains:
         → clear_row() not clearing fully

     - Random redraws:
         → row_buffer not null-terminated

   -----------------------------------------------------------------------
   STAGE 6 — DYNAMIC ROW UPDATES
   -----------------------------------------------------------------------
   EXPECT:
     - Only changed rows update over time
     - Stable display, no full redraw

   FAILURES:
     - All rows updating:
         → update_row() logic incorrect

     - Ghosting:
         → clear_row incomplete

     - Timing feels unstable:
         → loop too fast / no delay

   -----------------------------------------------------------------------
   STAGE 7 — FULL SYSTEM (CAPTURE → DISPLAY)
   -----------------------------------------------------------------------
   EXPECT:
     - Screen initially blank
     - Rows populate after CONF_DEPTH cycles
     - Updates only when classification changes

   FAILURES:
     - Nothing ever appears:
         → VSYNC not triggering
         → capture_rows() not working
         → history never reaches CONF_DEPTH

     - Flickering messages:
         → classification unstable
         → history depth too low

     - Wrong messages:
         → decode or LUT mismatch

     - Only one row works:
         → indexing issue in pipeline

   -----------------------------------------------------------------------
   GLOBAL FAILURE CHECKS
   -----------------------------------------------------------------------
     - If Stage 1 works but others don’t:
         → framebuffer mismatch (display.c vs LTDC)

     - If later stages draw nothing:
         → display.c not writing to 0x24000000

     - If behavior inconsistent:
         → memory corruption / buffer overflow

   -----------------------------------------------------------------------
   RULES
   -----------------------------------------------------------------------
     - Do NOT skip stages
     - Each stage must be visually correct before proceeding
     - Only debug one stage at a time

   ======================================================================= */


/* ================== MAIN ================== */
int main(void)
{
     
    uint16_t *fb = (uint16_t *)FB;

#if BRINGUP_STAGE == 0

    uart_init();

    while (1)
    {
        capture_rows();

        for (int r = 0; r < ROWS; r++)
        {
            decode_row((uint8_t*)capture_buf[r], capture_len[r], decoded_str[r]);
            normalize_string(decoded_str[r]);

            uart_print("ROW ");
            uart_print_int(r);
            uart_print(": ");
            uart_print(decoded_str[r]);
            uart_print("\r\n");
        }

        // small delay so terminal is readable
        for (volatile int i = 0; i < 2000000; i++);
    }

#endif


#if BRINGUP_STAGE == 1

    /*-------------------------------------- Bring up stage 1: Screen color --------------------*/

    ltdc_bringup();

    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x07E0; // GREEN
    }

    

    while (1)
    {
        // Do nothing — just display color
    }


#elif BRINGUP_STAGE == 2

    /* -----------------------------------Bring up stage 2: Pixels and screen bounds --------*/

    ltdc_bringup();

    /* Clear screen to black */
    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x0000;
    }

    /* Single pixel test */
    int x = 100;
    int y = 100;

    fb[y * 960 + x] = 0xFFFF; // WHITE pixel
    fb[0 * 960 + 0] = 0xF800;       // top-left RED
    fb[(412-1) * 960 + (960-1)] = 0x07E0; // bottom-right GREEN

    while (1) {}



#elif BRINGUP_STAGE == 3

    /* ----------------------------- Bring up stage 3: LUT and bitmap drawing ------------------*/

    ltdc_bringup();

    /* Clear screen */
    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x0000;
    }

    /* Draw one glyph */
    draw_bitmap(100, 100, 'A');

    while (1) {}



#elif BRINGUP_STAGE == 4

    /* -------------------------------- Bring up stage 4: Row rendering ------------------*/

    ltdc_bringup();

    /* Clear screen */
    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x0000;
    }

    /* Draw one full row */
    draw_row(0, "HELLO WORLD 123");

    while (1) {}



#elif BRINGUP_STAGE == 5

    /* -------------------------------- Bring up Stage 5: Row update and flicker check ----------------*/

    ltdc_bringup();

    /* Clear screen */
    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x0000;
    }

    /* Initial state */
    update_row(0, "HELLO");
    update_row(1, "WORLD");

    while (1)
    {
        // Simulate change after some time (conceptual)
        update_row(0, "HELLO");   // should NOT redraw
        update_row(1, "WORLD!");  // SHOULD redraw
    }



#elif BRINGUP_STAGE == 6

    /* --------------------------- Bring up Stage 6: Independent Row Update ------------------*/

    ltdc_bringup();

    /* Clear screen */
    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x0000;
    }

    /* Initial rows */
    update_row(0, "INITIAL");
    update_row(1, "STATE");
    update_row(2, "TEST");
    update_row(3, "READY");

    volatile int delay;

    while (1)
    {
        /* crude delay */
        for (delay = 0; delay < 5000000; delay++);

        update_row(0, "INITIAL");   // should NOT redraw
        update_row(1, "STATE");     // should NOT redraw

        update_row(2, "RUNNING");   // SHOULD update
        update_row(3, "READY");     // no change

        for (delay = 0; delay < 5000000; delay++);

        update_row(2, "TEST");      // SHOULD update back
    }


#elif BRINGUP_STAGE == 7

    ltdc_bringup();

    /* Clear framebuffer once */
    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0x0000;
    }

    uint8_t prev_vsync = 0;

    while (1)
    {
        uint8_t vsync = (GPIOB->IDR >> 8) & 1;

        if (vsync && !prev_vsync)
        {
            capture_rows();

            for (int r = 0; r < ROWS; r++)
            {
                decode_row((uint8_t*)capture_buf[r], capture_len[r], decoded_str[r]);
                normalize_string(decoded_str[r]);

                history_push(r, decoded_str[r]);

                if (history_count_arr[r] == CONF_DEPTH)
                {
                    int best_index = history_vote(r);

                    airframe_t af;
                    int msg;

                    classify_message(history_arr[r][best_index], &af, &msg);

                    const char *out = NULL;

                    switch (af)
                    {
                        case AIRFRAME_ATR:
                            if (msg < ATR_MSG_COUNT) out = atr[msg];
                            break;

                        case AIRFRAME_BAE:
                            if (msg < BAE_MSG_COUNT) out = bae[msg];
                            break;

                        case AIRFRAME_DHC:
                            if (msg < DHC_MSG_COUNT) out = dhc[msg];
                            break;

                        case AIRFRAME_CITATION:
                            if (msg < CITATION_MSG_COUNT) out = citation[msg];
                            break;
                    }

                    update_row(r, out ? out : "UNKNOWN");
                }
            }
        }

        prev_vsync = vsync;
    }

#endif

}