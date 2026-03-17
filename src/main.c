#include "stm32h723xx.h"
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


/* ================== MAIN ================== */
int main(void)
{
     

    uint16_t *fb = (uint16_t *)FB;

    for (int i = 0; i < (960 * 412); i++)
    {
        fb[i] = 0xF800; // RGB565 RED
    }

    ltdc_bringup();

    uart_send_str("\r\nSTM32 Capture Ready\r\n");

    uint8_t prev_vsync = 0;

 

    while (1)
    {
        uint8_t vsync = (GPIOB->IDR >> 8) & 1;

        /* Detect VSYNC rising edge */
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

                    uart_send_str("R");
                    uart_send_hex(r + 1);

                    uart_send_str(" AF:");
                    uart_send_hex(af);

                    uart_send_str(" MSG:");
                    uart_send_hex(msg);
                    uart_send_str(" ");

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

                    uart_send_str(out ? out : "UNKNOWN");
                    uart_send_str("\r\n");

                    uart_send_str(history_arr[r][best_index]);
                    uart_send_str("\r\n");
                }
            }
        }

        prev_vsync = vsync;
    }
}




