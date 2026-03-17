#include "stm32h723xx.h"
#include "capture.h"
#include "config.h"

/* shared buffers */
volatile uint8_t  capture_buf[ROWS][BUF_SIZE];
volatile uint32_t capture_len[ROWS];

static inline void delay_us_approx(uint32_t us)
{
    volatile uint32_t cycles = us * 48;
    while (cycles--)
        __NOP();
}

void capture_rows(void)
{
    const uint32_t row_offsets[ROWS] = {22, 98, 199, 357};
    const uint8_t  row_map[ROWS]     = {0, 1, 3, 2};

    uint32_t prev = 0;

    for (int i = 0; i < ROWS; i++)
    {
        uint32_t delay = row_offsets[i] - prev;
        prev = row_offsets[i];

        int r = row_map[i];

        capture_len[r] = 0;

        delay_us_approx(delay);

        while ((GPIOB->IDR >> 8) & 1 && capture_len[r] < BUF_SIZE)
        {
            capture_buf[r][capture_len[r]++] = GPIOC->IDR & 0xFF;
        }
    }
}