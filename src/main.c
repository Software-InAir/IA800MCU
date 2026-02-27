#include "stm32f0xx.h"

/* ================== CONFIG ================== */
#define CONF_DEPTH 8
#define BUF_SIZE  8192
#define OUTPUT_RAW_HEX  0   /* 0 = decoded, 1 = raw hex */
#define MAX_CHARS 26
char decoded_str1[MAX_CHARS + 1];   // +1 for null terminator
char decoded_str2[MAX_CHARS + 1];



char history[CONF_DEPTH][MAX_CHARS + 1];
uint8_t history_index = 0;
uint8_t history_count = 0;


/* ================== GLOBALS ================== */
volatile uint8_t  capture_buf1[BUF_SIZE];
volatile uint8_t capture_buf2[BUF_SIZE];
volatile uint32_t capture_len1 = 0;
volatile uint32_t capture_len2 = 0;

/* ================== UART ================== */
static void uart_send_char(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = c;
}

static void uart_send_hex(uint8_t v)
{
    const char hex[] = "0123456789ABCDEF";
    uart_send_char(hex[(v >> 4) & 0x0F]);
    uart_send_char(hex[v & 0x0F]);
    uart_send_char(' ');
}

static void uart_send_str(const char *s)
{
    while (*s)
        uart_send_char(*s++);
}

/* ---------------- CRC16 Modbus ---------------- */

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ---------------- Payload Builder ---------------- */

void build_and_print_payload(const char *message)
{
    uint8_t payload[128];
    uint16_t payload_len = 0;

    uint16_t msg_len = (uint16_t)strlen(message);

    for (uint16_t i = 0; i < msg_len; i++)
    {
        payload[payload_len++] = (uint8_t)message[i];
    }

    /* Pad if odd */
    if (payload_len & 0x01)
        payload[payload_len++] = 0x00;

    /* Python: length = payload_len + 5 */
    uint8_t length = (uint8_t)(payload_len + 5);

    uint8_t write_command[256];
    uint16_t write_len = 0;

    /* Header */
    write_command[write_len++] = 0x5A;
    write_command[write_len++] = 0xA5;
    write_command[write_len++] = length;
    write_command[write_len++] = 0x10;
    write_command[write_len++] = 0x00;
    write_command[write_len++] = 0x04;

    /* Payload */
    for (uint16_t i = 0; i < payload_len; i++)
        write_command[write_len++] = payload[i];

    /* CRC over [3:] */
    uint16_t crc = crc16_modbus(&write_command[3], write_len - 3);

    write_command[write_len++] = (uint8_t)(crc & 0xFF);
    write_command[write_len++] = (uint8_t)(crc >> 8);

//---------------------------Payload built constructed and intiliazed.

    /* ---------------- Debug Output ---------------- */
#ifdef DEBUG_PRINT
    printf("Message: \"%s\"\n", message);

    printf("Payload:\n");
    for (uint16_t i = 0; i < payload_len; i++)
        printf("0x%02X%s", payload[i], (i < payload_len - 1) ? ", " : "");
    printf("\n");

    printf("Write Command:\n");
    for (uint16_t i = 0; i < write_len; i++)
        printf("%02X ", write_command[i]);
    printf("\nCRC: %04X\n\n", crc);
#endif
}

static inline void delay_us_approx(uint32_t us)
{
    volatile uint32_t cycles = us * 48;
    while (cycles--)
        __NOP();
}

static void uart_delay_equivalent(uint8_t chars)
{
    /* Save PA2 mode */
    uint32_t moder = GPIOA->MODER;

    /* Set PA2 to input (disconnect TX pin) */
    GPIOA->MODER &= ~(0x3 << (2 * 2));

    for (uint8_t i = 0; i < chars; i++)
    {
        while (!(USART2->ISR & USART_ISR_TXE));
        USART2->TDR = 0x00;   // any value, never leaves pin
    }

    /* Restore PA2 AF mode */
    GPIOA->MODER = moder;
}

/* ================== LUT ================== */
/* Pattern 2 LUT ONLY */
static const char char_lut[256] = {
    [0x02] = 'A',
    [0x04] = 'B',
    [0x06] = 'C',
    [0x08] = 'D',
    [0x0A] = 'E',
    [0x0B] = 'F',
    [0x0C] = 'G',
    //[0x02] = 'H',
    //[0x0F] = 'I',
    [0x10] = 'J',
    [0x12] = 'K',
    [0x14] = 'L',
    [0x16] = 'M',
    [0x18] = 'N',
    //[0x19] = 'O',
    //[0x0B] = 'P',
    [0x1A] = 'Q',
    [0x1B] = 'R',
    [0x1D] = 'S',
    [0x1E] = 'T',
    [0x19] = 'U',
    [0x1F] = 'V',
    [0x21] = 'W',
    [0x23] = 'X',
    //[0x1E] = 'Y',
    [0x25] = 'Z',
    [0x0F] = '1',
    [0x28] = '2',
    [0x29] = '3',
    [0x2B] = '4',
    [0x2C] = '5',
    [0x2E] = '6',
    [0x30] = '7',
    [0x32] = '8',
    [0x34] = '9',
    //[0x19] = '0' Not needed, prints O for 0 on hardware.
    [0x00] = ' ',
    [0x53] = '!',
    [0x36] = '%',
    [0x3C] = '>',
    [0x38] = '*',
    [0x4D] = '-',
    [0x40] = '<'
};

static const char char_lut2[256] = {
    //[0x01] = 'A',
    [0x03] = 'B',
    [0x05] = 'C',
    [0x07] = 'D',
    [0x09] = 'E', 
    //[0x09] = 'F',
    //[0x05] = 'G',
    [0x0D] = 'H',
    [0x0E] = 'I',
    //[0x0E] = 'J',
    [0x11] = 'K',
    [0x13] = 'L',
    [0x15] = 'M',
    [0x17] = 'N',
    //[0x01] = 'O',
    //[0x03] = 'P',
    //[0x01] = 'Q',
    //[0x03] = 'R',
    [0x1C] = 'S',
    //[0x0E] = 'T',
    //[0x0D] = 'U',
    //[0x0D] = 'V',
    [0x20] = 'W',
    //[0x22] = 'X',
    [0x22] = 'Y',
    [0x24] = 'Z',
    [0x26] = '1',
    [0x27] = '2',
    //[0x27] = '3',
    [0x2A] = '4',
    //[0x09] = '5',
    [0x2D] = '6',
    [0x2F] = '7',
    [0x31] = '8',
    [0x33] = '9',
    [0x01] = '0',
    [0x52] = '!',
    [0x35] = '%',
    [0x3B] = '>',
    [0x37] = '*',
    [0x4C] = '-',
    [0x3F] = '<'
};

static inline int lut_valid1(uint8_t v)
{
    return char_lut[v] != 0;
}

static inline int lut_valid2(uint8_t v)
{
    return char_lut2[v] != 0;
}

/* ================== MAIN ================== */
int main(void)
{
    /* Enable clocks */
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN |
                    RCC_AHBENR_GPIOBEN |
                    RCC_AHBENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* USART2 pins PA2/PA3 */
    GPIOA->MODER &= ~(0xF << (2 * 2));
    GPIOA->MODER |=  (0xA << (2 * 2));
    GPIOA->AFR[0] |= (0x11 << (4 * 2));

    /* PC0–PC7 = data bus input */
    GPIOC->MODER &= ~0xFFFF;

    /* Optional: weak pull-downs for stability */
    GPIOC->PUPDR |= 0x5555;

    /* PB8 = VSYNC, PB9 = HSYNC */
    GPIOB->MODER &= ~(0xF << (8 * 2));

    /* USART2 */
    USART2->BRR  = SystemCoreClock / 115200;
    USART2->CR1  = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    uart_send_str("\r\nSTM32 Capture Ready\r\n");

    uint8_t prev_vsync = 0;

    while (1)
    {
        uint8_t vsync = (GPIOB->IDR >> 8) & 1;

        /* Detect VSYNC rising edge */
        if (vsync && !prev_vsync)
        {
            capture_len1 = 0;

            /* Allow system to settle after VSYNC */
            //uart_delay_equivalent(1600);
            delay_us_approx(22);  
            //delay_us_approx(68); 
            //delay_us_approx(118);
            //delay_us_approx(168);  
            /*
            __NOP();   // 20.8 ns
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();   // 20.8 ns
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();   // 20.8 ns
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            __NOP();   // 20.8 ns
            __NOP();
            __NOP();
            __NOP();
            __NOP();
            */

            uint8_t last_bus = GPIOC->IDR & 0xFF;

            /* Capture only while VSYNC high */
            
            while ((GPIOB->IDR >> 8) & 1)
            {
                if (capture_len1 < BUF_SIZE)
                    capture_buf1[capture_len1++] = GPIOC->IDR & 0xFF;
            }

#if OUTPUT_RAW_HEX

            for (uint32_t i = 0; i < capture_len1; i++)
            {
                uart_send_hex(capture_buf1[i]);
                if ((i & 0x1F) == 0x1F)
                    uart_send_str("\r\n");
            }

#else

            uint8_t last1 = 0;
            uint8_t first1 = 1;
            uint8_t charcounter1 = 0;

            for (uint32_t i = 0; i < capture_len1; i++)
            {
                if (charcounter1 >= MAX_CHARS)
                    break;

                uint8_t v = capture_buf1[i];

                if (lut_valid1(v))
                {
                    char c = char_lut[v];

                    /* Skip leading spaces entirely */
                    if (first1 && c == ' ')
                        continue;

                    if (first1 || v != last1 || c == ' ')
                    {
                        decoded_str1[charcounter1++] = c;
                        last1  = v;
                        first1 = 0;
                    }
                }
            }

            decoded_str1[charcounter1] = '\0';

            /* Trim leading spaces */
            char *p = decoded_str1;
            while (*p == ' ')
                p++;

            if (p != decoded_str1)
                memmove(decoded_str1, p, strlen(p) + 1);

            /* Ensure exactly 26 characters */
            /* Ensure exactly MAX_CHARS characters */
            uint8_t len = 0;
            while (decoded_str1[len] != '\0' && len < MAX_CHARS)
                len++;

            /* Pad with spaces if short */
            if (len < MAX_CHARS)
            {
                for (uint8_t i = len; i < MAX_CHARS; i++)
                    decoded_str1[i] = ' ';
            }

            /* Force fixed length */
            decoded_str1[MAX_CHARS] = '\0';

            /* ---- COLLAPSE INTERNAL MULTIPLE SPACES ---- */

            /* Find last non-space character */
            int last_non_space = -1;

            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (decoded_str1[i] != ' ')
                {
                    last_non_space = i;
                    break;
                }
            }

            /* Collapse internal multiple spaces */
            if (last_non_space >= 0)
            {
                int write = 0;
                int space_seen = 0;

                char temp[MAX_CHARS];

                for (int read = 0; read <= last_non_space; read++)
                {
                    if (decoded_str1[read] == ' ')
                    {
                        if (!space_seen)
                        {
                            temp[write++] = ' ';
                            space_seen = 1;
                        }
                    }
                    else
                    {
                        temp[write++] = decoded_str1[read];
                        space_seen = 0;
                    }
                }

                /* Fill remainder with spaces */
                while (write < MAX_CHARS)
                    temp[write++] = ' ';

                /* Copy back */
                for (int i = 0; i < MAX_CHARS; i++)
                    decoded_str1[i] = temp[i];
            }

            /* ------------------------------------------- */

            /* Replace accidental null characters with spaces */
            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (decoded_str1[i] == '\0')
                    decoded_str1[i] = ' ';
            }

            decoded_str1[MAX_CHARS] = '\0';

            /* Store decoded frame into history */
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                history[history_index][i] = decoded_str1[i];

                if (decoded_str1[i] == '\0')
                    break;
            }

            history_index++;
            if (history_index >= CONF_DEPTH)
                history_index = 0;

            if (history_count < CONF_DEPTH)
                history_count++;
            if (history_count == CONF_DEPTH)
            {
                int best_count = 0;
                int best_index = 0;

                for (uint8_t i = 0; i < CONF_DEPTH; i++)
                {
                    int count = 1;

                    for (uint8_t j = i + 1; j < CONF_DEPTH; j++)
                    {
                        int match = 1;

                        for (uint8_t k = 0; k < MAX_CHARS; k++)
                        {
                            if (history[i][k] != history[j][k])
                            {
                                match = 0;
                                break;
                            }
                        }

                        if (match)
                            count++;
                    }

                    if (count > best_count)
                    {
                        best_count = count;
                        best_index = i;
                    }
                }

                /* Output most common full row */
                uart_send_str(history[best_index]);
                uart_send_str("\r\n");
            }

            //build_and_print_payload(decoded_str1);

            //uart_send_str(decoded_str1);
            //uart_send_str("\r\n");

#endif
        }

        prev_vsync = vsync;
    }
}
