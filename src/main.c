#include "stm32f0xx.h"

/* ================== CONFIG ================== */
#define BUF_SIZE        4096
#define OUTPUT_RAW_HEX  0   /* 0 = decoded, 1 = raw hex */
#define MAX_CHARS 17
char decoded_str1[MAX_CHARS + 1];   // +1 for null terminator
char decoded_str2[MAX_CHARS + 1];


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

    /* PC0–PC7 = data bus */
    GPIOC->MODER &= ~0xFFFF;

    /* PB8 = VSYNC, PB9 = HSYNC */
    GPIOB->MODER &= ~(0xF << (8 * 2));

    /* USART2 */
    USART2->BRR  = SystemCoreClock / 115200;
    USART2->CR1  = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // ------------------------------ STM32 SETUP COMPLETE

    uart_send_str("\r\nSTM32 Capture Ready\r\n");

    uint8_t prev_vsync = 0;
    uint8_t prev_hsync = 0;

    while (1)
    {
        uint8_t vsync = (GPIOB->IDR >> 8) & 1;
        uint8_t hsync = (GPIOB->IDR >> 9) & 1;

        /* VSYNC rising edge */
        if (vsync && !prev_vsync)
        {
            capture_len1 = 0;
            capture_len2 = 0;
            uart_delay_equivalent(22);
            
            /* Capture during VSYNC */
            while ((GPIOB->IDR >> 8) & 1)
            {
                hsync = (GPIOB->IDR >> 9) & 1;

                /* HSYNC rising edge */
                //if (hsync && !prev_hsync)
                //{
                    if (capture_len1 < BUF_SIZE)
                        capture_buf1[capture_len1++] = GPIOC->IDR & 0xFF;
                    if (capture_len2 < BUF_SIZE)
                        capture_buf2[capture_len2++] = GPIOC->IDR & 0xFF;
                //}

                //prev_hsync = hsync;
            }

            //uart_send_str("\r\n--- VSYNC FALL ---\r\n");

#if OUTPUT_RAW_HEX
            /* ========= RAW HEX DUMP ========= */
            for (uint32_t i = 0; i < capture_len; i++)
            {
                uart_send_hex(capture_buf[i]);
                if ((i & 0x1F) == 0x1F)
                    uart_send_str("\r\n");
            }

#else
            /* ========= DECODED OUTPUT ========= */
        uint8_t last1 = 0;
        uint8_t first1 = 1;
        uint8_t charcounter1 = 0;

        uint8_t last2 = 0;
        uint8_t first2 = 1;
        uint8_t charcounter2 = 0;

        for (uint32_t i = 0; i < capture_len1; i++)
        {
            if (charcounter1 >= MAX_CHARS)
                break;

            uint8_t v = capture_buf1[i];
          

            if ((first1 || v != last1) && lut_valid(v))
            {
                decoded_str1[charcounter1++] = char_lut[v];
                last1  = v;
                first1 = 0;
            }
        }

        // -------------------------------- Second capture set stored

        for (uint32_t i = 0; i < capture_len2; i++)
        {
            if (charcounter2 >= MAX_CHARS)
                break;

            uint8_t v2 = capture_buf2[i];

            if ((first2 || v2 != last2) && lut_valid2(v2))
            {
                decoded_str2[charcounter2++] = char_lut2[v2];
                last2  = v2;
                first2 = 0;
            }
        }

        // ------------------------------- First capture set stored

        /* Null-terminate strings */
        decoded_str1[charcounter1] = '\0';
        decoded_str2[charcounter2] = '\0';

        /* -------- FIX: trim leading spaces / null-derived chars -------- */
        char *p = decoded_str1;
        char *p2 = decoded_str2;
        while (*p == ' ')
            p++;

        if (p != decoded_str1)
            memmove(decoded_str1, p, strlen(p) + 1);
        
        while (*p2 == ' ')
            p2++;
        
        if (p2 != decoded_str2)
            memmove(decoded_str2, p2, strlen(p2) + 1);

    

        for(uint32_t i = 0; i < strlen(decoded_str1); i++)
        {
            char c1 = decoded_str1[i];
            char c2 = decoded_str2[i];
            
            //(A, H) = if [0x02], reference decoded_str2[i], if decoded_str1[i] = char_lut2[H] swap [0x02] for char_lut2[H]
            if(c1 == 'A' && c2 == 'H')
            {
                decoded_str1[i] = 'H';
            }
            if(c1 == '1' && c2 == 'I')
            {
                decoded_str1[i] = 'I';
            }
            if(c1 == 'F' && c2 == 'P')
            {
                decoded_str1[i] = 'P';
            }
            if(c1 == 'U' && c2 == '0')
            {
                decoded_str1[i] = 'O';
            }
            //(T, Y) = if [0x1E], reference decoded_str2[i], if decoded_str1[i] = char_lut2[Y] swap [0x1E] for char_lut2[Y]
            if(c1 == 'T' && c2 == 'Y')
            {
                decoded_str1[i] = 'Y';
            }
            
        }

        build_and_print_payload(decoded_str1);

        uart_send_str(decoded_str1);
        uart_send_str("\r\n");


        
#endif

            //uart_send_str("\r\n--- END FRAME ---\r\n");
        }

        prev_vsync = vsync;
    }
}