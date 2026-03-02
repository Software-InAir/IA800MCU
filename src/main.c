#include "stm32f0xx.h"

/* ================== CONFIG ================== */
#define CONF_DEPTH 8
#define BUF_SIZE  1024
#define OUTPUT_RAW_HEX  0   /* 0 = decoded, 1 = raw hex */
#define MAX_CHARS 26
char decoded_str1[MAX_CHARS + 1];   // +1 for null terminator
char decoded_str2[MAX_CHARS + 1];
char decoded_str3[MAX_CHARS + 1];
char decoded_str4[MAX_CHARS + 1];

char check_str1[MAX_CHARS + 1];   // +1 for null terminator
char check_str2[MAX_CHARS + 1];
char check_str3[MAX_CHARS + 1];
char check_str4[MAX_CHARS + 1];

char check_history1[CONF_DEPTH][MAX_CHARS + 1];
char check_history2[CONF_DEPTH][MAX_CHARS + 1];
char check_history3[CONF_DEPTH][MAX_CHARS + 1];
char check_history4[CONF_DEPTH][MAX_CHARS + 1];

uint8_t check_history_index1 = 0;
uint8_t check_history_index2 = 0;
uint8_t check_history_index3 = 0;
uint8_t check_history_index4 = 0;

uint8_t check_history_count1 = 0;
uint8_t check_history_count2 = 0;
uint8_t check_history_count3 = 0;
uint8_t check_history_count4 = 0;




char history[CONF_DEPTH][MAX_CHARS + 1];
uint8_t history_index = 0;
uint8_t history_count = 0;

char history2[CONF_DEPTH][MAX_CHARS + 1];
uint8_t history_index2 = 0;
uint8_t history_count2 = 0;

char history3[CONF_DEPTH][MAX_CHARS + 1];
uint8_t history_index3 = 0;
uint8_t history_count3 = 0;

char history4[CONF_DEPTH][MAX_CHARS + 1];
uint8_t history_index4 = 0;
uint8_t history_count4 = 0;


/* ================== GLOBALS ================== */
volatile uint8_t  capture_buf1[BUF_SIZE];
volatile uint8_t  capture_buf2[BUF_SIZE];
volatile uint8_t  capture_buf3[BUF_SIZE];
volatile uint8_t  capture_buf4[BUF_SIZE];
volatile uint32_t capture_len1 = 0;
volatile uint32_t capture_len2 = 0;
volatile uint32_t capture_len3 = 0;
volatile uint32_t capture_len4 = 0;

volatile uint8_t  check_buf1[BUF_SIZE];
volatile uint8_t  check_buf2[BUF_SIZE];
volatile uint8_t  check_buf3[BUF_SIZE];
volatile uint8_t  check_buf4[BUF_SIZE];
volatile uint32_t check_len1 = 0;
volatile uint32_t check_len2 = 0;
volatile uint32_t check_len3 = 0;
volatile uint32_t check_len4 = 0;


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
    [0x3F] = '<',
    [0x00] = '0'
};

static const char atr_message_lut[256] = {
    [0] = 'AP/YD DISENGAGED',
    [1] = 'AP DISENGAGED',
    [2] = 'YD DISENGAGED',
    [3] = 'AILERON MISTRIM',
    [4] = 'PITCH TRIM FAIL',
    [5] = 'PITCH MISTRIM',
    [6] = 'RETRIM ROLL R WING DN',
    [7] = 'RETRIM ROLL L WING DN',
    [8] = 'PITCH MISTRIM NOSE UP',
    [9] = 'PITCH MISTRIM NOSE DOWN',
    [10] = 'DISENGAGE ANNUN DATA FAULT',
    [11] = 'ENGAGE INHIBIT',
    [12] = 'NO ENGAGEMENT ON GROUND',
    [13] = 'NO WOW',
    [14] = 'IAS TOO HIGH',
    [15]= 'CPL DATA INVALID',
    [16] = 'AHRS DATA INVALID',
    [17] = 'DADC DATA INVALID',
    [18] = 'AFCS INVALID',
    [19] = 'AP INVALID',
    [20] = 'CPL DATA INVALID',
    [21] = 'AHRS DATA INVALID',
    [22] = 'DADC DATA INVALID',
    [23] = 'AFCS INVALID',
    [24] = 'AP INVALID',
    [25] = 'NAV MISMATCH L SEL',
    [26] = 'NAV MISMATCH R SEL',
    [27] = 'EXCESS DEV',
    [28] = 'CAT 2 INVALID',
    [29] = 'CHECK NAV SOURCE',
    [30] = 'NO ENGAGEMENT ON GROUND',
};

static const char bae[256] = {
    [0] = 'TCS ENGAGE',
    [1] = 'HSI SELECT >',
    [2] = '< HSI SELECT',
    [3] = '< HSI SELECT >',
    [4] = 'L (R) AP/YD ENG',
    [5] = 'L (R) YD ENG',
    [6] = 'SYSTEM TEST',
    [7] = 'L AFCS MASTER',
    [8] = 'R AFCS MASTER',
    [9] = 'SELECT INHIBIT',
    [10] = 'L (R) AFCS FAIL',
    [11] = 'AP SERVOS FAIL',
    [12] = 'ENGAGE INHIBIT',
    [13] = 'DADC INVLD',
    [14] = 'AHRS INVLD',
    [15] = 'ACFT ON GND',
    [16] = 'NO GND TST',
    [17] = 'LOW BANK',
    [18] = 'AP/YD DISENGAGE',
    [19] = 'AP DISENGAGE',
    [20] = 'YD DISENGAGE',
    [21] = 'PITCH RETRIM NOSE UP',
    [22] = 'PITCH RETRIM NOSE DN',
    [23] = 'PITCH TRIM FAIL',
    [24] = 'RETRIM ROLL R WING DN',
    [25] = 'RETRIM ROLL L WING DN',
    [26] = 'EXCESSIVE DEV',
    [27] = 'DISENGAGE ANNUN',
    [28] = 'DATA FAULT',
    [29] = 'NO WOW',
    [30] = 'IAS HIGH',
    [31] = 'HSI SEL DATA INVALID',
    [32] = 'AHRS DATA INVALID',
    [33] = 'DADC DATA INVALID',
    [34] = 'NAV MISMATCH [L SEL]',
    [35] = 'NAV MISMATCH [R SEL]',
    [36] = 'CHECK NAV SOURCE',
    [37] = 'L (R) AFCS FAIL',
    [38] = 'ALT OFF',
    [39] = 'INVALID OPERATION'
}

static const char DHC[256] {
    [0] = 'ΗDG HOLD',
    [1] = 'PITCH HOLD',
    [2] = 'HDG SEL',
    [3] = 'ALT',
    [4] = 'WINGS LEVEL',
    [5] = 'AP FAIL/YD AVAIL',
    [6] = 'AHRS DATA INVLD',
    [7] = 'DADC DATA INVLD',
    [8] = 'AP DISENGAGED',
    [9] = 'AP/YD DISENGAGED',
    [10] = 'YD DISENGAGED',
    [11] = 'L AP/YD FAIL',
    [12] = 'R AP/YD FAIL',
    [13] = 'MISTRIM [TRIM L WING DN]',
    [14] = 'MISTRIM [TRIM R WING DN]',
    [15] = 'MISTRIM [TRIM NOSE UP]',
    [16] = 'MISTRIM [TRIM NOSE DN]',
    [17] = 'ADI PITCH/ROLL MISMATCH',
    [18] = 'ADI PITCH MISMATCH',
    [19] = 'ADI ROLL MISMATCH',
    [20] = 'HSI HDG MISMATCH',
    [21] = 'FD NAV MISMATCH R VALID',
    [22] = 'FD NAV MISMATCH L VALID'
}

static const char citation[256] = {
    [0] = 'HSI SEL >',
    [1] = '< HSI SEL',
    [2] = '< HSI SEL >',
    [3] = 'A AFCS MASTER',
    [4] = 'B AFCS MASTER',
    [5] = 'AP/YD ENGAGE',
    [6] = 'YD ENGAGED',
    [7] = 'LOW BANK',
    [8] = 'AP/YD DISENGAGE',
    [9] = 'AP DISENGAGE',
    [10] = 'YD DISENGAGE',
    [11] = 'ELEV MISTRIM NOSE UP',
    [12] = 'ELEV MISTRM NOSE DN',
    [13] = 'ELEV TRIM FAIL',
    [14] = 'RETRIM ROLL R WING DOWN',
    [15] = 'RETRIM ROLL L WING DOWN',
    [16] = 'DISENGAGE ANNUN',
    [17] = 'DATA FAULT',
    [18] = 'HSI DATA INVLD',
    [19] = 'AHRS INVALID',
    [20] = 'DADC INVALID',
    [21] = 'NAV MISMATCH [L SEL]',
    [22] = 'NAV MISMATCH [R SEL]',
    [23] = 'CHECK NAV SOURCE',
    [24] = 'AFCS FAIL',
    [25] = 'ALT OFF'
}

// still to write.. the f$ck iof


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
            // = GPIOC->IDR & 0xFF;

            delay_us_approx(22);   // row 1 offset

            while ((GPIOB->IDR >> 8) & 1 && capture_len1 < BUF_SIZE)
            {
                capture_buf1[capture_len1++] = GPIOC->IDR & 0xFF;  
            }
            capture_len2 = 0;
            delay_us_approx(98);  // move forward to row 2 window

            while ((GPIOB->IDR >> 8) & 1 && capture_len2 < BUF_SIZE)
            {
                capture_buf2[capture_len2++] = GPIOC->IDR & 0xFF;
            }
            
            capture_len4 = 0;
            delay_us_approx(199);  // move forward to row 2 window

            while ((GPIOB->IDR >> 8) & 1 && capture_len4 < BUF_SIZE)
            {
                capture_buf4[capture_len4++] = GPIOC->IDR & 0xFF;
            }

            capture_len3 = 0;
            delay_us_approx(357);  // move forward to row 2 windo
            

            while ((GPIOB->IDR >> 8) & 1 && capture_len3 < BUF_SIZE)
            {
                capture_buf3[capture_len3++] = GPIOC->IDR & 0xFF;
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
            uint8_t last2 = 0;
            uint8_t first2 = 1;
            uint8_t last3 = 0;
            uint8_t first3 = 1;
            uint8_t last4 = 0;
            uint8_t first4 = 1;
            uint8_t charcounter1 = 0;
            uint8_t charcounter2 = 0;
            uint8_t charcounter3 = 0;
            uint8_t charcounter4 = 0;

            /*
            uint8_t clast1 = 0;
            uint8_t cfirst1 = 1;
            uint8_t clast2 = 0;
            uint8_t cfirst2 = 1;
            uint8_t clast3 = 0;
            uint8_t cfirst3 = 1;
            uint8_t clast4 = 0;
            uint8_t cfirst4 = 1;
            uint8_t ccharcounter1 = 0;
            uint8_t ccharcounter2 = 0;
            uint8_t ccharcounter3 = 0;
            uint8_t ccharcounter4 = 0;
            */

            char c1prev = ' ';
            char c12prev = ' ';
            char c2prev = ' ';
            char c22prev = ' ';
            char c3prev = ' ';
            char c32prev = ' ';
            char c4prev = ' ';
            char c42prev = ' ';

            /*
            char cc1prev = ' ';
            char cc12prev = ' ';
            char cc2prev = ' ';
            char cc22prev = ' ';
            char cc3prev = ' ';
            char cc32prev = ' ';
            char cc4prev = ' ';
            char cc42prev = ' ';
            */

            for (uint32_t i = 0; i < capture_len1; i++)
            {
                if (charcounter1 >= MAX_CHARS)
                    break;

                uint8_t v1 = capture_buf1[i];
                
    

                if (lut_valid1(v1))
                {
                    char c1 = char_lut[v1];
                    
                    
                    /* Skip leading spaces entirely */
                    if (first1 && c1 == ' ')
                        continue;

                    if (first1 || v1 != last1 || c1 == ' ')
                    {
                        decoded_str1[charcounter1++] = c1;
                        last1  = v1;
                        first1 = 0;
                        c12prev = c1prev;
                        c1prev  = c1; 
                    }

                    if (((c12prev == 'A' && c1prev == c1) || (c12prev == 'E' && c1prev == c1) || (c12prev == 'I' && c1prev == c1) || (c12prev == 'O' && c1prev == c1)|| (c12prev == 'U' && c1prev == c1)) && (c1 != 'S')){
                        decoded_str1[charcounter1++] = c1;
                        last1  = v1;
                        first1 = 0;
                        c12prev = c1prev;
                        c1prev  = c1; 
                    }

                    if ((c1 == '0' || c1 == '1' || c1 == '2' || c1 == '3' 
                        || c1 == '4' || c1 == '5' || c1 == '6'
                        || c1 == '7' || c1 == '8' || c1 == '9') 
                        && (c12prev != 'A' && c12prev != 'B' && c12prev != 'C' &&
                            c12prev != 'D' && c12prev != 'E' && c12prev != 'F' && 
                            c12prev != 'G' && c12prev != 'H' && c12prev != 'I' &&
                            c12prev != 'J' && c12prev != 'K' && c12prev != 'L' &&
                            c12prev != 'M' && c12prev != 'N' && c12prev != 'O' &&
                            c12prev != 'P' && c12prev != 'Q' && c12prev != 'R' &&
                            c12prev != 'S' && c12prev != 'T' && c12prev != 'U' &&
                            c12prev != 'V' && c12prev != 'W' && c12prev != 'X' &&
                            c12prev != 'Y' && c12prev != 'Z')){
                        decoded_str1[charcounter1++] = c1;
                        last1  = v1;
                        first1 = 0;
                        c12prev = c1prev;
                        c1prev  = c1;   
                        }

                   
                }
            }

            for (uint32_t i = 0; i < capture_len2; i++)
            {
                if (charcounter2 >= MAX_CHARS)
                    break;

                
                uint8_t v2 = capture_buf2[i];
                
               
                if (lut_valid1(v2))
                {
                    char c2 = char_lut[v2];
                    
                    
                    
                    if (first2 && c2 == ' ')
                        continue;

                    if (first2 || v2 != last2 || c2 == ' ')
                    {
                        decoded_str2[charcounter2++] = c2;
                        last2  = v2;
                        first2 = 0;
                        c22prev = c2prev;
                        c2prev  = c2;   
                    }

                    if (((c22prev == 'A' && c2prev == c2) || (c22prev == 'E' && c2prev == c2) || (c22prev == 'I' && c2prev == c2) || (c22prev == 'O' && c2prev == c2) || (c22prev == 'U' && c2prev == c2)) && (c2 != 'S')){
                        decoded_str2[charcounter2++] = c2;
                        last2  = v2;
                        first2 = 0;
                        c22prev = c2prev;
                        c2prev  = c2;   
                    }

                    if ((c2 == '0' || c2 == '1' || c2 == '2' || c2 == '3' 
                        || c2 == '4' || c2 == '5' || c2 == '6'
                        || c2 == '7' || c2 == '8' || c2 == '9') 
                        && (c22prev != 'A' && c22prev != 'B' && c22prev != 'C' &&
                            c22prev != 'D' && c22prev != 'E' && c22prev != 'F' && 
                            c22prev != 'G' && c22prev != 'H' && c22prev != 'I' &&
                            c22prev != 'J' && c22prev != 'K' && c22prev != 'L' &&
                            c22prev != 'M' && c22prev != 'N' && c22prev != 'O' &&
                            c22prev != 'P' && c22prev != 'Q' && c22prev != 'R' &&
                            c22prev != 'S' && c22prev != 'T' && c22prev != 'U' &&
                            c22prev != 'V' && c22prev != 'W' && c22prev != 'X' &&
                            c22prev != 'Y' && c22prev != 'Z')) {
                        decoded_str2[charcounter2++] = c2;
                        last2  = v2;
                        first2 = 0;
                        c22prev = c2prev;
                        c2prev  = c2;       
                        }
                }
            }

            for (uint32_t i = 0; i < capture_len3; i++)
            {
                if (charcounter3 >= MAX_CHARS)
                    break;

                uint8_t v3 = capture_buf3[i];
              
                int index3 = 0;

                if (lut_valid1(v3))
                {
                    char c3 = char_lut[v3];
                    
                    
                    /* Skip leading spaces entirely */
                    if (first3 && c3 == ' ')
                        continue;

                    if (first3 || v3 != last3 || c3 == ' ')
                    {
                        decoded_str3[charcounter3++] = c3;
                        last3  = v3;
                        first3 = 0;
                        c32prev = c3prev;
                        c3prev  = c3;
                    }
                    
                    if (((c32prev == 'A' && c3prev == c3) || (c32prev == 'E' && c3prev == c3) || (c32prev == 'I' && c3prev == c3) || (c32prev == 'O' && c3prev == c3) || (c32prev == 'U' && c3prev == c3)) && (c3 != 'S')){
                        decoded_str3[charcounter3++] = c3;
                        last3  = v3;
                        first3 = 0;
                        c32prev = c3prev;
                        c3prev  = c3;
                    }
                        

                    if ((c3 == '0' || c3 == '1' || c3 == '2' || c3 == '3' 
                        || c3 == '4' || c3 == '5' || c3 == '6'
                        || c3 == '7' || c3 == '8' || c3 == '9') 
                        && (c32prev != 'A' && c32prev != 'B' && c32prev != 'C' &&
                            c32prev != 'D' && c32prev != 'E' && c32prev != 'F' && 
                            c32prev != 'G' && c32prev != 'H' && c32prev != 'I' &&
                            c32prev != 'J' && c32prev != 'K' && c32prev != 'L' &&
                            c32prev != 'M' && c32prev != 'N' && c32prev != 'O' &&
                            c32prev != 'P' && c32prev != 'Q' && c32prev != 'R' &&
                            c32prev != 'S' && c32prev != 'T' && c32prev != 'U' &&
                            c32prev != 'V' && c32prev != 'W' && c32prev != 'X' &&
                            c32prev != 'Y' && c32prev != 'Z')){
                        decoded_str3[charcounter3++] = c3;
                        last3  = v3;
                        first3 = 0;
                        c32prev = c3prev;
                        c3prev  = c3;
                        }
        
                }
            }

            for (uint32_t i = 0; i < capture_len4; i++)
            {
                if (charcounter4 >= MAX_CHARS)
                    break;

               
                uint8_t v4 = capture_buf4[i];
            
               

                if (lut_valid1(v4))
                {
                    char c4 = char_lut[v4];
                    
                    
                    /* Skip leading spaces entirely */
                    if (first4 && c4 == ' ')
                        continue;

                    if (first4 || v4 != last4 || c4 == ' ')
                    {
                        decoded_str4[charcounter4++] = c4;
                        last4  = v4;
                        first4 = 0;
                        c42prev = c4prev;
                        c4prev  = c4; 
                    }

                    if (((c42prev == 'A' && c4prev == c4) || (c42prev == 'E' && c4prev == c4) || (c42prev == 'I' && c4prev == c4) || (c42prev == 'O' && c4prev == c4) || (c42prev == 'U' && c4prev == c4)) && (c4 != 'S')){
                        decoded_str4[charcounter4++] = c4;
                        last4 = v4;
                        first4 = 0;
                        c42prev = c4prev;
                        c4prev  = c4; 
                    }

                    if ((c4 == '0' || c4 == '1' || c4 == '2' || c4 == '3' 
                        || c4 == '4' || c4 == '5' || c4 == '6'
                        || c4 == '7' || c4 == '8' || c4 == '9') 
                        && (c42prev != 'A' && c42prev != 'B' && c42prev != 'C' &&
                            c42prev != 'D' && c42prev != 'E' && c42prev != 'F' && 
                            c42prev != 'G' && c42prev != 'H' && c42prev != 'I' &&
                            c42prev != 'J' && c42prev != 'K' && c42prev != 'L' &&
                            c42prev != 'M' && c42prev != 'N' && c42prev != 'O' &&
                            c42prev != 'P' && c42prev != 'Q' && c42prev != 'R' &&
                            c42prev != 'S' && c42prev != 'T' && c42prev != 'U' &&
                            c42prev != 'V' && c42prev != 'W' && c42prev != 'X' &&
                            c42prev != 'Y' && c42prev != 'Z')) {
                        decoded_str4[charcounter4++] = c4;
                        last4  = v4;
                        first4 = 0;
                        c42prev = c4prev;
                        c4prev  = c4; 
                        }

                }
            }
            /*
            check_str1[ccharcounter1] = '\0';
            check_str2[ccharcounter2] = '\0';
            check_str3[ccharcounter3] = '\0';
            check_str4[ccharcounter4] = '\0';
            */

///////////////////////////////////////////////////////////////////////////////////

/*
            for (uint32_t i = 0; i < check_len1; i++)
            {
                if (ccharcounter1 >= MAX_CHARS)
                    break;

                uint8_t cv1 = check_buf1[i];
                
    

                if (lut_valid2(cv1))
                {
                    char cc1 = char_lut2[cv1];
                    
                    
                  
                    if (cfirst1 && cc1 == ' ')
                        continue;

                    if (cfirst1 || cv1 != clast1 || cc1 == ' ')
                    {
                        check_str1[ccharcounter1++] = cc1;
                        clast1  = cv1;
                        cfirst1 = 0;
                        cc12prev = cc1prev;
                        cc1prev  = cc1; 
                    }

                    if (((cc12prev == 'A' && cc1prev == cc1) || (cc12prev == 'E' && cc1prev == cc1) || (cc12prev == 'I' && cc1prev == cc1) || (cc12prev == 'O' && cc1prev == cc1)|| (cc12prev == 'U' && cc1prev == cc1)) && (cc1 != 'S')){
                        check_str1[ccharcounter1++] = cc1;
                        clast1  = cv1;
                        cfirst1 = 0;
                        cc12prev = cc1prev;
                        cc1prev  = cc1; 
                    }

                    if ((cc1 == '0' || cc1 == '1' || cc1 == '2' || cc1 == '3' 
                        || cc1 == '4' || cc1 == '5' || cc1 == '6'
                        || cc1 == '7' || cc1 == '8' || cc1 == '9') 
                        && (cc12prev != 'A' && cc12prev != 'B' && cc12prev != 'C' &&
                            cc12prev != 'D' && cc12prev != 'E' && cc12prev != 'F' && 
                            cc12prev != 'G' && cc12prev != 'H' && cc12prev != 'I' &&
                            cc12prev != 'J' && cc12prev != 'K' && cc12prev != 'L' &&
                            cc12prev != 'M' && cc12prev != 'N' && cc12prev != 'O' &&
                            cc12prev != 'P' && cc12prev != 'Q' && cc12prev != 'R' &&
                            cc12prev != 'S' && cc12prev != 'T' && cc12prev != 'U' &&
                            cc12prev != 'V' && cc12prev != 'W' && cc12prev != 'X' &&
                            cc12prev != 'Y' && cc12prev != 'Z')){
                        check_str1[ccharcounter1++] = cc1;
                        clast1  = cv1;
                        cfirst1 = 0;
                        cc12prev = cc1prev;
                        cc1prev  = cc1;   
                        }  
                }
            }

            for (uint32_t i = 0; i < check_len2; i++)
            {
                if (ccharcounter2 >= MAX_CHARS)
                    break;

                
                uint8_t cv2 = check_buf2[i];
                
               
               
            

                if (lut_valid2(cv2))
                {
                    char cc2 = char_lut2[cv2];
                    
                    
                    
                    if (cfirst2 && cc2 == ' ')
                        continue;

                    if (cfirst2 || cv2 != clast2 || cc2 == ' ')
                    {
                        check_str2[ccharcounter2++] = cc2;
                        clast2  = cv2;
                        cfirst2 = 0;
                        cc22prev = cc2prev;
                        cc2prev  = cc2;   
                    }

                    if (((cc22prev == 'A' && cc2prev == cc2) || (cc22prev == 'E' && cc2prev == cc2) || (cc22prev == 'I' && cc2prev == cc2) || (cc22prev == 'O' && cc2prev == cc2) || (cc22prev == 'U' && cc2prev == cc2)) && (cc2 != 'S')){
                        check_str2[ccharcounter2++] = cc2;
                        clast2  = cv2;
                        cfirst2 = 0;
                        cc22prev = cc2prev;
                        cc2prev  = cc2;   
                    }

                    if ((cc2 == '0' || cc2 == '1' || cc2 == '2' || cc2 == '3' 
                        || cc2 == '4' || cc2 == '5' || cc2 == '6'
                        || cc2 == '7' || cc2 == '8' || cc2 == '9') 
                        && (cc22prev != 'A' && cc22prev != 'B' && cc22prev != 'C' &&
                            cc22prev != 'D' && cc22prev != 'E' && cc22prev != 'F' && 
                            cc22prev != 'G' && cc22prev != 'H' && cc22prev != 'I' &&
                            cc22prev != 'J' && cc22prev != 'K' && cc22prev != 'L' &&
                            cc22prev != 'M' && cc22prev != 'N' && cc22prev != 'O' &&
                            cc22prev != 'P' && cc22prev != 'Q' && cc22prev != 'R' &&
                            cc22prev != 'S' && cc22prev != 'T' && cc22prev != 'U' &&
                            cc22prev != 'V' && cc22prev != 'W' && cc22prev != 'X' &&
                            cc22prev != 'Y' && cc22prev != 'Z')) {
                        check_str2[ccharcounter2++] = cc2;
                        clast2  = cv2;
                        cfirst2 = 0;
                        cc22prev = cc2prev;
                        cc2prev  = cc2;       
                        }
                }
            }

            for (uint32_t i = 0; i < check_len3; i++)
            {
                if (ccharcounter3 >= MAX_CHARS)
                    break;

                uint8_t cv3 = check_buf3[i];
              
                int index3 = 0;

                if (lut_valid2(cv3))
                {
                    char cc3 = char_lut2[cv3];
                    
                    
                    
                    if (cfirst3 && cc3 == ' ')
                        continue;

                    if (cfirst3 || cv3 != clast3 || cc3 == ' ')
                    {
                        check_str3[ccharcounter3++] = cc3;
                        clast3  = cv3;
                        cfirst3 = 0;
                        cc32prev = cc3prev;
                        cc3prev  = cc3;
                    }
                    
                    if (((cc32prev == 'A' && cc3prev == cc3) || (cc32prev == 'E' && cc3prev == cc3) || (cc32prev == 'I' && cc3prev == cc3) || (cc32prev == 'O' && cc3prev == cc3) || (cc32prev == 'U' && cc3prev == cc3)) && (cc3 != 'S')){
                        check_str3[ccharcounter3++] = cc3;
                        clast3  = cv3;
                        cfirst3 = 0;
                        cc32prev = cc3prev;
                        cc3prev  = cc3;
                    }
                        

                    if ((cc3 == '0' || cc3 == '1' || cc3 == '2' || cc3 == '3' 
                        || cc3 == '4' || cc3 == '5' || cc3 == '6'
                        || cc3 == '7' || cc3 == '8' || cc3 == '9') 
                        && (cc32prev != 'A' && cc32prev != 'B' && cc32prev != 'C' &&
                            cc32prev != 'D' && cc32prev != 'E' && cc32prev != 'F' && 
                            cc32prev != 'G' && cc32prev != 'H' && cc32prev != 'I' &&
                            cc32prev != 'J' && cc32prev != 'K' && cc32prev != 'L' &&
                            cc32prev != 'M' && cc32prev != 'N' && cc32prev != 'O' &&
                            cc32prev != 'P' && cc32prev != 'Q' && cc32prev != 'R' &&
                            cc32prev != 'S' && cc32prev != 'T' && cc32prev != 'U' &&
                            cc32prev != 'V' && cc32prev != 'W' && cc32prev != 'X' &&
                            cc32prev != 'Y' && cc32prev != 'Z')){
                        check_str3[ccharcounter3++] = cc3;
                        clast3  = cv3;
                        cfirst3 = 0;
                        cc32prev = cc3prev;
                        cc3prev  = cc3;
                        }

                   

        
                }
            }

            for (uint32_t i = 0; i < check_len4; i++)
            {
                if (ccharcounter4 >= MAX_CHARS)
                    break;

               
                uint8_t cv4 = check_buf4[i];
            
               

                if (lut_valid2(cv4))
                {
                    char cc4 = char_lut2[cv4];
                    
                    
                   
                    if (cfirst4 && cc4 == ' ')
                        continue;

                    if (cfirst4 || cv4 != clast4 || cc4 == ' ')
                    {
                        check_str4[ccharcounter4++] = cc4;
                        clast4  = cv4;
                        cfirst4 = 0;
                        cc42prev = cc4prev;
                        cc4prev  = cc4; 
                    }

                    if (((cc42prev == 'A' && cc4prev == cc4) || (cc42prev == 'E' && cc4prev == cc4) || (cc42prev == 'I' && cc4prev == cc4) || (cc42prev == 'O' && cc4prev == cc4) || (cc42prev == 'U' && cc4prev == cc4)) && (cc4 != 'S')){
                        check_str4[ccharcounter4++] = cc4;
                        clast4 = cv4;
                        cfirst4 = 0;
                        cc42prev = cc4prev;
                        cc4prev  = cc4; 
                    }

                    if ((cc4 == '0' || cc4 == '1' || cc4 == '2' || cc4 == '3' 
                        || cc4 == '4' || cc4 == '5' || cc4 == '6'
                        || cc4 == '7' || cc4 == '8' || cc4 == '9') 
                        && (cc42prev != 'A' && cc42prev != 'B' && cc42prev != 'C' &&
                            cc42prev != 'D' && cc42prev != 'E' && cc42prev != 'F' && 
                            cc42prev != 'G' && cc42prev != 'H' && cc42prev != 'I' &&
                            cc42prev != 'J' && cc42prev != 'K' && cc42prev != 'L' &&
                            cc42prev != 'M' && cc42prev != 'N' && cc42prev != 'O' &&
                            cc42prev != 'P' && cc42prev != 'Q' && cc42prev != 'R' &&
                            cc42prev != 'S' && cc42prev != 'T' && cc42prev != 'U' &&
                            cc42prev != 'V' && cc42prev != 'W' && cc42prev != 'X' &&
                            cc42prev != 'Y' && cc42prev != 'Z')) {
                        check_str4[ccharcounter4++] = cc4;
                        clast4  = cv4;
                        cfirst4 = 0;
                        cc42prev = cc4prev;
                        cc4prev  = cc4; 
                        }

                }
            }

            check_str1[ccharcounter1] = '\0';
            check_str2[ccharcounter2] = '\0';
            check_str3[ccharcounter3] = '\0';
            check_str4[ccharcounter4] = '\0';

            */

            

            /* Clean-up String 1 W/O loop during debugging---------------------------------*/
            char *p1 = decoded_str1;
            while (*p1 == ' ')
                p1++;

            if (p1 != decoded_str1)
                memmove(decoded_str1, p1, strlen(p1) + 1);

            /* Ensure exactly 26 characters */
            /* Ensure exactly MAX_CHARS characters */
            uint8_t len1 = 0;
            while (decoded_str1[len1] != '\0' && len1 < MAX_CHARS)
                len1++;

            /* Pad with spaces if short */
            if (len1 < MAX_CHARS)
            {
                for (uint8_t i = len1; i < MAX_CHARS; i++)
                    decoded_str1[i] = ' ';
            }

            /* Force fixed length */
            decoded_str1[MAX_CHARS] = '\0';

            /* ---- COLLAPSE INTERNAL MULTIPLE SPACES ---- */

            /* Find last non-space character */
            int last_non_space1 = -1;

            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (decoded_str1[i] != ' ')
                {
                    last_non_space1 = i;
                    break;
                }
            }

            /* Collapse internal multiple spaces */
            if (last_non_space1 >= 0)
            {
                int write1 = 0;
                int space_seen1 = 0;

                char temp1[MAX_CHARS];

                for (int read1 = 0; read1 <= last_non_space1; read1++)
                {
                    if (decoded_str1[read1] == ' ')
                    {
                        if (!space_seen1)
                        {
                            temp1[write1++] = ' ';
                            space_seen1 = 1;
                        }
                    }
                    else
                    {
                        temp1[write1++] = decoded_str1[read1];
                        space_seen1 = 0;
                    }
                }

                /* Fill remainder with spaces */
                while (write1 < MAX_CHARS)
                    temp1[write1++] = ' ';

                /* Copy back */
                for (int i = 0; i < MAX_CHARS; i++)
                    decoded_str1[i] = temp1[i];
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
                int best_count1 = 0;
                int best_index1 = 0;

                for (uint8_t i = 0; i < CONF_DEPTH; i++)
                {
                    int count1 = 1;

                    for (uint8_t j = i + 1; j < CONF_DEPTH; j++)
                    {
                        int match1 = 1;

                        for (uint8_t k = 0; k < MAX_CHARS; k++)
                        {
                            if (history[i][k] != history[j][k])
                            {
                                match1 = 0;
                                break;
                            }
                        }

                        if (match1)
                            count1++;
                    }

                    if (count1 > best_count1)
                    {
                        best_count1 = count1;
                        best_index1 = i;
                    }
                }

                /* Output most common full row */
                uart_send_str(history[best_index1]);
                uart_send_str("\r\n");

                
            }

            /* Clean-up String 2 W/O loop during debugging---------------------------------*/
            char *p2 = decoded_str2;
            while (*p2 == ' ')
                p2++;

            if (p2 != decoded_str2)
                memmove(decoded_str2, p2, strlen(p2) + 1);

            /* Ensure exactly 26 characters */
            /* Ensure exactly MAX_CHARS characters */
            uint8_t len2 = 0;
            while (decoded_str2[len2] != '\0' && len2 < MAX_CHARS)
                len2++;

            /* Pad with spaces if short */
            if (len2 < MAX_CHARS)
            {
                for (uint8_t i = len2; i < MAX_CHARS; i++)
                    decoded_str2[i] = ' ';
            }

            /* Force fixed length */
            decoded_str2[MAX_CHARS] = '\0';

            /* ---- COLLAPSE INTERNAL MULTIPLE SPACES ---- */

            /* Find last non-space character */
            int last_non_space2 = -1;

            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (decoded_str2[i] != ' ')
                {
                    last_non_space2 = i;
                    break;
                }
            }

            /* Collapse internal multiple spaces */
            if (last_non_space2 >= 0)
            {
                int write2 = 0;
                int space_seen2 = 0;

                char temp2[MAX_CHARS];

                for (int read2 = 0; read2 <= last_non_space2; read2++)
                {
                    if (decoded_str2[read2] == ' ')
                    {
                        if (!space_seen2)
                        {
                            temp2[write2++] = ' ';
                            space_seen2 = 1;
                        }
                    }
                    else
                    {
                        temp2[write2++] = decoded_str2[read2];
                        space_seen2 = 0;
                    }
                }

                /* Fill remainder with spaces */
                while (write2 < MAX_CHARS)
                    temp2[write2++] = ' ';

                /* Copy back */
                for (int i = 0; i < MAX_CHARS; i++)
                    decoded_str2[i] = temp2[i];
            }

            /* ------------------------------------------- */

            /* Replace accidental null characters with spaces */
            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (decoded_str2[i] == '\0')
                    decoded_str2[i] = ' ';
            }

            decoded_str2[MAX_CHARS] = '\0';

            /* Store decoded frame into history */
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                history2[history_index2][i] = decoded_str2[i];

                if (decoded_str2[i] == '\0')
                    break;
            }

            history_index2++;
            if (history_index2 >= CONF_DEPTH)
                history_index2 = 0;

            if (history_count2 < CONF_DEPTH)
                history_count2++;
            if (history_count2 == CONF_DEPTH)
            {
                int best_count2 = 0;
                int best_index2 = 0;

                for (uint8_t i = 0; i < CONF_DEPTH; i++)
                {
                    int count2 = 1;

                    for (uint8_t j = i + 1; j < CONF_DEPTH; j++)
                    {
                        int match2 = 1;

                        for (uint8_t k = 0; k < MAX_CHARS; k++)
                        {
                            if (history2[i][k] != history2[j][k])
                            {
                                match2 = 0;
                                break;
                            }
                        }

                        if (match2)
                            count2++;
                    }

                    if (count2 > best_count2)
                    {
                        best_count2 = count2;
                        best_index2 = i;
                    }
                }

                /* Output most common full row */
                uart_send_str(history2[best_index2]);
                uart_send_str("\r\n");

                
            }

            /* Clean-up String 3 W/O loop during debugging-------Yes.. I know... unnecessary... debugging is easier outside loops for me------------------*/
            char *p3 = decoded_str3;
            while (*p3 == ' ')
                p3++;

            if (p3 != decoded_str3)
                memmove(decoded_str3, p3, strlen(p3) + 1);

            /* Ensure exactly 26 characters */
            /* Ensure exactly MAX_CHARS characters */
            uint8_t len3 = 0;
            while (decoded_str3[len3] != '\0' && len3 < MAX_CHARS)
                len3++;

            /* Pad with spaces if short */
            if (len3 < MAX_CHARS)
            {
                for (uint8_t i = len3; i < MAX_CHARS; i++)
                    decoded_str3[i] = ' ';
            }

            /* Force fixed length */
            decoded_str3[MAX_CHARS] = '\0';

            /* ---- COLLAPSE INTERNAL MULTIPLE SPACES ---- */

            /* Find last non-space character */
            int last_non_space3 = -1;

            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (decoded_str3[i] != ' ')
                {
                    last_non_space3 = i;
                    break;
                }
            }

            /* Collapse internal multiple spaces */
            if (last_non_space3 >= 0)
            {
                int write3 = 0;
                int space_seen3 = 0;

                char temp3[MAX_CHARS];

                for (int read3 = 0; read3 <= last_non_space3; read3++)
                {
                    if (decoded_str3[read3] == ' ')
                    {
                        if (!space_seen3)
                        {
                            temp3[write3++] = ' ';
                            space_seen3 = 1;
                        }
                    }
                    else
                    {
                        temp3[write3++] = decoded_str3[read3];
                        space_seen3 = 0;
                    }
                }

                /* Fill remainder with spaces */
                while (write3 < MAX_CHARS)
                    temp3[write3++] = ' ';

                /* Copy back */
                for (int i = 0; i < MAX_CHARS; i++)
                    decoded_str3[i] = temp3[i];
            }

            /* ------------------------------------------- */

            /* Replace accidental null characters with spaces */
            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (decoded_str3[i] == '\0')
                    decoded_str3[i] = ' ';
            }

            decoded_str3[MAX_CHARS] = '\0';

            /* Store decoded frame into history */
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                history3[history_index3][i] = decoded_str3[i];

                if (decoded_str3[i] == '\0')
                    break;
            }

            history_index3++;
            if (history_index3 >= CONF_DEPTH)
                history_index3 = 0;

            if (history_count3 < CONF_DEPTH)
                history_count3++;
            if (history_count3 == CONF_DEPTH)
            {
                int best_count3 = 0;
                int best_index3 = 0;

                for (uint8_t i = 0; i < CONF_DEPTH; i++)
                {
                    int count3 = 1;

                    for (uint8_t j = i + 1; j < CONF_DEPTH; j++)
                    {
                        int match3 = 1;

                        for (uint8_t k = 0; k < MAX_CHARS; k++)
                        {
                            if (history3[i][k] != history3[j][k])
                            {
                                match3 = 0;
                                break;
                            }
                        }

                        if (match3)
                            count3++;
                    }

                    if (count3 > best_count3)
                    {
                        best_count3 = count3;
                        best_index3 = i;
                    }
                }

                /* Output most common full row */
                uart_send_str(history3[best_index3]);
                uart_send_str("\r\n");

                
            }

            /* Clean-up String 4 W/O loop during debugging---------------------------------*/
            char *p4 = decoded_str4;
            while (*p4 == ' ')
                p4++;

            if (p4 != decoded_str4)
                memmove(decoded_str4, p4, strlen(p4) + 1);

            /* Ensure exactly 26 characters */
            /* Ensure exactly MAX_CHARS characters */
            uint8_t len4 = 0;
            while (decoded_str4[len4] != '\0' && len4 < MAX_CHARS)
                len4++;

            /* Pad with spaces if short */
            if (len4 < MAX_CHARS)
            {
                for (uint8_t i = len4; i < MAX_CHARS; i++)
                    decoded_str4[i] = ' ';
            }

            /* Force fixed length */
            decoded_str4[MAX_CHARS] = '\0';

            /* ---- COLLAPSE INTERNAL MULTIPLE SPACES ---- */

            /* Find last non-space character */
            int last_non_space4 = -1;

            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (decoded_str4[i] != ' ')
                {
                    last_non_space4 = i;
                    break;
                }
            }

            /* Collapse internal multiple spaces */
            if (last_non_space4 >= 0)
            {
                int write4 = 0;
                int space_seen4 = 0;

                char temp4[MAX_CHARS];

                for (int read4 = 0; read4 <= last_non_space4; read4++)
                {
                    if (decoded_str4[read4] == ' ')
                    {
                        if (!space_seen4)
                        {
                            temp4[write4++] = ' ';
                            space_seen4 = 1;
                        }
                    }
                    else
                    {
                        temp4[write4++] = decoded_str4[read4];
                        space_seen4 = 0;
                    }
                }

                /* Fill remainder with spaces */
                while (write4 < MAX_CHARS)
                    temp4[write4++] = ' ';

                /* Copy back */
                for (int i = 0; i < MAX_CHARS; i++)
                    decoded_str4[i] = temp4[i];
            }

            /* ------------------------------------------- */

            /* Replace accidental null characters with spaces */
            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (decoded_str4[i] == '\0')
                    decoded_str4[i] = ' ';
            }

            decoded_str4[MAX_CHARS] = '\0';

            /* Store decoded frame into history */
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                history4[history_index4][i] = decoded_str4[i];

                if (decoded_str4[i] == '\0')
                    break;
            }

            history_index4++;
            if (history_index4 >= CONF_DEPTH)
                history_index4 = 0;

            if (history_count4 < CONF_DEPTH)
                history_count4++;
            if (history_count4 == CONF_DEPTH)
            {
                int best_count4 = 0;
                int best_index4 = 0;

                for (uint8_t i = 0; i < CONF_DEPTH; i++)
                {
                    int count4 = 1;

                    for (uint8_t j = i + 1; j < CONF_DEPTH; j++)
                    {
                        int match4 = 1;

                        for (uint8_t k = 0; k < MAX_CHARS; k++)
                        {
                            if (history4[i][k] != history4[j][k])
                            {
                                match4 = 0;
                                break;
                            }
                        }

                        if (match4)
                            count4++;
                    }

                    if (count4 > best_count4)
                    {
                        best_count4 = count4;
                        best_index4 = i;
                    }
                }

                /* Output most common full row */
                uart_send_str(history4[best_index4]);
                uart_send_str("\r\n");

                
            }

            /* ================= CLEANUP + CONFIDENCE : CHECK STRING 1 ================= */

            /*
            char *cp1 = check_str1;
            while (*cp1 == ' ')
                cp1++;

            if (cp1 != check_str1)
                memmove(check_str1, cp1, strlen(cp1) + 1);

            uint8_t clen1 = 0;
            while (check_str1[clen1] != '\0' && clen1 < MAX_CHARS)
                clen1++;

            if (clen1 < MAX_CHARS)
            {
                for (uint8_t i = clen1; i < MAX_CHARS; i++)
                    check_str1[i] = ' ';
            }

            check_str1[MAX_CHARS] = '\0';

           
            int clast_non_space1 = -1;
            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (check_str1[i] != ' ')
                {
                    clast_non_space1 = i;
                    break;
                }
            }

            if (clast_non_space1 >= 0)
            {
                int write = 0;
                int space_seen = 0;
                char temp[MAX_CHARS];

                for (int read = 0; read <= clast_non_space1; read++)
                {
                    if (check_str1[read] == ' ')
                    {
                        if (!space_seen)
                        {
                            temp[write++] = ' ';
                            space_seen = 1;
                        }
                    }
                    else
                    {
                        temp[write++] = check_str1[read];
                        space_seen = 0;
                    }
                }

                while (write < MAX_CHARS)
                    temp[write++] = ' ';

                for (int i = 0; i < MAX_CHARS; i++)
                    check_str1[i] = temp[i];
            }

            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (check_str1[i] == '\0')
                    check_str1[i] = ' ';
            }

            check_str1[MAX_CHARS] = '\0';

           
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                check_history1[check_history_index1][i] = check_str1[i];
                if (check_str1[i] == '\0')
                    break;
            }

            check_history_index1++;
            if (check_history_index1 >= CONF_DEPTH)
                check_history_index1 = 0;

            if (check_history_count1 < CONF_DEPTH)
                check_history_count1++;

            if (check_history_count1 == CONF_DEPTH)
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
                            if (check_history1[i][k] != check_history1[j][k])
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

                uart_send_str(check_history1[best_index]);
                uart_send_str("\r\n");
            }

            

            char *cp2 = check_str2;
            while (*cp2 == ' ')
                cp2++;

            if (cp2 != check_str2)
                memmove(check_str2, cp2, strlen(cp2) + 1);

            uint8_t clen2 = 0;
            while (check_str2[clen2] != '\0' && clen2 < MAX_CHARS)
                clen2++;

            if (clen2 < MAX_CHARS)
            {
                for (uint8_t i = clen2; i < MAX_CHARS; i++)
                    check_str2[i] = ' ';
            }

            check_str2[MAX_CHARS] = '\0';

            int clast_non_space2 = -1;
            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (check_str2[i] != ' ')
                {
                    clast_non_space2 = i;
                    break;
                }
            }

            if (clast_non_space2 >= 0)
            {
                int write = 0;
                int space_seen = 0;
                char temp[MAX_CHARS];

                for (int read = 0; read <= clast_non_space2; read++)
                {
                    if (check_str2[read] == ' ')
                    {
                        if (!space_seen)
                        {
                            temp[write++] = ' ';
                            space_seen = 1;
                        }
                    }
                    else
                    {
                        temp[write++] = check_str2[read];
                        space_seen = 0;
                    }
                }

                while (write < MAX_CHARS)
                    temp[write++] = ' ';

                for (int i = 0; i < MAX_CHARS; i++)
                    check_str2[i] = temp[i];
            }

            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (check_str2[i] == '\0')
                    check_str2[i] = ' ';
            }

            check_str2[MAX_CHARS] = '\0';

            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                check_history2[check_history_index2][i] = check_str2[i];
                if (check_str2[i] == '\0')
                    break;
            }

            check_history_index2++;
            if (check_history_index2 >= CONF_DEPTH)
                check_history_index2 = 0;

            if (check_history_count2 < CONF_DEPTH)
                check_history_count2++;

            if (check_history_count2 == CONF_DEPTH)
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
                            if (check_history2[i][k] != check_history2[j][k])
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

                uart_send_str(check_history2[best_index]);
                uart_send_str("\r\n");
            }

            

            char *cp3 = check_str3;
            while (*cp3 == ' ')
                cp3++;

            if (cp3 != check_str3)
                memmove(check_str3, cp3, strlen(cp3) + 1);

            uint8_t clen3 = 0;
            while (check_str3[clen3] != '\0' && clen3 < MAX_CHARS)
                clen3++;

            if (clen3 < MAX_CHARS)
            {
                for (uint8_t i = clen3; i < MAX_CHARS; i++)
                    check_str3[i] = ' ';
            }

            check_str3[MAX_CHARS] = '\0';

            
            int clast_non_space3 = -1;
            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (check_str3[i] != ' ')
                {
                    clast_non_space3 = i;
                    break;
                }
            }

            if (clast_non_space3 >= 0)
            {
                int write = 0;
                int space_seen = 0;
                char temp[MAX_CHARS];

                for (int read = 0; read <= clast_non_space3; read++)
                {
                    if (check_str3[read] == ' ')
                    {
                        if (!space_seen)
                        {
                            temp[write++] = ' ';
                            space_seen = 1;
                        }
                    }
                    else
                    {
                        temp[write++] = check_str3[read];
                        space_seen = 0;
                    }
                }

                while (write < MAX_CHARS)
                    temp[write++] = ' ';

                for (int i = 0; i < MAX_CHARS; i++)
                    check_str3[i] = temp[i];
            }

            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (check_str3[i] == '\0')
                    check_str3[i] = ' ';
            }

            check_str3[MAX_CHARS] = '\0';

            /* Store in check_history3 */
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                check_history3[check_history_index3][i] = check_str3[i];
                if (check_str3[i] == '\0')
                    break;
            }

            check_history_index3++;
            if (check_history_index3 >= CONF_DEPTH)
                check_history_index3 = 0;

            if (check_history_count3 < CONF_DEPTH)
                check_history_count3++;

            if (check_history_count3 == CONF_DEPTH)
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
                            if (check_history3[i][k] != check_history3[j][k])
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

                uart_send_str(check_history3[best_index]);
                uart_send_str("\r\n");
            }

            

            char *cp4 = check_str4;
            while (*cp4 == ' ')
                cp4++;

            if (cp4 != check_str4)
                memmove(check_str4, cp4, strlen(cp4) + 1);

            uint8_t clen4 = 0;
            while (check_str4[clen4] != '\0' && clen4 < MAX_CHARS)
                clen4++;

            if (clen4 < MAX_CHARS)
            {
                for (uint8_t i = clen4; i < MAX_CHARS; i++)
                    check_str4[i] = ' ';
            }

            check_str4[MAX_CHARS] = '\0';

            int clast_non_space4 = -1;
            for (int i = MAX_CHARS - 1; i >= 0; i--)
            {
                if (check_str4[i] != ' ')
                {
                    clast_non_space4 = i;
                    break;
                }
            }

            if (clast_non_space4 >= 0)
            {
                int write = 0;
                int space_seen = 0;
                char temp[MAX_CHARS];

                for (int read = 0; read <= clast_non_space4; read++)
                {
                    if (check_str4[read] == ' ')
                    {
                        if (!space_seen)
                        {
                            temp[write++] = ' ';
                            space_seen = 1;
                        }
                    }
                    else
                    {
                        temp[write++] = check_str4[read];
                        space_seen = 0;
                    }
                }

                while (write < MAX_CHARS)
                    temp[write++] = ' ';

                for (int i = 0; i < MAX_CHARS; i++)
                    check_str4[i] = temp[i];
            }

            for (uint8_t i = 0; i < MAX_CHARS; i++)
            {
                if (check_str4[i] == '\0')
                    check_str4[i] = ' ';
            }

            check_str4[MAX_CHARS] = '\0';

            /* Store in check_history4 */
            for (uint8_t i = 0; i <= MAX_CHARS; i++)
            {
                check_history4[check_history_index4][i] = check_str4[i];
                if (check_str4[i] == '\0')
                    break;
            }

            check_history_index4++;
            if (check_history_index4 >= CONF_DEPTH)
                check_history_index4 = 0;

            if (check_history_count4 < CONF_DEPTH)
                check_history_count4++;

            if (check_history_count4 == CONF_DEPTH)
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
                            if (check_history4[i][k] != check_history4[j][k])
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

                uart_send_str(check_history4[best_index]);
                uart_send_str("\r\n"); 
                */
            }

            //build_and_print_payload(decoded_str1);

            //uart_send_str(decoded_str1);
            //uart_send_str("\r\n");

#endif
        }

        prev_vsync = vsync;
    }
}
