#include "stm32h723xx.h"
#include <string.h>

/* ================== CONFIG ================== */
#define CONF_DEPTH 8
#define BUF_SIZE  4096
#define OUTPUT_RAW_HEX  0   /* 0 = decoded, 1 = raw hex */
#define MAX_CHARS 26
#define ATR_MSG_COUNT (sizeof(atr)/sizeof(atr[0]))
#define BAE_MSG_COUNT (sizeof(bae)/sizeof(bae[0]))
#define DHC_MSG_COUNT (sizeof(dhc)/sizeof(dhc[0]))
#define CITATION_MSG_COUNT (sizeof(citation)/sizeof(citation[0]))

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


/* ================ LTDC SETUP ================== */

// Clocks

static void ltdc_setup(void)
{ 
    RCC->AHB4

}




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
    [0x00] = ' '
};

static const char *atr[256] = {
    [0] = "AP/YD DISENGAGED",
    [1] = "AP DISENGAGED",
    [2] = "YD DISENGAGED",
    [3] = "AILERON MISTRIM",
    [4] = "PITCH TRIM FAIL",
    [5] = "PITCH MISTRIM",
    [6] = "RETRIM ROLL R WING DN",
    [7] = "RETRIM ROLL L WING DN",
    [8] = "PITCH MISTRIM NOSE UP",
    [9] = "PITCH MISTRIM NOSE DOWN",
    [10] = "DISENGAGE ANNUN DATA FAULT",
    [11] = "ENGAGE INHIBIT",
    [12] = "NO ENGAGEMENT ON GROUND",
    [13] = "NO WOW",
    [14] = "IAS TOO HIGH",
    [15]= "CPL DATA INVALID",
    [16] = "AHRS DATA INVALID",
    [17] = "DADC DATA INVALID",
    [18] = "AFCS INVALID",
    [19] = "AP INVALID",
    [20] = "CPL DATA INVALID",
    [21] = "AHRS DATA INVALID",
    [22] = "DADC DATA INVALID",
    [23] = "AFCS INVALID",
    [24] = "AP INVALID",
    [25] = "NAV MISMATCH L SEL",
    [26] = "NAV MISMATCH R SEL",
    [27] = "EXCESS DEV",
    [28] = "CAT 2 INVALID",
    [29] = "CHECK NAV SOURCE",
    [30] = "NO ENGAGEMENT ON GROUND",
    [31] = "ASCB TEST FCS1 A",
    [32] = "ASCB TEST FCS1 B",
    [33] = "ASCB TEST FCS1 C",
    [34] = "ASCB TEST FCS1 D",
    [35] = "ASCB TEST FCS2 A",
    [36] = "ASCB TEST FCS2 B",
    };

static const char *bae[256] = {
    [0] = "TCS ENGAGE",
    [1] = "HSI SELECT >",
    [2] = "< HSI SELECT",
    [3] = "< HSI SELECT >",
    [4] = "L (R) AP/YD ENG",
    [5] = "L (R) YD ENG",
    [6] = "SYSTEM TEST",
    [7] = "L AFCS MASTER",
    [8] = "R AFCS MASTER",
    [9] = "SELECT INHIBIT",
    [10] = "L (R) AFCS FAIL",
    [11] = "AP SERVOS FAIL",
    [12] = "ENGAGE INHIBIT",
    [13] = "DADC INVLD",
    [14] = "AHRS INVLD",
    [15] = "ACFT ON GND",
    [16] = "NO GND TST",
    [17] = "LOW BANK",
    [18] = "AP/YD DISENGAGE",
    [19] = "AP DISENGAGE",
    [20] = "YD DISENGAGE",
    [21] = "PITCH RETRIM NOSE UP",
    [22] = "PITCH RETRIM NOSE DN",
    [23] = "PITCH TRIM FAIL",
    [24] = "RETRIM ROLL R WING DN",
    [25] = "RETRIM ROLL L WING DN",
    [26] = "EXCESSIVE DEV",
    [27] = "DISENGAGE ANNUN",
    [28] = "DATA FAULT",
    [29] = "NO WOW",
    [30] = "IAS HIGH",
    [31] = "HSI SEL DATA INVALID",
    [32] = "AHRS DATA INVALID",
    [33] = "DADC DATA INVALID",
    [34] = "NAV MISMATCH [L SEL]",
    [35] = "NAV MISMATCH [R SEL]",
    [36] = "CHECK NAV SOURCE",
    [37] = "L (R) AFCS FAIL",
    [38] = "ALT OFF",
    [39] = "INVALID OPERATION"
};

static const char *dhc[256] = {
    [0] = "HDG HOLD",
    [1] = "PITCH HOLD",
    [2] = "HDG SEL",
    [3] = "ALT",
    [4] = "WINGS LEVEL",
    [5] = "AP FAIL/YD AVAIL",
    [6] = "AHRS DATA INVLD",
    [7] = "DADC DATA INVLD",
    [8] = "AP DISENGAGED",
    [9] = "AP/YD DISENGAGED",
    [10] = "YD DISENGAGED",
    [11] = "L AP/YD FAIL",
    [12] = "R AP/YD FAIL",
    [13] = "MISTRIM [TRIM L WING DN]",
    [14] = "MISTRIM [TRIM R WING DN]",
    [15] = "MISTRIM [TRIM NOSE UP]",
    [16] = "MISTRIM [TRIM NOSE DN]",
    [17] = "ADI PITCH/ROLL MISMATCH",
    [18] = "ADI PITCH MISMATCH",
    [19] = "ADI ROLL MISMATCH",
    [20] = "HSI HDG MISMATCH",
    [21] = "FD NAV MISMATCH R VALID",
    [22] = "FD NAV MISMATCH L VALID"
};

static const char *citation[256] = {
    [0] = "HSI SEL >",
    [1] = "< HSI SEL",
    [2] = "< HSI SEL >",
    [3] = "A AFCS MASTER",
    [4] = "B AFCS MASTER",
    [5] = "AP/YD ENGAGE",
    [6] = "YD ENGAGED",
    [7] = "LOW BANK",
    [8] = "AP/YD DISENGAGE",
    [9] = "AP DISENGAGE",
    [10] = "YD DISENGAGE",
    [11] = "ELEV MISTRIM NOSE UP",
    [12] = "ELEV MISTRM NOSE DN",
    [13] = "ELEV TRIM FAIL",
    [14] = "RETRIM ROLL R WING DOWN",
    [15] = "RETRIM ROLL L WING DOWN",
    [16] = "DISENGAGE ANNUN",
    [17] = "DATA FAULT",
    [18] = "HSI DATA INVLD",
    [19] = "AHRS INVALID",
    [20] = "DADC INVALID",
    [21] = "NAV MISMATCH [L SEL]",
    [22] = "NAV MISMATCH [R SEL]",
    [23] = "CHECK NAV SOURCE",
    [24] = "AFCS FAIL",
    [25] = "ALT OFF"
};

static inline int lut_valid1(uint8_t v)
{
    return char_lut[v] != 0;
}

static inline int lut_valid2(uint8_t v)
{
    return char_lut2[v] != 0;
}

int score_match(const char *a, const char *b)
{
    int score = 0;

    for (int i = 0; a[i] && b[i]; i++)
    {
        if (a[i] == b[i])
            score++;
    }

    return score;
}

void best_in_lut(
    const char *decoded,
    const char **lut,
    int lut_size,
    int *best_index,
    int *best_score)
{
    for (int i = 0; i < lut_size; i++)
    {
        if (!lut[i])
            continue;   // skip empty LUT entries

        int s = score_match(decoded, lut[i]);

        if (s > *best_score)
        {
            *best_score = s;
            *best_index = i;
        }
    }
}

typedef enum
{
    AIRFRAME_NONE = 0,
    AIRFRAME_ATR,
    AIRFRAME_BAE,
    AIRFRAME_DHC,
    AIRFRAME_CITATION
} airframe_t;

void classify_message(
    const char *decoded,
    airframe_t *airframe,
    int *msg_index)
{
    int best_score = 0;
    int best_idx = -1;

    int idx = -1;
    int score = 0;

    *airframe = AIRFRAME_NONE;
    *msg_index = -1;    

    /* ATR */
    idx = -1; score = 0;
    best_in_lut(decoded, atr, ATR_MSG_COUNT, &idx, &score);
    if (score > best_score)
    {
        best_score = score;
        best_idx = idx;
        *airframe = AIRFRAME_ATR;
    }

    /* BAE */
    idx = -1; score = 0;
    best_in_lut(decoded, bae, BAE_MSG_COUNT, &idx, &score);
    if (score > best_score)
    {
        best_score = score;
        best_idx = idx;
        *airframe = AIRFRAME_BAE;
    }

    /* DHC */
    idx = -1; score = 0;
    best_in_lut(decoded, dhc, DHC_MSG_COUNT, &idx, &score);
    if (score > best_score)
    {
        best_score = score;
        best_idx = idx;
        *airframe = AIRFRAME_DHC;
    }

    /* CITATION */
    idx = -1; score = 0;
    best_in_lut(decoded, citation, CITATION_MSG_COUNT, &idx, &score);
    if (score > best_score)
    {
        best_score = score;
        best_idx = idx;
        *airframe = AIRFRAME_CITATION;
    }

    if (best_score < 6)
    {
        *airframe = AIRFRAME_NONE;
        *msg_index = -1;
        return;
    }

    *msg_index = best_idx;
}

/* ================== MAIN ================== */
int main(void)
{
    /////////////////////////////////////////////////////////// CLOCK setup
    /* Enable clocks */
    RCC->AHB4ENR  |= RCC_AHB4ENR_GPIOAEN |
                     RCC_AHB4ENR_GPIOBEN |
                     RCC_AHB4ENR_GPIOCEN;

    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /////////////////////////////////////////////////////////// GPIO setup

    // GPIO Pins (Blue channel)
    GPIOG->MODER &= ~(3 << (14 * 2));  ////// Blue 0
    GPIOG->MODER |=  (2 << (14 * 2));
    GPIOG->OSPEEDR &= ~(3 << (14 * 2));
    GPIOG->OSPEEDR |= (3 << (14 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((14 - 8) * 4)); 
    GPIOG->AFR[1] |=  (0xE << ((14 - 8) * 4));

    GPIOA->MODER &= ~(3 << (10 * 2));   /// Blue 1
    GPIOA->MODER |=  (2 << (10 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (10 * 2));
    GPIOA->OSPEEDR |= (3 << (10 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xE << ((10 - 8) * 4));

    GPIOC->MODER &= ~(3 << (9 * 2));   //// Blue 2
    GPIOC->MODER |=  (2 << (9 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (9 * 2));
    GPIOC->OSPEEDR |= (3 << (9 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((9 - 8) * 4)); 
    GPIOC->AFR[1] |=  (0xE << ((9 - 8) * 4)); 

    GPIOA->MODER &= ~(3 << (8 * 2));   //// Blue 3
    GPIOA->MODER |=  (2 << (8 * 2));
    GPIOA->OSPEEDR &= ~(3 << (8 * 2));
    GPIOA->OSPEEDR |= (3 << (8 * 2)); 
    GPIOA->AFR[1] &= ~(0xF << ((8 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xD << ((8 - 8) * 4)); 

    GPIOC->MODER &= ~(3 << (11 * 2));   /// Blue 4
    GPIOC->MODER |=  (2 << (11 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (11 * 2));
    GPIOC->OSPEEDR |= (3 << (11 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((11 - 8) * 4)); 
    GPIOC->AFR[1] |=  (0xE << ((11 - 8) * 4)); 

    GPIOA->MODER &= ~(3 << (3 * 2));  ///// Blue 5
    GPIOA->MODER |=  (2 << (3 * 2));
    GPIOA->OSPEEDR &= ~(3 << (3 * 2));
    GPIOA->OSPEEDR |= (3 << (3 * 2)); 
    GPIOA->AFR[0] &= ~(0xF << ((3) * 4)); 
    GPIOA->AFR[0] |=  (0xE << ((3) * 4)); 


    // GPIO Pins (Red channel)

    GPIOG->MODER &= ~(3 << (13 * 2));  ////// Red 0
    GPIOG->MODER |=  (2 << (13 * 2));
    GPIOG->OSPEEDR &= ~(3 << (13 * 2));
    GPIOG->OSPEEDR |= (3 << (13 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((13 - 8) * 4)); 
    GPIOG->AFR[1] |=  (0xE << ((13 - 8) * 4));

    GPIOA->MODER &= ~(3 << (2 * 2));  ////// Red 1
    GPIOA->MODER |=  (2 << (2 * 2));
    GPIOA->OSPEEDR &= ~(3 << (2 * 2));
    GPIOA->OSPEEDR |= (3 << (2 * 2));
    GPIOA->AFR[0] &= ~(0xF << ((2) * 4)); 
    GPIOA->AFR[0] |=  (0xD << ((2) * 4));

    GPIOC->MODER &= ~(3 << (10 * 2));  ///// Red 2
    GPIOC->MODER |=  (2 << (10 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (10 * 2));
    GPIOC->OSPEEDR |= (3 << (10 * 2));
    GPIOC->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOC->AFR[1] |=  (0xE << ((10 - 8) * 4));

    GPIOA->MODER &= ~(3 << (15 * 2));   //// Red 3
    GPIOA->MODER |=  (2 << (15 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (15 * 2));
    GPIOA->OSPEEDR |= (3 << (15 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0x9 << ((15 - 8) * 4));

    GPIOA->MODER &= ~(3 << (11 * 2));  //// Red 4
    GPIOA->MODER |=  (2 << (11 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (11 * 2));
    GPIOA->OSPEEDR |= (3 << (11 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((11 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xE << ((11 - 8) * 4));

    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |=  (2 << (9 * 2));
    GPIOA->OSPEEDR &= ~(3 << (9 * 2));
    GPIOA->OSPEEDR |= (3 << (9 * 2)); 
    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4)); 
    GPIOA->AFR[1] |=  (0xE << ((9 - 8) * 4));


    // GPIO Pins (Green channel)

    GPIOB->MODER &= ~(3 << (0 * 2));   //// Green 1
    GPIOB->MODER |=  (2 << (0 * 2));
    GPIOB->OSPEEDR &= ~(3 << (0 * 2));
    GPIOB->OSPEEDR |= (3 << (0 * 2));
    GPIOB->AFR[0] &= ~(0xF << ((0) * 4)); 
    GPIOB->AFR[0] |=  (0xE << ((0) * 4));

    GPIOB->MODER &= ~(3 << (1 * 2));  //// Green 0
    GPIOB->MODER |=  (2 << (1 * 2));
    GPIOB->OSPEEDR &= ~(3 << (1 * 2));
    GPIOB->OSPEEDR |= (3 << (1 * 2));
    GPIOB->AFR[0] &= ~(0xF << ((1) * 4)); 
    GPIOB->AFR[0] |=  (0xE << ((1) * 4));

    GPIOA->MODER &= ~(3 << (6 * 2)); //// Green 2
    GPIOA->MODER |=  (2 << (6 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (6 * 2));
    GPIOA->OSPEEDR |= (3 << (6 * 2));
    GPIOA->AFR[0] &= ~(0xF << ((6) * 4)); 
    GPIOA->AFR[0] |=  (0xE << ((6) * 4));

    GPIOG->MODER &= ~(3 << (10 * 2));  /// Green 3
    GPIOG->MODER |=  (2 << (10 * 2));
    GPIOG->OSPEEDR &= ~(3 << (10 * 2));
    GPIOG->OSPEEDR |= (3 << (10 * 2));
    GPIOG->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOG->AFR[1] |=  (0x9 << ((10 - 8) * 4)); 

    GPIOB->MODER &= ~(3 << (10 * 2)); /// Green 4
    GPIOB->MODER |=  (2 << (10 * 2)); 
    GPIOB->OSPEEDR &= ~(3 << (10 * 2));
    GPIOB->OSPEEDR |= (3 << (10 * 2));
    GPIOB->AFR[1] &= ~(0xF << ((10 - 8) * 4)); 
    GPIOB->AFR[1] |=  (0xE << ((10 - 8) * 4));

    GPIOB->MODER &= ~(3 << (11 * 2)); // Green 5
    GPIOB->MODER |=  (2 << (11 * 2)); // AF MODE
    GPIOB->OSPEEDR &= ~(3 << (11 * 2));
    GPIOB->OSPEEDR |= (3 << (11 * 2)); // HIGH SPEED
    GPIOB->AFR[1] &= ~(0xF << ((11 - 8) * 4)); // Alternate Function clear
    GPIOB->AFR[1] |=  (0xE << ((11 - 8) * 4)); // Alternate Functoon AF13 (LTDC)

    ////////////////////////////////////////////////// LTDC Clock Signals

    GPIOC->MODER &= ~(3 << (6 * 2));  //// HSYNC
    GPIOC->MODER |=  (2 << (6 * 2));
    GPIOC->OSPEEDR &= ~(3 << (6 * 2));
    GPIOC->OSPEEDR |= (3 << (6 * 2));
    GPIOC->AFR[0] &= ~(0xF << ((6) * 4)); 
    GPIOC->AFR[0] |=  (0xE << ((6) * 4));

    GPIOA->MODER &= ~(3 << (4 * 2)); //// VSYNC
    GPIOA->MODER |=  (2 << (4 * 2)); 
    GPIOA->OSPEEDR &= ~(3 << (4 * 2));
    GPIOA->OSPEEDR |= (3 << (4 * 2));
    GPIOA->AFR[0] &= ~(0xF << ((4) * 4)); 
    GPIOA->AFR[0] |=  (0xE << ((4) * 4));

    GPIOB->MODER &= ~(3 << (14 * 2));  /// LCD_CLK
    GPIOB->MODER |=  (2 << (14 * 2));
    GPIOB->OSPEEDR &= ~(3 << (14 * 2));
    GPIOB->OSPEEDR |= (3 << (14 * 2));
    GPIOB->AFR[1] &= ~(0xF << ((14 - 8) * 4)); 
    GPIOB->AFR[1] |=  (0xE << ((14 - 8) * 4)); 

    GPIOC->MODER &= ~(3 << (5 * 2)); /// LCD_DE
    GPIOC->MODER |=  (2 << (5 * 2)); 
    GPIOC->OSPEEDR &= ~(3 << (5 * 2));
    GPIOC->OSPEEDR |= (3 << (5 * 2));
    GPIOC->AFR[0] &= ~(0xF << ((5) * 4)); 
    GPIOC->AFR[0] |=  (0xE << ((5) * 4));

 
    // Set PA9 to Alternate Function mode (USART1_TX)
    GPIOA->MODER &= ~(3 << (9 * 2));
    GPIOA->MODER |=  (2 << (9 * 2));      // AF mode

    GPIOA->AFR[1] &= ~(0xF << ((9 - 8) * 4)); // Clear AF bits for PA9
    GPIOA->AFR[1] |=  (1 << ((9 - 8) * 4));   // AF1 = USART1

    GPIOA->MODER &= ~(3 << (10 * 2));
    GPIOA->MODER |=  (2 << (10 * 2));  

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

               

                airframe_t af;
                int msg;

                classify_message(history[best_index1], &af, &msg);

                uart_send_str("R1 AF:");
                uart_send_hex(af);

                uart_send_str(" MSG:");
                uart_send_hex(msg);

                uart_send_str(" ");

                const char *out = NULL;

                switch (af)
                {
                    case AIRFRAME_ATR:      out = atr[msg]; break;
                    case AIRFRAME_BAE:      out = bae[msg]; break;
                    case AIRFRAME_DHC:      out = dhc[msg]; break;
                    case AIRFRAME_CITATION: out = citation[msg]; break;
                    default: break;
                }

                if (out)
                    uart_send_str(out);
                else
                    uart_send_str("UNKNOWN");

                uart_send_str("\r\n");

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
                airframe_t af;
                int msg;

                classify_message(history2[best_index2], &af, &msg);

                uart_send_str("R2 AF:");
                uart_send_hex(af);

                uart_send_str(" MSG:");
                uart_send_hex(msg);

                uart_send_str(" ");

                const char *out = NULL;

                switch (af)
                {
                    case AIRFRAME_ATR:      out = atr[msg]; break;
                    case AIRFRAME_BAE:      out = bae[msg]; break;
                    case AIRFRAME_DHC:      out = dhc[msg]; break;
                    case AIRFRAME_CITATION: out = citation[msg]; break;
                    default: break;
                }

                if (out)
                    uart_send_str(out);
                else
                    uart_send_str("UNKNOWN");

                uart_send_str("\r\n");

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
                airframe_t af;
                int msg;

                classify_message(history3[best_index3], &af, &msg);

                uart_send_str("R3 AF:");
                uart_send_hex(af);

                uart_send_str(" MSG:");
                uart_send_hex(msg);

                uart_send_str(" ");

                const char *out = NULL;

                switch (af)
                {
                    case AIRFRAME_ATR:      out = atr[msg]; break;
                    case AIRFRAME_BAE:      out = bae[msg]; break;
                    case AIRFRAME_DHC:      out = dhc[msg]; break;
                    case AIRFRAME_CITATION: out = citation[msg]; break;
                    default: break;
                }

                if (out)
                    uart_send_str(out);
                else
                    uart_send_str("UNKNOWN");

                uart_send_str("\r\n");

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
                airframe_t af;
                int msg;

                classify_message(history4[best_index4], &af, &msg);

                uart_send_str("R4 AF:");
                uart_send_hex(af);

                uart_send_str(" MSG:");
                uart_send_hex(msg);

                uart_send_str(" ");

                const char *out = NULL;

                switch (af)
                {
                    case AIRFRAME_ATR:      out = atr[msg]; break;
                    case AIRFRAME_BAE:      out = bae[msg]; break;
                    case AIRFRAME_DHC:      out = dhc[msg]; break;
                    case AIRFRAME_CITATION: out = citation[msg]; break;
                    default: break;
                }

                if (out)
                    uart_send_str(out);
                else
                    uart_send_str("UNKNOWN");

                uart_send_str("\r\n");

                uart_send_str(history4[best_index4]);
                uart_send_str("\r\n");
            }

          

            /* Store in check_history3 ;*/
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
                
            }

            //build_and_print_payload(decoded_str1);

            //uart_send_str(decoded_str1);
            //uart_send_str("\r\n");

#endif
        }

        prev_vsync = vsync;
    }
}
