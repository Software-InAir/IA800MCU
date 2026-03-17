#include "stm32h723xx.h"
#include "uart.h"
#include "config.h"

/* ================== UART ================== */

void uart_gpio_init(void)
{
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
}

void uart_send_char(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = c;
}

void uart_send_hex(uint8_t v)
{
    const char hex[] = "0123456789ABCDEF";
    uart_send_char(hex[(v >> 4) & 0x0F]);
    uart_send_char(hex[v & 0x0F]);
    uart_send_char(' ');
}

void uart_send_str(const char *s)
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