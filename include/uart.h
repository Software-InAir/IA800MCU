#ifndef UART_H
#define UART_H

#include <stdint.h>

/* ================== INIT ================== */

// Initializes GPIO + USART
void uart_gpio_init(void);

/* ================== TX ================== */

// Send a single character
void uart_send_char(char c);

// Send null-terminated string
void uart_send_str(const char *s);

// Send byte as hex (e.g., "AF ")
void uart_send_hex(uint8_t v);

/* ================== RX ================== */

// Blocking receive (waits for one byte)
char uart_getc(void);

// Non-blocking receive
// Returns 1 if data available, 0 otherwise
int uart_available(void);

/* ================== UTIL ================== */

// CRC16 (Modbus)
uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

#endif