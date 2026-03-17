#ifndef CAPTURE_H
#define CAPTURE_H

#include <stdint.h>

#define ROWS 4
#define BUF_SIZE 4096

extern volatile uint8_t  capture_buf[ROWS][BUF_SIZE];
extern volatile uint32_t capture_len[ROWS];

void capture_rows(void);

#endif