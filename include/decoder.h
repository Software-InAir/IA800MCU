#ifndef DECODER_H
#define DECODER_H

#include <stdint.h>

#define MAX_CHARS 26
#define ROWS 4

extern char decoded_str[ROWS][MAX_CHARS + 1];

void decode_row(uint8_t* buf, uint32_t len, char* out);
void normalize_string(char *str);

#endif