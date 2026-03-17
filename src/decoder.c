#include "stm32h723xx.h"
#include "decoder.h"
#include "classifier.h"   // for char_lut + lut_valid1
#include "config.h"
#include <string.h>

char decoded_str[ROWS][MAX_CHARS + 1];

void decode_row(uint8_t* buf, uint32_t len, char* out)
    {
        uint32_t charcounter = 0;

        uint8_t last = 0;
        uint8_t first = 1;

        char cprev = 0;
        char c2prev = 0;

        for (uint32_t i = 0; i < len; i++)
        {
            if (charcounter >= MAX_CHARS)
                break;

            uint8_t v = buf[i];

            if (lut_valid1(v))
            {
                char c = char_lut[v];

                /* Skip leading spaces */
                if (first && c == ' ')
                    continue;

                if (first || v != last || c == ' ')
                {
                    out[charcounter++] = c;
                    last = v;
                    first = 0;

                    c2prev = cprev;
                    cprev = c;
                }

                /* Vowel repeat logic */
                if (((c2prev == 'A' && cprev == c) ||
                    (c2prev == 'E' && cprev == c) ||
                    (c2prev == 'I' && cprev == c) ||
                    (c2prev == 'O' && cprev == c) ||
                    (c2prev == 'U' && cprev == c)) && (c != 'S'))
                {
                    if (charcounter < MAX_CHARS)
                    {
                        out[charcounter++] = c;
                        last = v;
                        first = 0;

                        c2prev = cprev;
                        cprev = c;
                    }
                }

                /* Number handling logic */
                if ((c >= '0' && c <= '9') &&
                    !(c2prev >= 'A' && c2prev <= 'Z'))
                {
                    if (charcounter < MAX_CHARS)
                    {
                        out[charcounter++] = c;
                        last = v;
                        first = 0;

                        c2prev = cprev;
                        cprev = c;
                    }
                }
            }
        }

        out[charcounter] = '\0';
    }

void normalize_string(char *str)
    {
        /* Trim leading spaces */
        char *p = str;
        while (*p == ' ')
            p++;

        if (p != str)
            memmove(str, p, strlen(p) + 1);

        /* Determine length */
        uint8_t len = 0;
        while (str[len] != '\0' && len < MAX_CHARS)
            len++;

        /* Pad with spaces */
        for (uint8_t i = len; i < MAX_CHARS; i++)
            str[i] = ' ';

        str[MAX_CHARS] = '\0';

        /* Find last non-space */
        int last_non_space = -1;
        for (int i = MAX_CHARS - 1; i >= 0; i--)
        {
            if (str[i] != ' ')
            {
                last_non_space = i;
                break;
            }
        }

        /* Collapse multiple spaces */
        if (last_non_space >= 0)
        {
            int write = 0;
            int space_seen = 0;

            char temp[MAX_CHARS];

            for (int read = 0; read <= last_non_space; read++)
            {
                if (str[read] == ' ')
                {
                    if (!space_seen)
                    {
                        temp[write++] = ' ';
                        space_seen = 1;
                    }
                }
                else
                {
                    temp[write++] = str[read];
                    space_seen = 0;
                }
            }

            while (write < MAX_CHARS)
                temp[write++] = ' ';

            for (int i = 0; i < MAX_CHARS; i++)
                str[i] = temp[i];
        }

        /* Replace nulls with spaces */
        for (uint8_t i = 0; i < MAX_CHARS; i++)
        {
            if (str[i] == '\0')
                str[i] = ' ';
        }

        str[MAX_CHARS] = '\0';
    }