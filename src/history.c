#include "history.h"
#include "config.h"

char history_arr[ROWS][CONF_DEPTH][MAX_CHARS + 1];
uint8_t history_index_arr[ROWS];
uint8_t history_count_arr[ROWS];

void history_push(int row, char *str)
    {
        for (uint8_t i = 0; i <= MAX_CHARS; i++)
        {
            history_arr[row][history_index_arr[row]][i] = str[i];
        }

        history_index_arr[row]++;

        if (history_index_arr[row] >= CONF_DEPTH)
            history_index_arr[row] = 0;

        if (history_count_arr[row] < CONF_DEPTH)
            history_count_arr[row]++;
    }

int history_vote(int row)
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
                    if (history_arr[row][i][k] != history_arr[row][j][k])
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

        return best_index;
    }