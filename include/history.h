#ifndef HISTORY_H
#define HISTORY_H

#include <stdint.h>

#define ROWS 4
#define CONF_DEPTH 8
#define MAX_CHARS 26

extern char history_arr[ROWS][CONF_DEPTH][MAX_CHARS + 1];
extern uint8_t history_index_arr[ROWS];
extern uint8_t history_count_arr[ROWS];

void history_push(int row, char *str);
int  history_vote(int row);

#endif