#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <stdint.h>

#define ATR_MSG_COUNT      (sizeof(atr)/sizeof(atr[0]))
#define BAE_MSG_COUNT      (sizeof(bae)/sizeof(bae[0]))
#define DHC_MSG_COUNT      (sizeof(dhc)/sizeof(dhc[0]))
#define CITATION_MSG_COUNT (sizeof(citation)/sizeof(citation[0]))

/* ================== AIRFRAME TYPE ================== */

typedef enum
{
    AIRFRAME_NONE = 0,
    AIRFRAME_ATR,
    AIRFRAME_BAE,
    AIRFRAME_DHC,
    AIRFRAME_CITATION
} airframe_t;

/* ================== LUT ACCESS ================== */

/* These are defined in classifier.c */
extern const char char_lut[256];
extern const char char_lut2[256];

extern const char *atr[256];
extern const char *bae[256];
extern const char *dhc[256];
extern const char *citation[256];

/* ================== LUT HELPERS ================== */

int lut_valid1(uint8_t v);
int lut_valid2(uint8_t v);

/* ================== MATCH / CLASSIFICATION ================== */

int score_match(const char *a, const char *b);

void best_in_lut(
    const char *decoded,
    const char **lut,
    int lut_size,
    int *best_index,
    int *best_score);

void classify_message(
    const char *decoded,
    airframe_t *airframe,
    int *msg_index);

#endif