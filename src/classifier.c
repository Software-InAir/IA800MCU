#include "stm32h723xx.h"
#include "classifier.h"
#include "config.h"


/* ================== LUT ================== */
/* Pattern 2 LUT ONLY */
const char char_lut[256] = {
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

const char char_lut2[256] = {
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

const char *atr[256] = {
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

const char *bae[256] = {
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

const char *dhc[256] = {
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

const char *citation[256] = {
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

int lut_valid1(uint8_t v)
{
    return char_lut[v] != 0;
}

int lut_valid2(uint8_t v)
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