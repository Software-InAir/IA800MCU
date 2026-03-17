#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <stdint.h>

/* ================== PAYLOAD BUILDER ================== */

/*
 * Builds a command payload from a string message.
 *
 * Format:
 * [0x5A 0xA5] [LEN] [CMD] [ADDR_H] [ADDR_L] [DATA...] [CRC_L] [CRC_H]
 *
 * - CRC is Modbus CRC16 over bytes starting at index 3
 * - Payload is padded to even length
 */
void build_and_print_payload(const char *message);

#endif