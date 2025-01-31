#ifndef INC_COMMON_CRC_H_
#define INC_COMMON_CRC_H_

#pragma once

struct sbuf_s;

uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint16_t crc16_ccitt_update(uint16_t crc, const void *data, uint32_t length);
struct sbuf_s;
void crc16_ccitt_sbuf_append(struct sbuf_s *dst, uint8_t *start);

uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
void crc8_sbuf_append(struct sbuf_s *dst, uint8_t *start, uint8_t poly);
#define crc8_dvb_s2(crc, a)                         crc8_calc(crc, a, 0xD5)
#define crc8_dvb_s2_update(crc, data, length)       crc8_update(crc, data, length, 0xD5)
#define crc8_dvb_s2_sbuf_append(dst, start)         crc8_sbuf_append(dst, start, 0xD5)
#define crc8_poly_0xba(crc, a)                      crc8_calc(crc, a, 0xBA)
#define crc8_poly_0xba_sbuf_append(dst, start)      crc8_sbuf_append(dst, start, 0xBA)

uint8_t crc8_xor_update(uint8_t crc, const void *data, uint32_t length);
void crc8_xor_sbuf_append(struct sbuf_s *dst, uint8_t *start);

#define FNV_PRIME           16777619
#define FNV_OFFSET_BASIS    2166136261

uint32_t fnv_update(uint32_t hash, const void *data, uint32_t length);

#endif
