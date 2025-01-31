
#ifndef INC_COMMON_STREAMBUF_H_
#define INC_COMMON_STREAMBUF_H_

#pragma once

//#include "CK_DEFINITIONS.h"
#include <stdint.h>

// simple buffer-based serializer/deserializer without implicit size check

typedef struct sbuf_s {
    uint8_t *ptr;          // data pointer must be first (sbuf_t* is equivalent to uint8_t **)
    uint8_t *end;
} sbuf_t;

sbuf_t *sbufInit(sbuf_t *sbuf, uint8_t *ptr, uint8_t *end);

void sbufWriteU8(sbuf_t *dst, uint8_t val);
void sbufWriteU16(sbuf_t *dst, uint16_t val);
void sbufWriteU32(sbuf_t *dst, uint32_t val);
void sbufWriteU16BigEndian(sbuf_t *dst, uint16_t val);
void sbufWriteU32BigEndian(sbuf_t *dst, uint32_t val);
void sbufFill(sbuf_t *dst, uint8_t data, int len);
void sbufWriteData(sbuf_t *dst, const void *data, int len);
void sbufWriteString(sbuf_t *dst, const char *string);
void sbufWritePString(sbuf_t *dst, const char *string);
void sbufWriteStringWithZeroTerminator(sbuf_t *dst, const char *string);

uint8_t sbufReadU8(sbuf_t *src);
uint16_t sbufReadU16(sbuf_t *src);
uint32_t sbufReadU32(sbuf_t *src);
void sbufReadData(sbuf_t *dst, void *data, int len);

int sbufBytesRemaining(sbuf_t *buf);
uint8_t* sbufPtr(sbuf_t *buf);
const uint8_t* sbufConstPtr(const sbuf_t *buf);
void sbufAdvance(sbuf_t *buf, int size);

void sbufSwitchToReader(sbuf_t *buf, uint8_t * base);

#endif
