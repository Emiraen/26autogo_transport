/**
 * @file byte_utils.h
 * @brief 小端字节序读写工具 (header-only) - 副机版
 */

#ifndef __BYTE_UTILS_H
#define __BYTE_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

static inline uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline int16_t read_le16s(const uint8_t *p)
{
    return (int16_t)read_le16(p);
}

static inline uint32_t read_le32(const uint8_t *p)
{
    return ((uint32_t)p[0])
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static inline int32_t read_le32s(const uint8_t *p)
{
    return (int32_t)read_le32(p);
}

static inline void write_le16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline void write_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

#ifdef __cplusplus
}
#endif

#endif /* __BYTE_UTILS_H */
