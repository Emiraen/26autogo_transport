/**
 * @file crc16_util.h
 * @brief CRC16-Modbus 计算工具 (header-only, inline)
 *
 * 与主机 chassis/src/serial_protocol.c 中 crc16() 算法完全一致：
 *   多项式 0xA001 (反射), 初值 0xFFFF, RefIn=true, RefOut=true, XorOut=0x0000
 *
 * 使用 inline 的目的是让两个独立工程不必维护一份重复实现，
 * 复制此文件到对方 include 目录即可保持一致。
 */

#ifndef __CRC16_UTIL_H
#define __CRC16_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

static inline uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* __CRC16_UTIL_H */
