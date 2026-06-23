/**
 * @file i2c.h
 * @brief 主机 I2C 配置 - 陀螺仪 JY901B + OLED
 *
 * 硬件接线（电路说明文档 V1.0）：
 *   PB6 (SCL) / PB7 (SDA) → JY901B(5V) + OLED(3.3V)，I2C1/I2C2 共用引脚
 */

#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
