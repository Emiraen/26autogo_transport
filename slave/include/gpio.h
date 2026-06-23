/**
 * @file gpio.h
 * @brief 副机 GPIO 配置头文件
 * 
 * LED: PC13
 * RS485_DE: PB1 (MAX3485 方向控制)
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/**
 * @brief 初始化 GPIO
 */
void MX_GPIO_Init(void);

/**
 * @brief RS485 切换到发送模式
 */
void RS485_SetTxMode(void);

/**
 * @brief RS485 切换到接收模式
 */
void RS485_SetRxMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */
