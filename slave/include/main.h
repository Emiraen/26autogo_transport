/**
 * @file main.h
 * @brief 副机主头文件 - STM32F103C8T6
 * 
 * 副机职责：
 * - 接收主机指令 (USART2 RX)
 * - 通过 RS485 (USART3) 驱动 EMM42 电机
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ========== GPIO 定义 ========== */

/* LED - PC13 */
#define LED_Pin         GPIO_PIN_13
#define LED_GPIO_Port   GPIOC

/* RS485 方向控制 - PB1 */
#define RS485_DE_Pin        GPIO_PIN_1
#define RS485_DE_GPIO_Port  GPIOB

/* RS485 方向控制宏 */
#define RS485_TX_ENABLE()   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define RS485_RX_ENABLE()   HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)

/* ========== 系统函数 ========== */

void Error_Handler(void);
void SystemClock_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
