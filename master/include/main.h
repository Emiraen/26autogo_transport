/**
 * @file main.h
 * @brief 主机工程 - 主头文件
 * 
 * 硬件配置：
 * - USART1 (PA9/PA10) → 上位机通信 (230400)
 * - USART2 (PA2/PA3)  → 主从通信 TX→副机RX (115200)
 * - PC13 → LED 心跳
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void Error_Handler(void);

/* IWDG 喂狗（main.c 实现，长流程模块在阻塞延时期间使用） */
void IWDG_Feed(void);
void ERR_Report(const char *msg);

/* 引脚定义 */
#define LED_Pin         GPIO_PIN_13
#define LED_GPIO_Port   GPIOC

/* I2C1 默认引脚 (PB6=SCL, PB7=SDA) - 陀螺仪 JY901B + OLED */
#define JY901_SCL_Pin       GPIO_PIN_6
#define JY901_SDA_Pin       GPIO_PIN_7
#define JY901_GPIO_Port     GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
