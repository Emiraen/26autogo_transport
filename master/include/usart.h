/**
 * @file usart.h
 * @brief 主机串口配置
 * 
 * USART1 (PA9/PA10) → 上位机通信
 * USART2 (PA2/PA3)  → 主从通信 TX→副机RX
 */

#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
