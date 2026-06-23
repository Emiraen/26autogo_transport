/**
 * @file usart.h
 * @brief 副机 USART 配置头文件
 * 
 * USART2: 主从通信 RX (PA3 接收主机指令)
 * USART3: RS485 → EMM42 电机 (PB10 TX, PB11 RX)
 */

#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* 串口句柄声明 */
extern UART_HandleTypeDef huart2;  /* 主从通信 */
extern UART_HandleTypeDef huart3;  /* RS485 → EMM42 */

/* 接收缓冲区大小 */
#define USART2_RX_BUF_SIZE  128U
#define USART3_RX_BUF_SIZE  64U

/* 接收缓冲区声明 */
extern uint8_t usart2_rx_buf[USART2_RX_BUF_SIZE];
extern uint8_t usart3_rx_buf[USART3_RX_BUF_SIZE];

/**
 * @brief 初始化 USART2 (主从通信 RX)
 */
void MX_USART2_UART_Init(void);

/**
 * @brief 初始化 USART3 (RS485 → EMM42)
 */
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
