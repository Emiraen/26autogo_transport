/**
 * @file dma.h
 * @brief 主机 DMA 配置
 *
 * USART1 DMA: 上位机通信 (RX + TX, 带 FIFO 队列)
 * USART2 DMA: 主从通信 (RX + TX, 带 FIFO 队列)
 */

#ifndef __DMA_H
#define __DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* DMA handle 外部声明 */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

void MX_DMA_Init(void);

/* USART1 (上位机) */
void DMA_UART1_Start(void);
/**
 * @brief 非阻塞将一帧数据塞入 USART1 TX FIFO 队列
 * @return true 入队成功; false 队列已满
 */
bool DMA_UART1_TxEnqueue(const uint8_t *data, uint16_t len);
/**
 * @brief 旧接口兼容：入队 + 可选超时等待（timeout 仅用于等待队列有空位）
 */
bool DMA_UART1_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
void DMA_UART1_RxIdleHandler(void);
void DMA_UART1_TxCpltHandler(void);
bool DMA_UART1_GetPendingData(const uint8_t **buf1, uint16_t *len1,
                              const uint8_t **buf2, uint16_t *len2);
uint32_t DMA_UART1_GetRxOverrunCount(void);

/* USART2 (副机) */
void DMA_UART2_Start(void);
bool DMA_UART2_TxEnqueue(const uint8_t *data, uint16_t len);
bool DMA_UART2_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms);
void DMA_UART2_RxIdleHandler(void);
void DMA_UART2_TxCpltHandler(void);
bool DMA_UART2_GetPendingData(const uint8_t **buf1, uint16_t *len1,
                              const uint8_t **buf2, uint16_t *len2);
uint32_t DMA_UART2_GetRxOverrunCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H */
