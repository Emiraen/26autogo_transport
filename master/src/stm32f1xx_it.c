/**
 * @file stm32f1xx_it.c
 * @brief 中断服务程序 - chassis 主机
 */

#include "main.h"
#include "stm32f1xx_it.h"
#include "dma.h"
#include "serial_protocol.h"

/* 外部变量声明 */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* ========== Cortex-M3 异常处理 ========== */

void NMI_Handler(void)
{
    while (1) {}
}

/**
 * @brief HardFault：触发系统软复位（IWDG 兜底，立即重启）
 */
void HardFault_Handler(void)
{
    /* 关中断防止反复进入 */
    __disable_irq();
    /* 短延迟以便外设排空，然后系统复位 */
    for (volatile uint32_t i = 0; i < 100000; i++);
    NVIC_SystemReset();
    while (1) {}
}

void MemManage_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

void BusFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

void UsageFault_Handler(void)
{
    NVIC_SystemReset();
    while (1) {}
}

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ========== 外设中断处理 ========== */

void DMA1_Channel4_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

void DMA1_Channel5_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
 * @brief DMA1 Channel7 中断 (USART2 TX)
 */
void DMA1_Channel7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

/**
 * @brief USART1 中断处理 - 处理 IDLE 空闲中断
 */
void USART1_IRQHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        DMA_UART1_RxIdleHandler();
    }

    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief USART2 中断处理 - 处理 IDLE 空闲中断
 */
void USART2_IRQHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE) != RESET))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        DMA_UART2_RxIdleHandler();
    }

    HAL_UART_IRQHandler(&huart2);
}

/* ========== HAL UART TX 完成回调 ========== */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        DMA_UART1_TxCpltHandler();
    }
    else if (huart->Instance == USART2)
    {
        DMA_UART2_TxCpltHandler();
    }
}
