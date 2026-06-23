/**
 * @file stm32f1xx_it.c
 * @brief 中断服务程序 - chassis2 副机
 *
 * 鲁棒性设计:
 *   - USART2 IRQ 中先抢先清 ORE/FE/NE/PE 错误位，避免 HAL 把 RxState
 *     置为 READY 后接收中断停摆
 *   - HAL_UART_ErrorCallback 在错误事件时重新挂起单字节接收，保证
 *     上位机长时间静默后再发数据时副机能正常处理
 */

#include "main.h"
#include "stm32f1xx_it.h"
#include "usart.h"

/* 外部变量声明 */
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern uint8_t usart2_rx_buf[];

/* 接收恢复标志：在 ErrorCallback 中置位，main 循环兜底重启时使用 */
volatile uint8_t g_usart2_rx_need_restart = 0;

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

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ========== 外设中断处理 ========== */

/**
 * @brief USART2 中断处理 - 接收主机指令
 *
 * 进入 HAL_UART_IRQHandler 之前先把 ORE/FE/NE/PE 错误位清掉
 * （读 SR + 读 DR 是 STM32F1 清 ORE 的官方序列），否则一旦上位机
 * 长时间静默后再发数据，HAL 会因检测到 ORE 直接进入错误分支并
 * 把 RxState 置为 READY，HAL_UART_RxCpltCallback 不再触发，
 * 单字节中断接收链路就此停摆。
 */
void USART2_IRQHandler(void)
{
    uint32_t sr = huart2.Instance->SR;
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        (void)huart2.Instance->DR;
    }
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief USART3 中断处理 - RS485 EMM42 通信
 */
void USART3_IRQHandler(void)
{
    uint32_t sr = huart3.Instance->SR;
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        (void)huart3.Instance->DR;
    }
    HAL_UART_IRQHandler(&huart3);
}

/**
 * @brief HAL UART 错误回调
 *
 * 错误事件会让 HAL 把对应 UART 的 RxState 置为 READY，停止接收。
 * 这里立即重新挂起单字节接收，使主从链路自愈。
 *
 * 注意：HAL_UART_AbortReceive_IT + HAL_UART_Receive_IT 比直接 Receive_IT
 * 更安全，避免内部状态残留。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* 把可能残留的错误位再清一次 */
        __HAL_UART_CLEAR_PEFLAG(huart);
        (void)huart->Instance->SR;
        (void)huart->Instance->DR;
        /* 中止后重新挂起接收 */
        HAL_UART_AbortReceive_IT(huart);
        if (HAL_UART_Receive_IT(huart, usart2_rx_buf, 1) != HAL_OK)
        {
            /* 极端情况下重启失败，置标志由 main 兜底重启 */
            g_usart2_rx_need_restart = 1;
        }
    }
    else if (huart->Instance == USART3)
    {
        /* RS485 走的是阻塞式 HAL_UART_Transmit / Receive，无需在这里恢复，
         * 但仍要把错误位清掉防止下一次阻塞收发被错误标志影响 */
        __HAL_UART_CLEAR_PEFLAG(huart);
        (void)huart->Instance->SR;
        (void)huart->Instance->DR;
    }
}
