/**
 * @file gpio.c
 * @brief 副机 GPIO 配置实现
 * 
 * LED: PC13
 * RS485_DE: PB1 (MAX3485 方向控制，高电平=发送，低电平=接收)
 */

#include "gpio.h"
#include "main.h"

/**
 * @brief GPIO 初始化
 * - PC13: LED 输出
 * - PB1: RS485_DE 方向控制输出
 */
void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能 GPIO 时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 初始化 LED (PC13) 为低电平 (LED 灭) */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /* 初始化 RS485_DE (PB1) 为低电平 (接收模式) */
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);

    /* 配置 LED 引脚 (PC13) */
    GPIO_InitStruct.Pin = LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

    /* 配置 RS485_DE 引脚 (PB1) */
    GPIO_InitStruct.Pin = RS485_DE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief RS485 切换到发送模式
 * PB1 = HIGH → MAX3485 DE/RE 高电平 → 发送使能
 */
void RS485_SetTxMode(void)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
}

/**
 * @brief RS485 切换到接收模式
 * PB1 = LOW → MAX3485 DE/RE 低电平 → 接收使能
 */
void RS485_SetRxMode(void)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
}
