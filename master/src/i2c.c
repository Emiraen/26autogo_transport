/**
 * @file i2c.c
 * @brief 主机 I2C1 初始化 - 陀螺仪 JY901B + OLED
 *
 * 电路文档 V1.0 标注：PB6(SCL) / PB7(SDA) 默认引脚映射，
 * I2C1 与 I2C2 复用同一组 SCL/SDA 引脚（OLED@3.3V，JY901B@5V）。
 */

#include "i2c.h"

I2C_HandleTypeDef hi2c1;

/* 总线解锁后读到的线上电平 (1=高/正常, 0=被拉低/无有效上拉) */
volatile uint8_t g_i2c_scl_level = 0xFF;
volatile uint8_t g_i2c_sda_level = 0xFF;
/* 最终 BUSY 标志 (0=正常, 1=仍 BUSY) 与做了几次 SWRST 恢复 */
volatile uint8_t g_i2c_busy_after = 0xFF;
volatile uint8_t g_i2c_recover_cnt = 0;

/**
 * @brief I2C 总线解锁 (摆 9 个 SCL + STOP)
 *
 * STM32F103 经典问题: 上电瞬间总线状态异常会让从机拉住 SDA,
 * 或让 HAL I2C 进入 BUSY 死锁, 之后所有读写(含扫描)立即失败。
 * 解法: 把 PB6/PB7 临时当普通 GPIO, 手动甲 9 个 SCL 时钟把从机
 * 状态机赶出, 再补一个 STOP, 最后对 I2C 外设做 SWRST。
 */
static void I2C1_BusUnstick(void)
{
    GPIO_InitTypeDef gi = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB6=SCL, PB7=SDA 设为开漏输出, 先都拉高 */
    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gi.Mode = GPIO_MODE_OUTPUT_OD;
    gi.Pull = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    HAL_Delay(1);

    /* 若 SDA 被从机拉低, 甲 9 个 SCL 脉冲让其释放 */
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  /* SCL low */
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    /* SCL high */
        HAL_Delay(1);
    }

    /* 生成一个 STOP: SDA 在 SCL 高电平期间由低变高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);  /* SDA low */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);    /* SCL high */
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);    /* SDA high -> STOP */
    HAL_Delay(1);

    /* 释放总线后读真实线上电平 (OD 输出写 1 = 高阻, IDR 反映实际电平)
     * SCL/SDA 都应被上拉拉到高; 若读到 0 说明线起不来
     * (供电未到/上拉无效/对地短路/从机死拉) */
    HAL_Delay(1);
    g_i2c_scl_level = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1 : 0;
    g_i2c_sda_level = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1 : 0;
}

/**
 * @brief I2C 外设 BUSY 死锁恢复 (甲总线时钟 + SWRST + 重新 Init)
 *
 * 任何一笔失败/无应答的传输都可能把 F103 的 BUSY 标志卡死,
 * 之后所有传输直接返回 HAL_BUSY(2)。调用本函数可清掉。
 * 返回最终 BUSY 状态 (0=正常, 1=仍 BUSY)。
 */
uint8_t I2C1_RecoverBusy(void)
{
    uint8_t cnt = 0;
    while (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) && cnt < 5)
    {
        cnt++;
        I2C1_BusUnstick();
        __HAL_I2C_DISABLE(&hi2c1);
        hi2c1.Instance->CR1 |=  I2C_CR1_SWRST;
        HAL_Delay(1);
        hi2c1.Instance->CR1 &= ~I2C_CR1_SWRST;
        HAL_Delay(1);
        if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        {
            Error_Handler();
        }
    }
    return __HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) ? 1 : 0;
}

/**
 * @brief I2C1 初始化 - 100kHz (含 BUSY 死锁恢复)
 */
void MX_I2C1_Init(void)
{
    /* 初始化前先解锁总线, 避免上电 BUSY 死锁 */
    I2C1_BusUnstick();

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;   /* 100kHz: 配合弱上拉更稳, OLED/IMU 都够用 */
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    g_i2c_recover_cnt = 0;
    g_i2c_busy_after = I2C1_RecoverBusy();
}

/**
 * @brief I2C MSP 初始化 - GPIO 与时钟配置
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* I2C1 默认引脚: PB6=SCL, PB7=SDA
         * 开漏 + 开内部上拉作为兑底 (总线仍建议外部 4.7k 上拉) */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
    }
}
