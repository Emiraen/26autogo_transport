/**
 * @file lift_motor.c
 * @brief 升降 EMM42 步进电机驱动 (主机 USART3 / M5 接口)
 *
 * USART3 直连 EMM42 (主机侧 M5 通过 TTL 直接接到电机串口)
 * 由于不走 RS485，无方向控制 GPIO，TX 完成即可。
 */

#include "lift_motor.h"
#include <string.h>
#include <math.h>

/* 主机 USART3 句柄 (本文件内部维护) */
static UART_HandleTypeDef hlift_uart;

/* 软件记账的当前高度 (mm) */
static float s_lift_height_mm = 0.0f;

#define LIFT_TX_TIMEOUT_MS  10U

/* ----- 底层 USART3 初始化 ----- */
static void Lift_USART3_Init(void)
{
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* PB10 = USART3_TX (复用推挽) */
    gi.Pin = GPIO_PIN_10;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);

    /* PB11 = USART3_RX (浮空输入) */
    gi.Pin = GPIO_PIN_11;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gi);

    hlift_uart.Instance = USART3;
    hlift_uart.Init.BaudRate = 115200;
    hlift_uart.Init.WordLength = UART_WORDLENGTH_8B;
    hlift_uart.Init.StopBits = UART_STOPBITS_1;
    hlift_uart.Init.Parity = UART_PARITY_NONE;
    hlift_uart.Init.Mode = UART_MODE_TX_RX;
    hlift_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlift_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&hlift_uart) != HAL_OK) Error_Handler();
}

/* 阻塞发一帧 */
static bool Lift_TxFrame(const uint8_t *data, uint16_t len)
{
    return (HAL_UART_Transmit(&hlift_uart, (uint8_t *)data, len, LIFT_TX_TIMEOUT_MS) == HAL_OK);
}

/* ----- API ----- */

void Lift_Init(void)
{
    Lift_USART3_Init();
    HAL_Delay(20);
    /* 默认上电使能电机 */
    Lift_Enable(true);
    /* 约定: 机械装配时，上电默认钩爪位于塔吊最顶端 */
    s_lift_height_mm = LIFT_HEIGHT_TOP_MM;
}

bool Lift_Enable(bool enable)
{
    uint8_t f[6] = {
        LIFT_MOTOR_ADDR,
        LIFT_CMD_ENABLE,
        0xAB,
        (uint8_t)(enable ? 0x01 : 0x00),
        0x00,                 /* sync */
        LIFT_CHECKSUM
    };
    return Lift_TxFrame(f, sizeof(f));
}

bool Lift_StopNow(void)
{
    uint8_t f[5] = { LIFT_MOTOR_ADDR, LIFT_CMD_STOP, 0x98, 0x00, LIFT_CHECKSUM };
    return Lift_TxFrame(f, sizeof(f));
}

bool Lift_SyncStart(void)
{
    uint8_t f[4] = { 0x00, LIFT_CMD_SYNC, 0x66, LIFT_CHECKSUM };
    return Lift_TxFrame(f, sizeof(f));
}

bool Lift_SetSpeed(bool ccw, uint16_t rpm, uint8_t accel, bool sync)
{
    uint8_t f[8];
    f[0] = LIFT_MOTOR_ADDR;
    f[1] = LIFT_CMD_SPEED;
    f[2] = ccw ? 0x01 : 0x00;
    f[3] = (uint8_t)(rpm >> 8);
    f[4] = (uint8_t)(rpm & 0xFF);
    f[5] = accel;
    f[6] = sync ? 0x01 : 0x00;
    f[7] = LIFT_CHECKSUM;
    return Lift_TxFrame(f, sizeof(f));
}

bool Lift_MovePulses(bool ccw, uint16_t rpm, uint8_t accel,
                     uint32_t pulses, bool absolute, bool sync)
{
    uint8_t f[13];
    f[0]  = LIFT_MOTOR_ADDR;
    f[1]  = LIFT_CMD_POSITION;
    f[2]  = ccw ? 0x01 : 0x00;
    f[3]  = (uint8_t)(rpm >> 8);
    f[4]  = (uint8_t)(rpm & 0xFF);
    f[5]  = accel;
    f[6]  = (uint8_t)(pulses >> 24);
    f[7]  = (uint8_t)((pulses >> 16) & 0xFF);
    f[8]  = (uint8_t)((pulses >> 8) & 0xFF);
    f[9]  = (uint8_t)(pulses & 0xFF);
    f[10] = absolute ? 0x01 : 0x00;
    f[11] = sync ? 0x01 : 0x00;
    f[12] = LIFT_CHECKSUM;
    return Lift_TxFrame(f, sizeof(f));
}

/* mm → 脉冲 */
static uint32_t mm_to_pulses(float mm)
{
    float revs = mm / LIFT_MM_PER_REV;
    if (revs < 0) revs = -revs;
    return (uint32_t)(revs * LIFT_PULSES_PER_REV + 0.5f);
}

bool Lift_MoveToHeight(float height_mm)
{
    float delta = height_mm - s_lift_height_mm;
    bool ok = Lift_MoveRelative(delta);
    if (ok) s_lift_height_mm = height_mm;
    return ok;
}

bool Lift_MoveRelative(float delta_mm)
{
    if (fabsf(delta_mm) < 0.01f) return true;
    bool ccw = (delta_mm > 0.0f);    /* 约定: 升高 = CCW，可视实际机构反转 */
    uint32_t pulses = mm_to_pulses(delta_mm);
    bool ok = Lift_MovePulses(ccw, LIFT_DEFAULT_RPM, LIFT_DEFAULT_ACCEL,
                              pulses, false /* 相对 */, false);
    if (ok) s_lift_height_mm += delta_mm;
    return ok;
}

float Lift_GetHeight(void) { return s_lift_height_mm; }

UART_HandleTypeDef *Lift_GetUart(void) { return &hlift_uart; }

bool Lift_TxRaw(const uint8_t *data, uint16_t len)
{
    return Lift_TxFrame(data, len);
}
