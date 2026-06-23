/**
 * @file JY901B.c
 * @brief JY901B 陀螺仪驱动实现 - chassis 主机适配（仅 I2C 轮询）
 *
 * 精简自根目录 JY901B.c，移除 UART 路径，仅保留 I2C 轮询读取。
 * 调度建议：在主循环中以 5ms (200Hz) 间隔调用 JY901B_PollI2C(&hi2c1)。
 */

#include "JY901B.h"
#include "i2c.h"
#include <string.h>

JY901B_Data g_jy901_data = {0};

static uint8_t  s_raw[JY901_I2C_BLOCK_BYTES];
static uint32_t s_sample_counter = 0U;
static uint32_t s_error_counter  = 0U;
static uint32_t s_consecutive_errors = 0U;
static volatile uint32_t s_last_sample_tick = 0U;  /**< 最近一次成功采样时 HAL_GetTick() */

#ifndef JY901_I2C_RECOVER_THRESHOLD
#define JY901_I2C_RECOVER_THRESHOLD 10U
#endif

/* 软复位 I2C 总线（连续失败 N 次后调用） */
static void JY901B_I2C_Recover(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return;
    HAL_I2C_DeInit(hi2c);
    /* 与 i2c.c 中保持一致的初始化 */
    extern void MX_I2C1_Init(void);
    MX_I2C1_Init();
    s_consecutive_errors = 0;
}

/* ---- 内部工具 ---- */

static int16_t le_to_int16(const uint8_t *src)
{
    return (int16_t)(((uint16_t)src[1] << 8U) | src[0]);
}

/* ---- API 实现 ---- */

bool JY901B_PollI2C(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return false;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        hi2c,
        (uint16_t)(JY901_I2C_ADDR << 1U),
        JY901_REG_START,
        I2C_MEMADD_SIZE_8BIT,
        s_raw,
        JY901_I2C_BLOCK_BYTES,
        JY901_I2C_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        s_error_counter++;
        if (++s_consecutive_errors >= JY901_I2C_RECOVER_THRESHOLD)
        {
            JY901B_I2C_Recover(hi2c);
        }
        return false;
    }
    s_consecutive_errors = 0;

    /* 解析：跳过前 8 字节时间戳，从 AX 开始 */
    const uint8_t *p = &s_raw[JY901_DATA_OFFSET_BYTES];

    g_jy901_data.ax = (float)le_to_int16(&p[0])  / 32768.0f * 16.0f;
    g_jy901_data.ay = (float)le_to_int16(&p[2])  / 32768.0f * 16.0f;
    g_jy901_data.az = (float)le_to_int16(&p[4])  / 32768.0f * 16.0f;

    g_jy901_data.wx = (float)le_to_int16(&p[6])  / 32768.0f * 2000.0f;
    g_jy901_data.wy = (float)le_to_int16(&p[8])  / 32768.0f * 2000.0f;
    g_jy901_data.wz = (float)le_to_int16(&p[10]) / 32768.0f * 2000.0f;

    /* 跳过磁场 6 字节 (offset 12~17)，欧拉角从 offset 18 开始 */
    g_jy901_data.roll  = (float)le_to_int16(&p[18]) / 32768.0f * 180.0f;
    g_jy901_data.pitch = (float)le_to_int16(&p[20]) / 32768.0f * 180.0f;
    g_jy901_data.yaw   = (float)le_to_int16(&p[22]) / 32768.0f * 180.0f;

    s_last_sample_tick = HAL_GetTick();
    s_sample_counter++;
    return true;
}

bool JY901B_GetSnapshot(JY901B_Data *out)
{
    if (out == NULL)
    {
        return false;
    }
    *out = g_jy901_data;
    return (s_sample_counter > 0U);
}

bool JY901B_GetSnapshotEx(JY901B_Data *out, uint32_t *tick_ms)
{
    if (out == NULL) return false;
    /* 短临界区：避免在拷贝期间被 I2C 路径更新 */
    __disable_irq();
    *out = g_jy901_data;
    uint32_t t = s_last_sample_tick;
    uint32_t cnt = s_sample_counter;
    __enable_irq();
    if (tick_ms) *tick_ms = t;
    return (cnt > 0U);
}

uint32_t JY901B_GetSampleCounter(void)
{
    return s_sample_counter;
}

uint32_t JY901B_GetErrorCounter(void)
{
    return s_error_counter;
}
