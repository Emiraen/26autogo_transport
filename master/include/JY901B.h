/**
 * @file JY901B.h
 * @brief JY901B 陀螺仪驱动 - chassis 主机适配
 *
 * 通信方式：I2C（轮询读取）
 * 硬件接线：PB8(SCL) / PB9(SDA) → I2C1 REMAP
 * I2C 地址：0x50（7-bit）
 * 寄存器窗口：0x30~0x4F（32 字节，含时间戳+加速度+角速度+磁场+欧拉角）
 */

#ifndef __JY901B_H
#define __JY901B_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ---- I2C 参数 ---- */
#ifndef JY901_I2C_ADDR
#define JY901_I2C_ADDR          (0x50U)   /**< 7-bit 器件地址 */
#endif
#ifndef JY901_REG_START
#define JY901_REG_START         (0x30U)   /**< 连续读取起始寄存器 */
#endif
#ifndef JY901_I2C_BLOCK_BYTES
#define JY901_I2C_BLOCK_BYTES   (32U)     /**< 一次读取字节数 0x30~0x4F */
#endif
#ifndef JY901_DATA_OFFSET_BYTES
#define JY901_DATA_OFFSET_BYTES (8U)      /**< 前 8 字节为时间戳，IMU 数据从此偏移开始 */
#endif
#ifndef JY901_I2C_TIMEOUT_MS
#define JY901_I2C_TIMEOUT_MS    (5U)
#endif

/* ---- 数据结构 ---- */
typedef struct
{
    float ax, ay, az;       /**< 加速度 (g) */
    float wx, wy, wz;       /**< 角速度 (°/s) */
    float roll, pitch, yaw;  /**< 欧拉角 (°) */
} JY901B_Data;

extern JY901B_Data g_jy901_data;

/**
 * @brief 拷贝最近一次 IMU 数据快照 + 采样时刻 tick
 * @param out      数据输出
 * @param tick_ms  对应的 HAL_GetTick() 时刻（成功 I2C 读取完成时刻）
 * @return true 已有有效数据
 */
bool JY901B_GetSnapshotEx(JY901B_Data *out, uint32_t *tick_ms);

/* ---- API ---- */

/**
 * @brief 轮询式 I2C 读 IMU 并刷新全局姿态数据
 * @param hi2c I2C 句柄（本项目为 &hi2c1）
 * @return true 读取成功；false I2C 错误
 */
bool JY901B_PollI2C(I2C_HandleTypeDef *hi2c);

/**
 * @brief 拷贝最近一次 IMU 数据快照
 * @param out 目标缓冲
 * @return true 已有有效数据；false 尚未采集到
 */
bool JY901B_GetSnapshot(JY901B_Data *out);

/** @brief 成功采样计数 */
uint32_t JY901B_GetSampleCounter(void);

/** @brief I2C 读取失败计数 */
uint32_t JY901B_GetErrorCounter(void);

#ifdef __cplusplus
}
#endif

#endif /* __JY901B_H */
