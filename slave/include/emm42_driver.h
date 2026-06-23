/**
 * @file emm42_driver.h
 * @brief EMM42 闭环步进电机 RS485 驱动接口 (副机专用)
 * 
 * 通过 USART3 + RS485 (PB1 方向控制) 与 4 个 EMM42 电机通信
 * 校验模式: 固定 0x6B
 */

#ifndef __EMM42_DRIVER_H__
#define __EMM42_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* EMM42 电机地址定义 (1-4 对应四个轮子) */
#define EMM42_MOTOR_FL  1U  /* 前左轮 */
#define EMM42_MOTOR_FR  2U  /* 前右轮 */
#define EMM42_MOTOR_RL  3U  /* 后左轮 */
#define EMM42_MOTOR_RR  4U  /* 后右轮 */
#define EMM42_BROADCAST 0U  /* 广播地址 */

/* EMM42 功能码定义 */
#define EMM42_CMD_READ_PULSES   0x32U  /* 读取输入脉冲数 */
#define EMM42_CMD_READ_SPEED    0x35U  /* 读取电机实时转速 */
#define EMM42_CMD_READ_POS      0x36U  /* 读取电机实时位置 */
#define EMM42_CMD_ENABLE        0xF3U  /* 使能控制 */
#define EMM42_CMD_SPEED         0xF6U  /* 速度模式 */
#define EMM42_CMD_POSITION      0xFDU  /* 位置模式 */
#define EMM42_CMD_STOP          0xFEU  /* 立即停止 */
#define EMM42_CMD_SYNC          0xFFU  /* 同步启动 */

/* 电机数量 */
#define EMM42_MOTOR_COUNT   4U

/* 读取响应长度 */
#define EMM42_READ_RESP_LEN     8U  /* addr + cmd + sign + data(4B) + 0x6B */
#define EMM42_READ_SPEED_LEN    6U  /* addr + 0x35 + sign + rpm(2B) + 0x6B */
#define EMM42_READ_ERR_LEN      4U  /* addr + 0x00 + 0xEE + 0x6B */

/* EMM42 校验字节 */
#define EMM42_CHECKSUM      0x6BU

/**
 * @brief EMM42 状态码
 */
typedef enum {
    EMM42_OK = 0,
    EMM42_ERR_PARAM,
    EMM42_ERR_TIMEOUT,
    EMM42_ERR_BUSY
} EMM42_Status;

/**
 * @brief 初始化 EMM42 驱动
 * @param huart USART3 句柄指针
 */
void EMM42_Init(UART_HandleTypeDef *huart);

/**
 * @brief 电机使能控制
 * @param addr 电机地址 (1-4 或 0 广播)
 * @param enable true=使能, false=失能
 * @param sync 多机同步标志
 * @return EMM42_Status
 */
EMM42_Status EMM42_Enable(uint8_t addr, bool enable, bool sync);

/**
 * @brief 速度模式控制
 * @param addr 电机地址
 * @param ccw true=CCW方向, false=CW方向
 * @param rpm 目标转速 (RPM)
 * @param accel 加速度档位 (0=不加减速)
 * @param sync 多机同步标志
 * @return EMM42_Status
 */
EMM42_Status EMM42_SetSpeed(uint8_t addr, bool ccw, uint16_t rpm, uint8_t accel, bool sync);

/**
 * @brief 位置模式控制
 * @param addr 电机地址
 * @param ccw true=CCW方向, false=CW方向
 * @param rpm 目标速度 (RPM)
 * @param accel 加速度档位
 * @param pulses 目标脉冲数
 * @param absolute true=绝对位置, false=相对位置
 * @param sync 多机同步标志
 * @return EMM42_Status
 */
EMM42_Status EMM42_MovePosition(uint8_t addr, bool ccw, uint16_t rpm, uint8_t accel,
                                 uint32_t pulses, bool absolute, bool sync);

/**
 * @brief 立即停止
 * @param addr 电机地址
 * @param sync 多机同步标志
 * @return EMM42_Status
 */
EMM42_Status EMM42_StopNow(uint8_t addr, bool sync);

/**
 * @brief 多机同步启动
 * @return EMM42_Status
 */
EMM42_Status EMM42_SyncStart(void);

/**
 * @brief 设置四轮速度 (麦克纳姆轮运动学)
 * @param rpm_fl 前左轮速度 (正=CCW, 负=CW)
 * @param rpm_fr 前右轮速度
 * @param rpm_rl 后左轮速度
 * @param rpm_rr 后右轮速度
 * @param accel 加速度档位
 * @return EMM42_Status
 */
EMM42_Status EMM42_SetWheelSpeeds(int16_t rpm_fl, int16_t rpm_fr,
                                   int16_t rpm_rl, int16_t rpm_rr,
                                   uint8_t accel);

/**
 * @brief 读取单个电机实时位置 (RS485 原子操作: 发送查询 + 接收响应)
 * @param addr 电机地址 (1-4)
 * @param sign [out] 方向符号 (0=正, 1=负)
 * @param position [out] 位置值 (0-65535 表示一圈)
 * @return EMM42_Status
 */
EMM42_Status EMM42_ReadPosition(uint8_t addr, uint8_t *sign, uint32_t *position);

/**
 * @brief 读取单个电机输入脉冲数 (RS485 原子操作)
 * @param addr 电机地址 (1-4)
 * @param sign [out] 方向符号 (0=正, 1=负)
 * @param pulses [out] 累计脉冲数
 * @return EMM42_Status
 */
EMM42_Status EMM42_ReadPulses(uint8_t addr, uint8_t *sign, uint32_t *pulses);

/**
 * @brief 读取全部4个电机的实时位置
 * @param signs [out] 4个方向符号数组
 * @param positions [out] 4个位置值数组
 * @return EMM42_OK=全部成功, 否则返回第一个失败的错误码
 */
/**
 * @brief 读取单个电机实时转速 (RS485 原子操作)
 * @param addr 电机地址 (1-4)
 * @param sign [out] 方向符号 (0=正/CW, 1=负/CCW)
 * @param rpm [out] 实时转速 (RPM)
 * @return EMM42_Status
 */
EMM42_Status EMM42_ReadSpeed(uint8_t addr, uint8_t *sign, uint16_t *rpm);

/**
 * @brief 读取全部4个电机的实时转速 (容错式轮询)
 * @param signs [out] 4个方向符号数组 (失败的电机对应值不修改)
 * @param rpms [out] 4个转速值数组 (失败的电机对应值不修改)
 * @return 失败掩码 bitmask: 0=全部成功, bit0=FL失败, bit1=FR失败, bit2=RL失败, bit3=RR失败
 */
uint8_t EMM42_ReadAllSpeeds(uint8_t signs[EMM42_MOTOR_COUNT],
                             uint16_t rpms[EMM42_MOTOR_COUNT]);

EMM42_Status EMM42_ReadAllPositions(uint8_t signs[EMM42_MOTOR_COUNT],
                                     uint32_t positions[EMM42_MOTOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* __EMM42_DRIVER_H__ */
