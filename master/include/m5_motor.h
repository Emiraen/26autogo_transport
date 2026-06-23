/**
 * @file m5_motor.h
 * @brief M5 接口 EMM42 步进电机 — 慢速 + 高精度控制层
 *
 * 硬件：
 *   - 主机 USART3 (PB10/PB11) M5 接口，TTL 直连 EMM42 (非 RS485)
 *   - 物理上就是 lift_motor 控制的同一台升降电机 (地址 5)
 *   - 与 lift_motor 共享 USART3，串行调用互不打架
 *
 * 设计目标：
 *   - 慢速：默认 20 RPM (3 秒/圈)，上限夹紧 300 RPM
 *   - 高精度：依靠 EMM42 板上 16 细分 + 细分插补 (MPlyer=Enable) +
 *     加速度档位 200 的平滑 S 曲线，避免低速抖动；
 *     如需更高分辨率，可在板上菜单 MStep 设到 32/64/128/256，
 *     代码不再下发 0x84 切细分以免与 lift 起冲突。
 *
 * 协议：addr + cmd + data + 0x6B (EMM42 V5 标准)
 */

#ifndef __M5_MOTOR_H
#define __M5_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 可配置宏 ===== */
#ifndef M5_MOTOR_ADDR
#define M5_MOTOR_ADDR          5U      /* M5 接口 EMM42 从机地址 (升降电机) */
#endif

/* 与 lift_motor 保持一致的细分 (板上菜单 MStep) - 默认 16 细分 = 3200 pps */
#ifndef M5_MICROSTEP
#define M5_MICROSTEP           16U
#endif
#define M5_PULSES_PER_REV      (200U * M5_MICROSTEP)
#define M5_PULSES_PER_DEG      ((float)M5_PULSES_PER_REV / 360.0f)

/* 慢速默认参数 — 3 秒/圈 = 20 RPM */
#define M5_DEFAULT_RPM         20U
#define M5_DEFAULT_ACCEL       200U    /* 0=不加减速；越大加减速越慢更平滑 */
#define M5_MAX_RPM             300U    /* 上层夹紧 */

/* EMM42 命令码 */
#define M5_CMD_ENABLE          0xF3U
#define M5_CMD_SPEED           0xF6U
#define M5_CMD_POSITION        0xFDU
#define M5_CMD_STOP            0xFEU
#define M5_CMD_SYNC            0xFFU
#define M5_CMD_CLEAR_POS       0x0AU   /* 当前位置清零: addr 0A 6D 6B */
#define M5_CHECKSUM            0x6BU

/**
 * @brief 初始化 M5 慢速控制层
 *  - 必须在 Lift_Init() 之后调用 (复用 USART3, 不再重复初始化)
 *  - 不下发使能/细分修改命令, 避免与 lift_motor 起冲突
 *  - 仅做软件角度记账归零
 */
void M5_Init(void);

/* 直接命令封装 */
bool M5_Enable(bool enable);
bool M5_StopNow(void);
bool M5_SyncStart(void);
bool M5_ClearPosition(void);

/**
 * @brief 速度模式 (慢速)
 * @param ccw 方向
 * @param rpm 0~M5_MAX_RPM (内部夹紧)
 * @param accel 加速度档位 (0=不加减速)
 */
bool M5_SetSpeed(bool ccw, uint16_t rpm, uint8_t accel);

/**
 * @brief 位置模式 (脉冲, 相对/绝对)
 */
bool M5_MovePulses(bool ccw, uint16_t rpm, uint8_t accel,
                   uint32_t pulses, bool absolute);

/**
 * @brief 按角度移动 (相对，正=CCW)
 *        采用默认慢速参数，单位: 度
 */
bool M5_MoveAngleRel(float deg);

/**
 * @brief 按角度移动到绝对位置 (软件记账)
 */
bool M5_MoveAngleAbs(float deg);

/**
 * @brief 设置定位命令 (M5_MoveAngleRel/Abs) 使用的转速
 *        默认 M5_DEFAULT_RPM(20), 范围 1~M5_MAX_RPM, 内部夹紧
 */
void M5_SetMoveRpm(uint16_t rpm);

/**
 * @brief 获取当前期望角度 (软件记账)
 */
float M5_GetAngle(void);

/**
 * @brief 串口协议分发入口 (FUNC_SERVO 子命令 0x10~0x15)
 *        子命令定义见 .c 实现注释
 */
void M5_HandleFrame(uint8_t cmd, const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __M5_MOTOR_H */
