/**
 * @file lift_motor.h
 * @brief 升降 EMM42 闭环步进电机驱动 (主机 USART3 / M5 接口)
 *
 * 通讯：USART3 (PB10/PB11) 115200 8N1，直连 EMM42 (非 RS485)
 * 协议：addr + cmd + data + 0x6B
 *
 * 配置：
 *   - 16 细分: 3200 脉冲 = 360°
 *   - 默认地址 1 (可改)
 *
 * 升降高度暂用宏定义占位：
 *   - LIFT_HEIGHT_TOP_MM / LIFT_HEIGHT_BOTTOM_MM
 *   - LIFT_MM_PER_REV   每圈丝杠导程 mm
 */

#ifndef __LIFT_MOTOR_H
#define __LIFT_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 可配置宏 ===== */
#define LIFT_MOTOR_ADDR        5U      /* M5 接口升降电机的 EMM42 从机地址 */
#define LIFT_MICROSTEP         16U
#define LIFT_PULSES_PER_REV    (200U * LIFT_MICROSTEP)   /* = 3200 */
#define LIFT_MM_PER_REV        8.0f                       /* 丝杠导程占位 */
#define LIFT_DEFAULT_RPM       300U
#define LIFT_DEFAULT_ACCEL     200U

/* 升降高度位姿占位 (mm)
 *  - 取料盘与塔吊最底端持平 ⇒ 取料位 = BOTTOM
 *  - 默认空闲状态钩爪在最上方 (TOP)
 */
#define LIFT_HEIGHT_BOTTOM_MM  0.0f       /* 料盘平面，取料高度 */
#define LIFT_HEIGHT_PICK_MM    LIFT_HEIGHT_BOTTOM_MM
#define LIFT_HEIGHT_TOP_MM     150.0f     /* 默认/待机高度，送料位 */

/* EMM42 命令码 (与副机驱动保持一致) */
#define LIFT_CMD_ENABLE     0xF3U
#define LIFT_CMD_SPEED      0xF6U
#define LIFT_CMD_POSITION   0xFDU
#define LIFT_CMD_STOP       0xFEU
#define LIFT_CMD_SYNC       0xFFU
#define LIFT_CHECKSUM       0x6BU

/**
 * @brief 初始化升降电机驱动 (USART3 + GPIO + 默认使能)
 */
void Lift_Init(void);

/* 直接命令封装 */
bool Lift_Enable(bool enable);
bool Lift_StopNow(void);
bool Lift_SyncStart(void);

/**
 * @brief 速度模式
 * @param ccw 方向 (true=CCW)
 * @param rpm 0~3000
 * @param accel 0~255 (0=不加减速)
 * @param sync 多机同步
 */
bool Lift_SetSpeed(bool ccw, uint16_t rpm, uint8_t accel, bool sync);

/**
 * @brief 位置模式 (脉冲)
 */
bool Lift_MovePulses(bool ccw, uint16_t rpm, uint8_t accel,
                     uint32_t pulses, bool absolute, bool sync);

/**
 * @brief 高度移动到绝对位置 (mm)
 * @note  内部按 LIFT_MM_PER_REV 与 LIFT_PULSES_PER_REV 换算
 *        约定: 高度增大 ⇒ CCW 方向，可在实现中调整
 */
bool Lift_MoveToHeight(float height_mm);

/**
 * @brief 高度相对移动 (正数=升高)
 */
bool Lift_MoveRelative(float delta_mm);

/* 获取当前期望高度 (软件记账值) */
float Lift_GetHeight(void);

/**
 * @brief 获取 USART3 UART 句柄 (供同总线其他从机驱动复用, 如 m5_motor)
 *        必须在 Lift_Init() 之后调用，否则返回未初始化的句柄。
 */
UART_HandleTypeDef *Lift_GetUart(void);

/**
 * @brief 通过 USART3 同步发送原始字节 (供同总线其他从机驱动使用)
 *        线程不安全：主循环串行调用即可。
 */
bool Lift_TxRaw(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_MOTOR_H */
