/**
 * @file tower_crane.h
 * @brief 塔吊高层控制 - 整合舵机 + 升降电机的抓取/放料流程 (主机侧)
 *
 * 完全在主机实现 (题目要求)。提供:
 *   1) 初始化
 *   2) 原子动作 (旋转物料盘/底座、爪子开合、升降到指定高度)
 *   3) 整套抓取-放置序列 (阻塞，内部 HAL_Delay 等待舵机/电机到位)
 *   4) 串口命令入口，由 serial_protocol FUNC_SERVO 分发
 */

#ifndef __TOWER_CRANE_H
#define __TOWER_CRANE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 动作时延 (ms) ===== */
#define TC_DELAY_PLATE_MS     600U    /* 物料盘 120° 转动等待 */
#define TC_DELAY_BASE_MS      700U    /* 底座 90° 转动等待 */
#define TC_DELAY_CLAW_MS      400U    /* 爪子开合等待 */
#define TC_DELAY_LIFT_MS      1500U   /* 升降等待 (粗估) */

/**
 * @brief 初始化塔吊 (舵机 + 升降电机)
 */
void TowerCrane_Init(void);

/* ===== 原子动作 ===== */
void TowerCrane_RotatePlate(bool forward);    /* 物料盘转一格 */
void TowerCrane_PlateGoto(uint8_t slot);      /* 物料盘直接定位 */
void TowerCrane_BaseToWork(void);             /* 底座摆到 90° */
void TowerCrane_BaseToHome(void);             /* 底座回 0°  */
void TowerCrane_ClawOpen(void);
void TowerCrane_ClawClose(void);
void TowerCrane_LiftTo(float mm);
void TowerCrane_LiftStop(void);

/* ===== 完整流程 (阻塞) ===== */
/**
 * @brief 从当前工位抓取 -> 放到塔吊送料位
 * 序列:
 *   1. 爪子张开
 *   2. 底座转到工作位
 *   3. 升降到抓取高度，爪子闭合
 *   4. 升降到顶部
 *   5. 底座回原位
 *   6. 爪子张开 (放料)
 *   7. 升降到底部
 */
void TowerCrane_PickAndPlace(void);

/**
 * @brief 换下一格物料并抓取
 */
void TowerCrane_NextSlotAndPick(void);

/* 子命令定义 -> 见 serial_protocol.h 的 ProtocolServoCmd */
#include "serial_protocol.h"
#define TC_CMD_SERVO_SET_ANGLE   CMD_SERVO_SET_ANGLE
#define TC_CMD_PLATE_NEXT        CMD_SERVO_PLATE_NEXT
#define TC_CMD_PLATE_GOTO        CMD_SERVO_PLATE_GOTO
#define TC_CMD_BASE_SET          CMD_SERVO_BASE_SET
#define TC_CMD_CLAW_SET          CMD_SERVO_CLAW_SET
#define TC_CMD_LIFT_MOVE         CMD_SERVO_LIFT_MOVE
#define TC_CMD_LIFT_STOP         CMD_SERVO_LIFT_STOP
#define TC_CMD_PICK_PLACE        CMD_SERVO_PICK_PLACE
#define TC_CMD_NEXT_AND_PICK     CMD_SERVO_NEXT_PICK

/**
 * @brief 上位机 FUNC_SERVO 帧处理 (由 main.c 调用)
 */
void TowerCrane_HandleFrame(uint8_t cmd, const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __TOWER_CRANE_H */
