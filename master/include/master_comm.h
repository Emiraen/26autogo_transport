/**
 * @file master_comm.h
 * @brief 主机向副机通信模块 - 通过 USART2 发送运动指令
 */

#ifndef MASTER_COMM_H
#define MASTER_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* ========== 主从通信命令类型 ========== */

/* 命令码与 docs/协议文档.md 3.2 节保持一致 (0x03 是 GET_STATE，预留给副机响应) */
typedef enum {
    MCOMM_CMD_SET_VEL   = 0x01,  /**< 设置四轮速度 */
    MCOMM_CMD_SET_POS   = 0x02,  /**< 设置四轮位置 */
    MCOMM_CMD_STOP      = 0x04,  /**< 紧急停止 */
    MCOMM_CMD_ENABLE    = 0x05,  /**< 电机使能/失能 */
    MCOMM_CMD_SYNC      = 0x06,  /**< 同步启动 */
} MComm_CmdType;

/* ========== 速度指令结构 ========== */

typedef struct {
    int16_t v1;  /**< 轮1速度 (RPM, 正负表示方向) */
    int16_t v2;  /**< 轮2速度 */
    int16_t v3;  /**< 轮3速度 */
    int16_t v4;  /**< 轮4速度 */
    uint8_t accel;  /**< 加速度档位 (0=不加减速) */
} MComm_VelCmd;

/* ========== 位置指令结构 ========== */

typedef struct {
    int32_t p1;  /**< 轮1目标脉冲 */
    int32_t p2;  /**< 轮2目标脉冲 */
    int32_t p3;  /**< 轮3目标脉冲 */
    int32_t p4;  /**< 轮4目标脉冲 */
    uint16_t rpm;   /**< 运动速度 (RPM) */
    uint8_t accel;  /**< 加速度档位 */
    bool absolute;  /**< true=绝对位置, false=相对位置 */
} MComm_PosCmd;

/* ========== API 函数 ========== */

/**
 * @brief 初始化主从通信模块
 * @note 需在 USART2 初始化后调用
 */
void MComm_Init(void);

/**
 * @brief 发送速度指令到副机
 * @param cmd 速度指令结构指针
 * @return true=发送成功, false=失败
 */
bool MComm_SendVelocity(const MComm_VelCmd *cmd);

/**
 * @brief 发送位置指令到副机
 * @param cmd 位置指令结构指针
 * @return true=发送成功, false=失败
 */
bool MComm_SendPosition(const MComm_PosCmd *cmd);

/**
 * @brief 发送紧急停止指令
 * @return true=发送成功, false=失败
 */
bool MComm_SendStop(void);

/**
 * @brief 发送电机使能/失能指令
 * @param enable true=使能, false=失能
 * @return true=发送成功, false=失败
 */
bool MComm_SendEnable(bool enable);

/**
 * @brief 发送同步启动指令
 * @return true=发送成功, false=失败
 */
bool MComm_SendSync(void);

#ifdef __cplusplus
}
#endif

#endif /* MASTER_COMM_H */
