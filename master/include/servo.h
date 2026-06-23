/**
 * @file servo.h
 * @brief DS3225 数字舵机 PWM 驱动 (3 路)
 *
 * 硬件:
 *   - 物料盘: TIM3_CH2  (PA7)
 *   - 塔吊底座: TIM3_CH1 (PA6)
 *   - 爪子    : TIM1_CH1 (PA8)
 *
 * 参数:
 *   - PWM 周期 20ms (50Hz)
 *   - 脉宽 500us~2500us 对应 0~270°
 *   - 计时器时钟设为 1MHz (PSC=71)，ARR=19999 → 20ms 周期
 */

#ifndef __SERVO_H
#define __SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 舵机标识 */
typedef enum {
    SERVO_ID_PLATE = 0,   /* 物料盘 (PA7 TIM3_CH2) */
    SERVO_ID_BASE  = 1,   /* 塔吊底座 (PA6 TIM3_CH1) */
    SERVO_ID_CLAW  = 2,   /* 爪子 (PA8 TIM1_CH1) */
    SERVO_ID_COUNT
} ServoId;

/* 舵机参数常量 */
#define SERVO_PWM_FREQ_HZ        50U
#define SERVO_PULSE_MIN_US       500U     /* 0°   */
#define SERVO_PULSE_MAX_US       2500U    /* 270° */
#define SERVO_ANGLE_MAX_DEG      270.0f
/* 每度脉宽 us */
#define SERVO_US_PER_DEG         ((SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / SERVO_ANGLE_MAX_DEG)

/* 物料盘步进角 (3 工位 → 120°/工位)
 * PLATE_HOME_DEG = test_servo_alive.py 中 PLATE 的"归位位置"
 *   即 logical 0° 对应的 PWM 角 = servo_zero_offsets.json 中 PLATE 的 zero_pwm_deg
 *
 * PLATE_ANGLE_MAX_DEG: 物料盘单独的钳位上限,放开通用 270° 限位
 *   (BASE/CLAW 仍走 SERVO_ANGLE_MAX_DEG=270° 的安全限位)
 */
#define PLATE_SLOT_COUNT         3U
#define PLATE_STEP_DEG           120.0f
#define PLATE_HOME_DEG           142.7f       /* 初始位置 */
#define PLATE_ANGLE_MAX_DEG      360.0f       /* 物料盘上限,>270° */

/* 塔吊底座两个工位
 * BASE_HOME_DEG = test_servo_alive.py 中 BASE 的"归位位置"
 *   即 logical 0° 对应的 PWM 角 = servo_zero_offsets.json 中 BASE 的 zero_pwm_deg
 *   (servo 1: zero_pwm = 100°)
 */
#define BASE_HOME_DEG            81.6f     /* 初始位置 (= zero_pwm 100°) */
#define BASE_WORK_DEG            264.0f      /* 工作位置 */

/* 爪子默认开合角 (可通过 Servo_SetClawAngles 修改) */
#define CLAW_OPEN_DEG_DEFAULT    187.0f
#define CLAW_CLOSE_DEG_DEFAULT   125.0f

/**
 * @brief 初始化 3 路舵机 PWM (TIM1 + TIM3)
 * @note  内部完成 GPIO 复用、TIMx 配置与启动
 */
void Servo_Init(void);

/**
 * @brief 直接以角度设置某一舵机
 * @param id    舵机 ID
 * @param deg   目标角度 0~270
 * @return true 成功
 */
bool Servo_SetAngle(ServoId id, float deg);

/**
 * @brief 直接以脉宽设置 (us)，越界自动钳位
 */
bool Servo_SetPulseUs(ServoId id, uint16_t pulse_us);

/**
 * @brief 获取当前缓存角度
 */
float Servo_GetAngle(ServoId id);

/* ====== 物料盘动作 ====== */
/**
 * @brief 物料盘旋转一格 (120°)
 * @param forward true=正向，false=反向
 */
void Servo_PlateNextSlot(bool forward);

/**
 * @brief 直接旋转到指定工位 0/1/2
 */
void Servo_PlateGotoSlot(uint8_t slot_index);

/**
 * @brief 获取当前物料盘工位 (0/1/2)
 */
uint8_t Servo_PlateGetSlot(void);

/* ====== 塔吊底座 ====== */
/**
 * @brief true=工作位 (90°), false=初始位 (0°)
 */
void Servo_BaseSetWorking(bool working);

/* ====== 爪子 ====== */
/**
 * @brief 配置爪子开/合两种角度
 */
void Servo_SetClawAngles(float open_deg, float close_deg);

/**
 * @brief 爪子开合
 * @param close true=闭合, false=张开
 */
void Servo_ClawSet(bool close);

bool Servo_ClawIsClosed(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H */
