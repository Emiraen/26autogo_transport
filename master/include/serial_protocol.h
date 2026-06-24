/**
 * @file serial_protocol.h
 * @brief 主机协议定义 - 上位机通信协议
 *
 * 设计要点：
 *   - 解析器采用上下文结构 (ProtocolParser)，可创建多个独立实例（上位机/副机透传）。
 *   - 帧分发改为函数指针注册模式，避免弱符号链接顺序问题。
 */

#ifndef __SERIAL_PROTOCOL_H
#define __SERIAL_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*--------------------------------- 帧常量 ----------------------------------*/
#define FRAME_SOF1          (0xA5U)
#define FRAME_SOF2          (0x5AU)
#define FRAME_CRC_SIZE      (2U)
#define FRAME_CFG_NEED_ACK  (0x01U)
#define FRAME_CFG_IS_ACK    (0x02U)

/*--------------------------------- 帧结构 ----------------------------------*/
#pragma pack(push, 1)
typedef struct {
    uint8_t sof1;
    uint8_t sof2;
    uint8_t cfg;
    uint8_t seq;
    uint8_t src;
    uint8_t dst;
    uint8_t func;
    uint8_t cmd;
    uint16_t len;
} FrameHeader;
#pragma pack(pop)

#define FRAME_HEADER_SIZE   (sizeof(FrameHeader))
#define PROTOCOL_MAX_FRAME_BYTES    (256U)
#define PROTOCOL_MAX_PAYLOAD_BYTES  (PROTOCOL_MAX_FRAME_BYTES - FRAME_HEADER_SIZE - FRAME_CRC_SIZE)

/*--------------------------------- ID 枚举 ----------------------------------*/
typedef enum {
    FUNC_SYS     = 0x01U,
    FUNC_CHASSIS = 0x02U,
    FUNC_SERVO   = 0x03U,
    FUNC_SENSOR  = 0x04U,
    FUNC_OLED    = 0x05U,
} ProtocolFuncId;

typedef enum {
    CMD_CHASSIS_SET_VEL   = 0x01U,
    CMD_CHASSIS_SET_POS   = 0x02U,
    CMD_CHASSIS_GET_STATE = 0x03U,
    CMD_CHASSIS_STOP      = 0x04U,
    CMD_CHASSIS_ENABLE    = 0x05U,
    CMD_CHASSIS_SYNC      = 0x06U,
} ProtocolChassisCmd;

/* FUNC_SERVO (0x03) 子命令 —— 塔吊/舵机/升降控制 */
typedef enum {
    CMD_SERVO_SET_ANGLE    = 0x01U,   /* id(1B) + angle_x10(int16 LE) */
    CMD_SERVO_PLATE_NEXT   = 0x02U,   /* forward(1B) */
    CMD_SERVO_PLATE_GOTO   = 0x03U,   /* slot(1B) */
    CMD_SERVO_BASE_SET     = 0x04U,   /* working(1B) */
    CMD_SERVO_CLAW_SET     = 0x05U,   /* close(1B) */
    CMD_SERVO_LIFT_MOVE    = 0x06U,   /* height_mm_x10(int16 LE) */
    CMD_SERVO_LIFT_STOP    = 0x07U,
    CMD_SERVO_PICK_PLACE   = 0x08U,
    CMD_SERVO_NEXT_PICK    = 0x09U,
} ProtocolServoCmd;

/* FUNC_OLED (0x05) 子命令 —— OLED 显示控制 */
typedef enum {
    CMD_OLED_CLEAR     = 0x01U,   /* 无 payload, 清屏 */
    CMD_OLED_SHOW_TEXT = 0x02U,   /* page(1B) + col(1B) + ASCII 文本(≤ 32B) */
} ProtocolOledCmd;


/* 完整帧回调 (CRC 已校验) */
typedef void (*protocol_frame_cb)(const uint8_t *frame_buf, uint16_t frame_len, void *user);

typedef struct {
    uint8_t  buf[PROTOCOL_MAX_FRAME_BYTES];
    uint16_t idx;
    uint8_t  state;
    uint16_t payload_len;
    protocol_frame_cb on_frame;
    void    *user;
} ProtocolParser;

/* CRC16 (Modbus) */
uint16_t crc16(const uint8_t *buf, uint16_t len);

/* 解析器 API */
void protocol_parser_init(ProtocolParser *p, protocol_frame_cb cb, void *user);
void protocol_parser_feed(ProtocolParser *p, const uint8_t *buf, uint16_t count);
void protocol_parser_reset(ProtocolParser *p);

/*------------------- 兼容旧接口（全局上位机解析器） -------------------*/
/* 注册上位机帧分发回调（替代弱函数链接方式） */
void protocol_set_dispatch_handler(protocol_frame_cb cb, void *user);
/* 喂入字节流到全局解析器 */
void protocol_feed_stream(const uint8_t *buf, uint16_t count);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_PROTOCOL_H */
