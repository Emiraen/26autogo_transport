/**
 * @file m5_motor.c
 * @brief M5 接口 EMM42 步进电机驱动 - 慢速 + 高精度实现
 *
 * 与 lift_motor 共享 USART3，通过不同从机地址区分 (lift=1, m5=2)。
 * 主循环串行调用，TX 期间不会与 lift 发生时序冲突。
 *
 * 协议串口子命令 (FUNC_SERVO):
 *   0x10 M5_ENABLE       payload[0]=enable(1B)
 *   0x11 M5_STOP         无 payload
 *   0x12 M5_SPEED        ccw(1B) + rpm(uint16 LE)
 *   0x13 M5_MOVE_DEG     deg_x100(int32 LE)   相对角度，单位 0.01°
 *   0x14 M5_MOVE_DEG_ABS deg_x100(int32 LE)   绝对角度
 *   0x15 M5_SET_MOVE_RPM rpm(uint16 LE)       设置定位转速 (供 0x13/0x14 使用, 不触发运动)
 */

#include "m5_motor.h"
#include "lift_motor.h"
#include "byte_utils.h"
#include "serial_protocol.h"
#include "dma.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* 调试：把每次 TX 的命令和 USART3 RX 回包 (若有) 都打到 USART1 上位机口，
 * 方便用串口助手/python 看到链路是否真有应答。 */
static void m5_dbg_dump(const char *tag, const uint8_t *data, uint16_t len)
{
    char buf[160];
    int n = snprintf(buf, sizeof(buf), "[m5 %s len=%u]:", tag, len);
    for (uint16_t i = 0; i < len && n < (int)sizeof(buf) - 4; i++) {
        n += snprintf(buf + n, sizeof(buf) - n, " %02X", data[i]);
    }
    n += snprintf(buf + n, sizeof(buf) - n, "\r\n");
    if (n > 0) DMA_UART1_TxEnqueue((const uint8_t *)buf, (uint16_t)n);
}

/* 阻塞读 USART3 一段时间, 收集 EMM42 回包 */
static uint16_t m5_rx_collect(uint8_t *buf, uint16_t maxlen, uint32_t timeout_ms)
{
    UART_HandleTypeDef *h = Lift_GetUart();
    uint16_t idx = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < timeout_ms && idx < maxlen) {
        uint8_t b;
        if (HAL_UART_Receive(h, &b, 1, 2) == HAL_OK) {
            buf[idx++] = b;
        }
    }
    return idx;
}


/* 软件记账的当前角度 (deg)，与 EMM42 内部位置同步清零 */
static float s_m5_angle_deg = 0.0f;

/* 定位命令 (0x13/0x14) 使用的转速, 可由上位机 0x15 修改 (默认 20RPM) */
static uint16_t s_m5_move_rpm = M5_DEFAULT_RPM;

/* ===== 内部工具 ===== */

static inline bool m5_tx(const uint8_t *data, uint16_t len)
{
    /* 复用 lift_motor 已经初始化好的 USART3 */
    return Lift_TxRaw(data, len);
}

/* deg → 脉冲数 (取绝对值) */
static uint32_t deg_to_pulses(float deg)
{
    if (deg < 0.0f) deg = -deg;
    return (uint32_t)(deg * M5_PULSES_PER_DEG + 0.5f);
}

/* ===== API ===== */

bool M5_Enable(bool enable)
{
    uint8_t f[6] = {
        M5_MOTOR_ADDR, M5_CMD_ENABLE, 0xAB,
        (uint8_t)(enable ? 0x01 : 0x00),
        0x00, M5_CHECKSUM
    };
    return m5_tx(f, sizeof(f));
}

bool M5_StopNow(void)
{
    uint8_t f[5] = { M5_MOTOR_ADDR, M5_CMD_STOP, 0x98, 0x00, M5_CHECKSUM };
    return m5_tx(f, sizeof(f));
}

bool M5_SyncStart(void)
{
    uint8_t f[4] = { 0x00, M5_CMD_SYNC, 0x66, M5_CHECKSUM };
    return m5_tx(f, sizeof(f));
}

bool M5_ClearPosition(void)
{
    uint8_t f[4] = { M5_MOTOR_ADDR, M5_CMD_CLEAR_POS, 0x6D, M5_CHECKSUM };
    bool ok = m5_tx(f, sizeof(f));
    if (ok) s_m5_angle_deg = 0.0f;
    return ok;
}

bool M5_SetSpeed(bool ccw, uint16_t rpm, uint8_t accel)
{
    if (rpm > M5_MAX_RPM) rpm = M5_MAX_RPM;  /* 慢速场景上限保护 */
    uint8_t f[8];
    f[0] = M5_MOTOR_ADDR;
    f[1] = M5_CMD_SPEED;
    f[2] = ccw ? 0x01 : 0x00;
    f[3] = (uint8_t)(rpm >> 8);
    f[4] = (uint8_t)(rpm & 0xFF);
    f[5] = accel;
    f[6] = 0x00;             /* 不启用同步 */
    f[7] = M5_CHECKSUM;
    return m5_tx(f, sizeof(f));
}

bool M5_MovePulses(bool ccw, uint16_t rpm, uint8_t accel,
                   uint32_t pulses, bool absolute)
{
    if (rpm > M5_MAX_RPM) rpm = M5_MAX_RPM;
    uint8_t f[13];
    f[0]  = M5_MOTOR_ADDR;
    f[1]  = M5_CMD_POSITION;
    f[2]  = ccw ? 0x01 : 0x00;
    f[3]  = (uint8_t)(rpm >> 8);
    f[4]  = (uint8_t)(rpm & 0xFF);
    f[5]  = accel;
    f[6]  = (uint8_t)(pulses >> 24);
    f[7]  = (uint8_t)((pulses >> 16) & 0xFF);
    f[8]  = (uint8_t)((pulses >> 8) & 0xFF);
    f[9]  = (uint8_t)(pulses & 0xFF);
    f[10] = absolute ? 0x01 : 0x00;
    f[11] = 0x00;            /* sync */
    f[12] = M5_CHECKSUM;
    return m5_tx(f, sizeof(f));
}

void M5_SetMoveRpm(uint16_t rpm)
{
    if (rpm == 0) rpm = 1;                 /* 防止 0 速度卡死 */
    if (rpm > M5_MAX_RPM) rpm = M5_MAX_RPM;
    s_m5_move_rpm = rpm;
}

bool M5_MoveAngleRel(float deg)
{
    if (fabsf(deg) < 0.001f) return true;
    bool ccw = (deg > 0.0f);
    uint32_t pulses = deg_to_pulses(deg);
    bool ok = M5_MovePulses(ccw, s_m5_move_rpm, M5_DEFAULT_ACCEL,
                            pulses, false /* 相对 */);
    if (ok) s_m5_angle_deg += deg;
    return ok;
}

bool M5_MoveAngleAbs(float deg)
{
    float delta = deg - s_m5_angle_deg;
    bool ok = M5_MoveAngleRel(delta);
    if (ok) s_m5_angle_deg = deg;
    return ok;
}

float M5_GetAngle(void) { return s_m5_angle_deg; }

/* 一发一收: 写 USART3, 阻塞读回包, 把发送+回包都打到 USART1 调试 */
static void m5_send_and_dump(const char *tag, const uint8_t *tx, uint16_t txlen)
{
    /* 先清空 USART3 RX 残留 */
    UART_HandleTypeDef *h = Lift_GetUart();
    __HAL_UART_CLEAR_FLAG(h, UART_FLAG_RXNE);
    (void)h->Instance->DR;

    m5_dbg_dump(tag, tx, txlen);
    Lift_TxRaw(tx, txlen);

    uint8_t rx[32] = {0};
    uint16_t n = m5_rx_collect(rx, sizeof(rx), 80);
    if (n == 0) {
        const char *msg = "  -> no reply (RX silent, check wiring/baud/addr)\r\n";
        DMA_UART1_TxEnqueue((const uint8_t *)msg, (uint16_t)strlen(msg));
    } else {
        m5_dbg_dump("rx", rx, n);
    }
}

/* 上电自检默认关闭：启动期阻塞累计 ≈2.4s 与 IWDG 超时同量级，是潜在隐患。
 * 仅在烧录调试时通过 build_flags=-DM5_INIT_SELFTEST=1 临时打开。 */
#ifndef M5_INIT_SELFTEST
#define M5_INIT_SELFTEST 0
#endif

void M5_Init(void)
{
    s_m5_angle_deg = 0.0f;

#if M5_INIT_SELFTEST
    /* ===== 上电自检 + 链路诊断 (调试用) ===== */
    HAL_Delay(200);
    {
        uint8_t f[6] = { M5_MOTOR_ADDR, M5_CMD_ENABLE, 0xAB, 0x01, 0x00, M5_CHECKSUM };
        m5_send_and_dump("enable", f, sizeof(f));
    }
    HAL_Delay(50);
    {
        uint8_t f[8] = {
            M5_MOTOR_ADDR, M5_CMD_SPEED, 0x00,
            (uint8_t)(M5_DEFAULT_RPM >> 8), (uint8_t)(M5_DEFAULT_RPM & 0xFF),
            M5_DEFAULT_ACCEL, 0x00, M5_CHECKSUM
        };
        m5_send_and_dump("speed20cw", f, sizeof(f));
    }
    HAL_Delay(2000);
    {
        uint8_t f[5] = { M5_MOTOR_ADDR, M5_CMD_STOP, 0x98, 0x00, M5_CHECKSUM };
        m5_send_and_dump("stop", f, sizeof(f));
    }
#else
    /* 正常运行：仅静默使能，不阻塞启动流程 */
    HAL_Delay(20);
    M5_Enable(true);
#endif
}

/* ===== FUNC_SERVO 协议分发 ===== */

void M5_HandleFrame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    switch (cmd)
    {
        case 0x10: /* M5_ENABLE */
            if (len >= 1) M5_Enable(payload[0] != 0);
            break;

        case 0x11: /* M5_STOP */
            M5_StopNow();
            break;

        case 0x12: /* M5_SPEED: ccw(1B) + rpm(uint16 LE) */
            if (len >= 3)
            {
                bool ccw = (payload[0] != 0);
                uint16_t rpm = read_le16(&payload[1]);
                M5_SetSpeed(ccw, rpm, M5_DEFAULT_ACCEL);
            }
            break;

        case 0x13: /* M5_MOVE_DEG (相对): deg_x100(int32 LE) */
            if (len >= 4)
            {
                int32_t deg_x100 = read_le32s(&payload[0]);
                M5_MoveAngleRel((float)deg_x100 / 100.0f);
            }
            break;

        case 0x14: /* M5_MOVE_DEG_ABS (绝对): deg_x100(int32 LE) */
            if (len >= 4)
            {
                int32_t deg_x100 = read_le32s(&payload[0]);
                M5_MoveAngleAbs((float)deg_x100 / 100.0f);
            }
            break;

        case 0x15: /* M5_SET_MOVE_RPM: rpm(uint16 LE) — 仅设置定位转速, 不运动 */
            if (len >= 2)
            {
                M5_SetMoveRpm(read_le16(&payload[0]));
            }
            break;

        default:
            break;
    }
}
