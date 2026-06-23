/**
 * @file master_comm.c
 * @brief 主机向副机通信模块实现 - 通过 USART2 DMA 队列发送运动指令
 *
 * 改造点：
 *   - 删除重复的 MComm_CalcCRC16，统一使用 serial_protocol 的 crc16
 *   - 改用 DMA TX 队列 (DMA_UART2_TxEnqueue)，避免阻塞主循环
 *   - 字节序使用 byte_utils 工具函数
 */

#include "master_comm.h"
#include "usart.h"
#include "dma.h"
#include "serial_protocol.h"
#include "byte_utils.h"
#include <string.h>

/* ========== 内部定义 ========== */

#define MCOMM_SOF1          0xA5
#define MCOMM_SOF2          0x5A
#define MCOMM_FUNC_CHASSIS  0x02
#define MCOMM_ADDR_MASTER   0x01
#define MCOMM_ADDR_SLAVE    0x10

static uint8_t g_seq = 0;  /* 帧序号 */

/* ========== 内部函数 ========== */

/**
 * @brief 打包并通过 USART2 DMA 队列发送帧到副机（非阻塞）
 */
static bool MComm_SendFrame(uint8_t cmd, const uint8_t *payload, uint16_t payload_len)
{
    uint8_t buf[64];
    uint16_t total_len = 10 + payload_len + 2;

    if (total_len > sizeof(buf))
    {
        return false;
    }

    buf[0] = MCOMM_SOF1;
    buf[1] = MCOMM_SOF2;
    buf[2] = 0x10;  /* CFG: 版本1, 不需要ACK */
    buf[3] = g_seq++;
    buf[4] = MCOMM_ADDR_MASTER;
    buf[5] = MCOMM_ADDR_SLAVE;
    buf[6] = MCOMM_FUNC_CHASSIS;
    buf[7] = cmd;
    write_le16(&buf[8], payload_len);

    if (payload != NULL && payload_len > 0)
    {
        memcpy(&buf[10], payload, payload_len);
    }

    uint16_t crc = crc16(buf, 10 + payload_len);
    write_le16(&buf[10 + payload_len], crc);

    /* 非阻塞入队 DMA TX FIFO；队列满则丢弃当前帧（避免阻塞主循环）。
     * 上层若要确认发送结果可检查返回值，必要时记录丢帧计数。 */
    return DMA_UART2_TxEnqueue(buf, total_len);
}

/* ========== API 实现 ========== */

void MComm_Init(void)
{
    g_seq = 0;
}

bool MComm_SendVelocity(const MComm_VelCmd *cmd)
{
    if (cmd == NULL) return false;

    uint8_t payload[9];
    write_le16(&payload[0], (uint16_t)cmd->v1);
    write_le16(&payload[2], (uint16_t)cmd->v2);
    write_le16(&payload[4], (uint16_t)cmd->v3);
    write_le16(&payload[6], (uint16_t)cmd->v4);
    payload[8] = cmd->accel;

    return MComm_SendFrame(MCOMM_CMD_SET_VEL, payload, sizeof(payload));
}

bool MComm_SendPosition(const MComm_PosCmd *cmd)
{
    if (cmd == NULL) return false;

    uint8_t payload[20];
    write_le32(&payload[0],  (uint32_t)cmd->p1);
    write_le32(&payload[4],  (uint32_t)cmd->p2);
    write_le32(&payload[8],  (uint32_t)cmd->p3);
    write_le32(&payload[12], (uint32_t)cmd->p4);
    write_le16(&payload[16], cmd->rpm);
    payload[18] = cmd->accel;
    payload[19] = cmd->absolute ? 0x01 : 0x00;

    return MComm_SendFrame(MCOMM_CMD_SET_POS, payload, sizeof(payload));
}

bool MComm_SendStop(void)
{
    return MComm_SendFrame(MCOMM_CMD_STOP, NULL, 0);
}

bool MComm_SendEnable(bool enable)
{
    uint8_t payload = enable ? 0x01 : 0x00;
    return MComm_SendFrame(MCOMM_CMD_ENABLE, &payload, 1);
}

bool MComm_SendSync(void)
{
    return MComm_SendFrame(MCOMM_CMD_SYNC, NULL, 0);
}
