

#include "serial_protocol.h"
#include <string.h>

/* ========== CRC16 计算 (Modbus) ========== */

uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* ========== 解析器实现 ========== */

void protocol_parser_init(ProtocolParser *p, protocol_frame_cb cb, void *user)
{
    if (p == NULL) return;
    p->idx = 0;
    p->state = 0;
    p->payload_len = 0;
    p->on_frame = cb;
    p->user = user;
}

void protocol_parser_reset(ProtocolParser *p)
{
    if (p == NULL) return;
    p->idx = 0;
    p->state = 0;
    p->payload_len = 0;
}

void protocol_parser_feed(ProtocolParser *p, const uint8_t *buf, uint16_t count)
{
    if (p == NULL || buf == NULL) return;

    for (uint16_t i = 0; i < count; i++)
    {
        uint8_t byte = buf[i];

        switch (p->state)
        {
            case 0:  /* 等待 SOF1 */
                if (byte == FRAME_SOF1)
                {
                    p->buf[0] = byte;
                    p->idx = 1;
                    p->state = 1;
                }
                break;

            case 1:  /* 等待 SOF2 */
                if (byte == FRAME_SOF2)
                {
                    p->buf[p->idx++] = byte;
                    p->state = 2;
                }
                else if (byte == FRAME_SOF1)
                {
                    /* 保持在等待 SOF2 状态，避免 A5 A5 5A 丢帧 */
                    p->buf[0] = byte;
                    p->idx = 1;
                }
                else
                {
                    p->state = 0;
                    p->idx = 0;
                }
                break;

            case 2:  /* 接收头部剩余字节 */
                p->buf[p->idx++] = byte;
                if (p->idx >= FRAME_HEADER_SIZE)
                {
                    p->payload_len = p->buf[8] | ((uint16_t)p->buf[9] << 8);

                    if (p->payload_len > PROTOCOL_MAX_PAYLOAD_BYTES)
                    {
                        p->state = 0;
                        p->idx = 0;
                    }
                    else if (p->payload_len == 0)
                    {
                        p->state = 4;
                    }
                    else
                    {
                        p->state = 3;
                    }
                }
                break;

            case 3:  /* 接收 payload */
                p->buf[p->idx++] = byte;
                if (p->idx >= FRAME_HEADER_SIZE + p->payload_len)
                {
                    p->state = 4;
                }
                break;

            case 4:  /* 接收 CRC */
                p->buf[p->idx++] = byte;
                if (p->idx >= FRAME_HEADER_SIZE + p->payload_len + FRAME_CRC_SIZE)
                {
                    uint16_t calc_crc = crc16(p->buf, FRAME_HEADER_SIZE + p->payload_len);
                    uint16_t recv_crc = p->buf[p->idx - 2] | ((uint16_t)p->buf[p->idx - 1] << 8);

                    if (calc_crc == recv_crc && p->on_frame != NULL)
                    {
                        p->on_frame(p->buf, p->idx, p->user);
                    }

                    p->state = 0;
                    p->idx = 0;
                }
                break;

            default:
                p->state = 0;
                p->idx = 0;
                break;
        }
    }
}

/* ========== 全局上位机解析器（兼容旧接口） ========== */

static ProtocolParser s_global_parser;
static uint8_t s_global_inited = 0;

static void s_ensure_inited(void)
{
    if (!s_global_inited)
    {
        protocol_parser_init(&s_global_parser, NULL, NULL);
        s_global_inited = 1;
    }
}

void protocol_set_dispatch_handler(protocol_frame_cb cb, void *user)
{
    s_ensure_inited();
    s_global_parser.on_frame = cb;
    s_global_parser.user = user;
}

void protocol_feed_stream(const uint8_t *buf, uint16_t count)
{
    s_ensure_inited();
    protocol_parser_feed(&s_global_parser, buf, count);
}
