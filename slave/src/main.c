/**
 * @file main.c
 * @brief 副机主程序 - STM32F103C8T6
 * 
 * 职责：
 * - 接收主机指令 (USART2 RX, 115200)
 * - 解析运动指令 (A5 5A 协议格式)
 * - 通过 RS485 (USART3, 115200) 驱动 EMM42 电机
 * - 定时读取电机转速数据，通过 USART2 TX 回传主机
 * 
 * 防踩踏设计：
 *   所有 RS485 操作（发送运动指令 / 读取里程计）都在主循环中顺序执行，
 *   不存在并发访问 RS485 总线的可能，天然防踩踏。
 */

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "emm42_driver.h"
#include "byte_utils.h"
#include "crc16_util.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* ========== 协议定义 ========== */

#define FRAME_SOF1          0xA5U
#define FRAME_SOF2          0x5AU
#define FRAME_HEADER_SIZE   10U
#define FRAME_CRC_SIZE      2U
#define MAX_PAYLOAD_SIZE    64U

/* 命令码与协议文档 docs/协议文档.md 3.2 节严格对齐 */
#define FUNC_CHASSIS        0x02U
#define CMD_SET_VEL         0x01U  /* 上→下 */
#define CMD_SET_POS         0x02U  /* 上→下 */
#define CMD_GET_STATE       0x03U  /* 下→上 (本副机暂未实现 24B 状态查询) */
#define CMD_STOP            0x04U  /* 上→下 */
#define CMD_ENABLE          0x05U  /* 上→下 */
#define CMD_SYNC            0x06U  /* 上→下 */
#define CMD_ODOM_REPORT     0x07U  /* 下→上 转速 12B 上报 */


/* 里程计轮询间隔 (ms) - 50Hz 满足 EKF 融合需求 */
#define ODOM_POLL_INTERVAL  20U

/* 里程计上报 payload 大小: 4 × (sign1B + rpm2B) = 12 bytes */
#define ODOM_PAYLOAD_SIZE   12U

/* ========== 环形缓冲区 (中断写，主循环读) ========== */

#define RX_RING_SIZE  128U

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_ring_head = 0;  /* ISR 写入位置 */
static uint16_t rx_ring_tail = 0;           /* 主循环读取位置 */

/* ========== 帧解析状态机 ========== */

static uint8_t rx_frame[FRAME_HEADER_SIZE + MAX_PAYLOAD_SIZE + FRAME_CRC_SIZE];
static uint16_t rx_idx = 0;
static uint8_t rx_state = 0;
static uint16_t rx_payload_len = 0;

/* ========== 转速缓存 (超时容错：读取失败时使用上一周期值) ========== */

static uint8_t  cached_signs[EMM42_MOTOR_COUNT] = {0};
static uint16_t cached_rpms[EMM42_MOTOR_COUNT]  = {0};

/* ========== IWDG 看门狗 ========== */

static IWDG_HandleTypeDef hiwdg;

static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    /* LSI ≈ 40kHz，预分频 64，重载 1500 → 超时 ≈ 2.4s */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1500;
    (void)HAL_IWDG_Init(&hiwdg);
}

static inline void IWDG_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* ========== ERR Report (FUNC=0x01, CMD=0x01) ========== */
#define FUNC_SYSTEM       0x01U
#define CMD_SYS_ERROR     0x01U
#define ERR_MAX_MSG_LEN   60U

static void ERR_Report(const char *msg)
{
    uint16_t msg_len = 0;
    while (msg[msg_len] && msg_len < ERR_MAX_MSG_LEN) msg_len++;
    uint16_t payload_len = 4U + msg_len;
    uint8_t frame[10 + 4 + ERR_MAX_MSG_LEN + 2];
    static uint8_t err_seq = 0;
    uint32_t tick = HAL_GetTick();
    frame[0] = FRAME_SOF1; frame[1] = FRAME_SOF2;
    frame[2] = 0x10; frame[3] = err_seq++;
    frame[4] = 0x02; frame[5] = 0x00;
    frame[6] = FUNC_SYSTEM; frame[7] = CMD_SYS_ERROR;
    write_le16(&frame[8], payload_len);
    frame[10] = (uint8_t)(tick & 0xFF);
    frame[11] = (uint8_t)((tick >> 8) & 0xFF);
    frame[12] = (uint8_t)((tick >> 16) & 0xFF);
    frame[13] = (uint8_t)((tick >> 24) & 0xFF);
    memcpy(&frame[14], msg, msg_len);
    uint16_t crc = crc16_modbus(frame, 10 + payload_len);
    write_le16(&frame[10 + payload_len], crc);
    HAL_UART_Transmit(&huart2, frame, 10 + payload_len + 2, 20);
}

/* ========== 私有函数声明 ========== */

static void ProcessFrame(const uint8_t *frame, uint16_t len);
static void FeedByte(uint8_t byte);
static void Odom_PollAndReport(void);
static void Odom_SendSpeedReport(const uint8_t signs[], const uint16_t rpms[]);

/* ========== 系统时钟配置 ========== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========== 主函数 ========== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    
    /* EMM42 驱动初始化 */
    EMM42_Init(&huart3);
    
    /* RS485 默认接收模式 */
    RS485_SetRxMode();
    
    /* 上电逐个使能 4 个电机 (与 chassis3 一致的稳妥方案) */
    EMM42_Enable(EMM42_MOTOR_FL, true, false); HAL_Delay(20);
    EMM42_Enable(EMM42_MOTOR_FR, true, false); HAL_Delay(20);
    EMM42_Enable(EMM42_MOTOR_RL, true, false); HAL_Delay(20);
    EMM42_Enable(EMM42_MOTOR_RR, true, false); HAL_Delay(100);
    
    /* 清掉残留错误位再启动接收，避免外设被先前噪声触发的 ORE 卡住 */
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    (void)huart2.Instance->SR;
    (void)huart2.Instance->DR;

    /* 启动 USART2 接收中断 */
    HAL_UART_Receive_IT(&huart2, usart2_rx_buf, 1);
    
    /* LED 闪烁表示启动成功 */
    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(100);
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /* 启动看门狗（放在外设/通信启动后，避免启动期饿死） */
    MX_IWDG_Init();

    uint32_t last_health_tick = 0;
    uint32_t last_odom_tick   = 0;
    uint32_t last_uart_health_tick = 0;
    uint32_t last_enable_hb_tick = 0;
    
    while (1)
    {
        /* ---- 1. 处理主机下发的指令 ---- */
        while (rx_ring_tail != rx_ring_head)
        {
            uint8_t byte = rx_ring[rx_ring_tail];
            rx_ring_tail = (rx_ring_tail + 1) % RX_RING_SIZE;
            FeedByte(byte);
        }
        
        uint32_t now = HAL_GetTick();
        
        /* ---- 2. 定时轮询里程计并上报 ---- */
        if (now - last_odom_tick >= ODOM_POLL_INTERVAL)
        {
            last_odom_tick = now;
            Odom_PollAndReport();
        }
        
        /* ---- 3. USART2 RX 健康检查（兜底，500ms 一次）----
         * IRQ 抢先清错误位 + ErrorCallback 自动重启已经能覆盖 99% 的场景。
         * 这里再加一道兜底：定期检查 RxState 是否仍处于 BUSY_RX，
         * 任何异常都强制重启接收，使主从链路在断电重连、长时间静默、
         * 突发干扰等极端工况下也必然恢复。 */
        if (now - last_uart_health_tick >= 500)
        {
            last_uart_health_tick = now;

            extern volatile uint8_t g_usart2_rx_need_restart;
            bool need_restart = (g_usart2_rx_need_restart != 0);

            if (huart2.RxState != HAL_UART_STATE_BUSY_RX &&
                huart2.RxState != HAL_UART_STATE_BUSY_TX_RX)
            {
                need_restart = true;
            }
            else if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
            {
                /* 清掉历史 ORE 标志，下一字节接收时不会再卡 HAL */
                __HAL_UART_CLEAR_PEFLAG(&huart2);
                (void)huart2.Instance->DR;
            }

            if (need_restart)
            {
                /* 改用 NVIC 局部禁用替代 __disable_irq()：
                 * - HAL_UART_AbortReceive_IT 内部会改 huart2 状态机并依赖 NVIC
                 *   挂起标志，全局关中断期间调用是 HAL 不规范用法，极端工况下
                 *   会和未及时进入的 USART2_IRQ 错位。
                 * - 只屏蔽 USART2_IRQn 即可保证 RX 临界区，其它中断照常工作。
                 */
                HAL_NVIC_DisableIRQ(USART2_IRQn);
                HAL_UART_AbortReceive_IT(&huart2);
                __HAL_UART_CLEAR_PEFLAG(&huart2);
                (void)huart2.Instance->SR;
                (void)huart2.Instance->DR;
                /* 重置环形缓冲指针，丢弃残帧避免脏数据 */
                rx_ring_head = 0;
                rx_ring_tail = 0;
                rx_state = 0;
                rx_idx = 0;
                HAL_NVIC_EnableIRQ(USART2_IRQn);
                HAL_UART_Receive_IT(&huart2, usart2_rx_buf, 1);
                g_usart2_rx_need_restart = 0;
            }
        }

        /* ---- 4. EMM42 使能心跳 (500ms) ----
         * 实测中, 用户中途切断 EMM42 的 12V 再上电后, 驱动器复位为 disabled,
         * 后续的 SetVel / Move / Stop 帧到达驱动器后会被忽略, 必须重新发
         * enable 才能恢复, 现象上"必须重启 MCU 才能继续控制"。
         * 这里每 500ms 主动用广播地址重发一次 enable=1, 12V 恢复后最迟
         * 1 秒内 4 个电机自动重新使能, 不再需要复位 MCU。
         * EMM42_Enable(BROADCAST) 本身不会触发任何运动。
         *
         * 时序考虑: Odom_PollAndReport 每 20ms 占用 RS485, 我们紧接着发,
         * 同一主循环串行执行, 不会有总线竞争。
         */
        if (now - last_enable_hb_tick >= 500)
        {
            last_enable_hb_tick = now;
            EMM42_Enable(EMM42_BROADCAST, true, false);
        }

        /* ---- 5. 1Hz 心跳 LED + alive 打印 ---- */
        if (now - last_health_tick >= 1000)
        {
            last_health_tick = now;
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            /* 通过 USART2 TX 发送 alive 给主机，主机会透传到上位机 */
            char alive_buf[40];
            int alive_len = snprintf(alive_buf, sizeof(alive_buf),
                                     "stm322alive T=%lums\r\n", (unsigned long)now);
            if (alive_len > 0)
                HAL_UART_Transmit(&huart2, (uint8_t *)alive_buf, (uint16_t)alive_len, 10);
        }

        /* ---- 6. 喂狗 ---- */
        IWDG_Feed();
    }
}

/* ========== 帧处理 ========== */

static void ProcessFrame(const uint8_t *frame, uint16_t len)
{
    if (len < FRAME_HEADER_SIZE + FRAME_CRC_SIZE)
        return;
    
    uint8_t func = frame[6];
    uint8_t cmd = frame[7];
    uint16_t payload_len = read_le16(&frame[8]);
    const uint8_t *payload = frame + FRAME_HEADER_SIZE;
    
    if (func == FUNC_CHASSIS)
    {
        switch (cmd)
        {
            case CMD_SET_VEL:
                if (payload_len >= 8)
                {
                    int16_t v1 = read_le16s(&payload[0]);
                    int16_t v2 = read_le16s(&payload[2]);
                    int16_t v3 = read_le16s(&payload[4]);
                    int16_t v4 = read_le16s(&payload[6]);
                    uint8_t accel = (payload_len >= 9) ? payload[8] : 0;
                    
                    EMM42_SetWheelSpeeds(v1, v2, v3, v4, accel);
                }
                break;
                
            case CMD_SET_POS:
                if (payload_len >= 20)
                {
                    int32_t p1 = read_le32s(&payload[0]);
                    int32_t p2 = read_le32s(&payload[4]);
                    int32_t p3 = read_le32s(&payload[8]);
                    int32_t p4 = read_le32s(&payload[12]);
                    uint16_t rpm = read_le16(&payload[16]);
                    uint8_t accel = payload[18];
                    bool absolute = (payload[19] != 0);
                    
                    bool ccw1 = (p1 >= 0);
                    bool ccw2 = (p2 >= 0);
                    bool ccw3 = (p3 >= 0);
                    bool ccw4 = (p4 >= 0);
                    
                    EMM42_MovePosition(EMM42_MOTOR_FL, ccw1, rpm, accel, 
                                       (uint32_t)(ccw1 ? p1 : -p1), absolute, true);
                    EMM42_MovePosition(EMM42_MOTOR_FR, ccw2, rpm, accel, 
                                       (uint32_t)(ccw2 ? p2 : -p2), absolute, true);
                    EMM42_MovePosition(EMM42_MOTOR_RL, ccw3, rpm, accel, 
                                       (uint32_t)(ccw3 ? p3 : -p3), absolute, true);
                    EMM42_MovePosition(EMM42_MOTOR_RR, ccw4, rpm, accel, 
                                       (uint32_t)(ccw4 ? p4 : -p4), absolute, true);
                    
                    EMM42_SyncStart();
                }
                break;
                
            case CMD_STOP:
                EMM42_StopNow(EMM42_BROADCAST, false);
                break;
                
            case CMD_ENABLE:
                if (payload_len >= 1)
                {
                    bool enable = (payload[0] != 0);
                    EMM42_Enable(EMM42_BROADCAST, enable, false);
                }
                break;
                
            case CMD_SYNC:
                EMM42_SyncStart();
                break;
                
            default:
                break;
        }
    }
}

/* ========== 字节流状态机 ========== */

static void FeedByte(uint8_t byte)
{
    switch (rx_state)
    {
        case 0:
            if (byte == FRAME_SOF1)
            {
                rx_frame[0] = byte;
                rx_idx = 1;
                rx_state = 1;
            }
            break;
            
        case 1:
            if (byte == FRAME_SOF2)
            {
                rx_frame[rx_idx++] = byte;
                rx_state = 2;
            }
            else
            {
                rx_state = 0;
            }
            break;
            
        case 2:
            rx_frame[rx_idx++] = byte;
            if (rx_idx >= FRAME_HEADER_SIZE)
            {
                rx_payload_len = read_le16(&rx_frame[8]);
                
                if (rx_payload_len > MAX_PAYLOAD_SIZE)
                    rx_state = 0;
                else if (rx_payload_len == 0)
                    rx_state = 4;
                else
                    rx_state = 3;
            }
            break;
            
        case 3:
            rx_frame[rx_idx++] = byte;
            if (rx_idx >= FRAME_HEADER_SIZE + rx_payload_len)
                rx_state = 4;
            break;
            
        case 4:
            rx_frame[rx_idx++] = byte;
            if (rx_idx >= FRAME_HEADER_SIZE + rx_payload_len + FRAME_CRC_SIZE)
            {
                uint16_t calc_crc = crc16_modbus(rx_frame, FRAME_HEADER_SIZE + rx_payload_len);
                uint16_t recv_crc = read_le16(&rx_frame[rx_idx - 2]);
                
                if (calc_crc == recv_crc)
                    ProcessFrame(rx_frame, rx_idx);
                
                rx_state = 0;
                rx_idx = 0;
            }
            break;
            
        default:
            rx_state = 0;
            break;
    }
}

/* ========== 里程计轮询与上报 (转速方案) ========== */

/**
 * @brief 逐个轮询 4 个电机转速，失败的电机使用上一周期缓存值，然后上报
 * 
 * 容错策略：单个电机超时不会阻塞其他电机的读取，也不会跳过整轮上报。
 * 超时的电机沿用 cached_signs / cached_rpms 中的上一周期值。
 */
static void Odom_PollAndReport(void)
{
    uint8_t  signs[EMM42_MOTOR_COUNT];
    uint16_t rpms[EMM42_MOTOR_COUNT];
    static const uint8_t addrs[EMM42_MOTOR_COUNT] = {
        EMM42_MOTOR_FL, EMM42_MOTOR_FR, EMM42_MOTOR_RL, EMM42_MOTOR_RR
    };

    for (uint8_t i = 0; i < EMM42_MOTOR_COUNT; i++)
    {
        EMM42_Status st = EMM42_ReadSpeed(addrs[i], &signs[i], &rpms[i]);
        if (st == EMM42_OK)
        {
            /* 读取成功，更新缓存 */
            cached_signs[i] = signs[i];
            cached_rpms[i]  = rpms[i];
        }
        else
        {
            /* 读取失败，使用上一周期缓存值 */
            signs[i] = cached_signs[i];
            rpms[i]  = cached_rpms[i];
        }
    }

    Odom_SendSpeedReport(signs, rpms);
}

/**
 * @brief 构造 A5 5A 协议帧并通过 USART2 发送转速数据给主机
 * 
 * 帧格式:
 *   SOF1(A5) + SOF2(5A) + cfg(0) + seq + src(0x02副机) + dst(0x01主机)
 *   + func(FUNC_CHASSIS=0x02) + cmd(CMD_ODOM_REPORT=0x07)
 *   + len(12, little-endian) + payload(12B) + CRC16(2B)
 * 
 * Payload (12 bytes):
 *   sign1(1B) + rpm1_h(1B) + rpm1_l(1B)
 *   + sign2(1B) + rpm2_h(1B) + rpm2_l(1B)
 *   + sign3(1B) + rpm3_h(1B) + rpm3_l(1B)
 *   + sign4(1B) + rpm4_h(1B) + rpm4_l(1B)
 */
static void Odom_SendSpeedReport(const uint8_t signs[], const uint16_t rpms[])
{
    static uint8_t seq = 0;
    
    uint8_t buf[FRAME_HEADER_SIZE + ODOM_PAYLOAD_SIZE + FRAME_CRC_SIZE];
    
    /* 帧头 */
    buf[0] = FRAME_SOF1;
    buf[1] = FRAME_SOF2;
    buf[2] = 0x00U;          /* cfg: 无需ACK */
    buf[3] = seq++;           /* seq */
    buf[4] = 0x02U;          /* src: 副机 */
    buf[5] = 0x01U;          /* dst: 主机 */
    buf[6] = FUNC_CHASSIS;   /* func */
    buf[7] = CMD_ODOM_REPORT;/* cmd */
    write_le16(&buf[8], ODOM_PAYLOAD_SIZE);
    
    /* payload: 4个电机的 sign + rpm(大端) */
    uint16_t offset = FRAME_HEADER_SIZE;
    for (uint8_t i = 0; i < EMM42_MOTOR_COUNT; i++)
    {
        buf[offset++] = signs[i];
        buf[offset++] = (uint8_t)(rpms[i] >> 8);
        buf[offset++] = (uint8_t)(rpms[i] & 0xFF);
    }
    
    /* CRC16 覆盖 header + payload */
    uint16_t crc = crc16_modbus(buf, FRAME_HEADER_SIZE + ODOM_PAYLOAD_SIZE);
    write_le16(&buf[offset], crc);
    offset += 2;
    
    /* 通过 USART2 TX 发送给主机 (USART2 是普通串口，非 RS485，无方向控制) */
    HAL_UART_Transmit(&huart2, buf, offset, 10);
}

/* ========== UART 接收回调 (仅缓存到环形缓冲区) ========== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint16_t next_head = (rx_ring_head + 1) % RX_RING_SIZE;
        if (next_head != rx_ring_tail)
        {
            rx_ring[rx_ring_head] = usart2_rx_buf[0];
            rx_ring_head = next_head;
        }
        HAL_UART_Receive_IT(&huart2, usart2_rx_buf, 1);
    }
}

/* ========== 错误处理 ========== */

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (volatile uint32_t i = 0; i < 100000; i++);
    }
}
