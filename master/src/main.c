/**
 * @file main.c
 * @brief 主机工程 - 主程序
 *
 * 功能：
 * 1. USART1 (230400) DMA 接收上位机协议帧，DMA FIFO 发送响应
 * 2. 解析后通过 USART2 DMA (115200) 转发指令给副机
 * 3. I2C1 轮询 JY901B 陀螺仪 (200Hz)
 * 4. 主动 30Hz 上报 IMU 数据；同时兼容上位机 GET_IMU 请求
 * 5. 副机上报数据过 CRC 校验后透传给上位机（防止残帧）
 * 6. LED 心跳 + IWDG 看门狗 + 启动时通过 USART1 打印版本/编译时间
 */

#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "i2c.h"
#include "JY901B.h"
#include "serial_protocol.h"
#include "master_comm.h"
#include "tower_crane.h"
#include "m5_motor.h"
#include "byte_utils.h"
#include <string.h>
#include <stdio.h>

void SystemClock_Config(void);

/* ========== 固件版本 ========== */
#define FIRMWARE_VERSION  "chassis v1.1.0"

/* ========== 传感器命令定义 ========== */
#define CMD_SENSOR_GET_IMU  (0x01U)

/* ========== IWDG 看门狗 ==========
 * 把 hiwdg / IWDG_Feed 改成全局可见，便于 tower_crane 等长流程模块
 * 在阻塞延时期间主动喂狗，避免 2.4s 超时复位。
 */
IWDG_HandleTypeDef hiwdg;

static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG;
    /* LSI ≈ 40kHz，预分频 64，重载 1500 → 超时 ≈ 2.4s */
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    hiwdg.Init.Reload = 1500;
    (void)HAL_IWDG_Init(&hiwdg);
}

void IWDG_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

/* ========== IMU 数据结构布局断言 ========== */
/* 保证 JY901B_Data 是 9 个连续的 float（36B 无 padding） */
_Static_assert(sizeof(JY901B_Data) == 9 * sizeof(float),
               "JY901B_Data must be 9 contiguous floats (36 bytes, no padding)");
_Static_assert(sizeof(float) == 4, "float must be 4 bytes");

/* ========== IMU 数据上报 ==========
 * 上报频率：IMU_AUTO_REPORT_PERIOD_MS (默认 33ms ≈ 30Hz)
 * 上报格式：FUNC_SENSOR / CMD_SENSOR_GET_IMU，payload = 36B (9 个 float)
 * SRC = 0x10 (下位机)，DST = 0x00 (上位机/PC)
 * seq 自增，主动上报与请求响应共用同一发送函数。
 */
#ifndef IMU_AUTO_REPORT_PERIOD_MS
#define IMU_AUTO_REPORT_PERIOD_MS  (33U)   /**< ≈30Hz 自动上报 */
#endif

#define IMU_DST_PC_ADDR           (0x00U)
#define IMU_SRC_CHASSIS_ADDR      (0x10U)

/*
 * IMU 帧 payload 布局 (共 40B)：
 *   [0..35]   9 个 float (ax,ay,az, wx,wy,wz, roll,pitch,yaw)   —— 与 JY901B_Data 对齐
 *   [36..39]  uint32 采样时刻 tick_ms (HAL_GetTick()，单位 ms，LE)
 *             —— 表示该姿态数据 I2C 读取完成时的下位机时间戳，便于上位机做时间对齐/插值
 */
#define IMU_PAYLOAD_LEN  (36U + 4U)

static void IMU_BuildAndSend(uint8_t seq, uint8_t dst_addr)
{
    JY901B_Data imu;
    uint32_t sample_tick = 0;
    if (!JY901B_GetSnapshotEx(&imu, &sample_tick))
    {
        return;  /* 尚无有效数据 */
    }

    uint8_t frame[10 + IMU_PAYLOAD_LEN + 2];
    const uint16_t payload_len = IMU_PAYLOAD_LEN;

    frame[0] = FRAME_SOF1;
    frame[1] = FRAME_SOF2;
    frame[2] = 0x10;  /* CFG: 版本1, 非ACK */
    frame[3] = seq;
    frame[4] = IMU_SRC_CHASSIS_ADDR;
    frame[5] = dst_addr;
    frame[6] = FUNC_SENSOR;
    frame[7] = CMD_SENSOR_GET_IMU;
    write_le16(&frame[8], payload_len);

    /* 静态断言已保证 JY901B_Data 无 padding，memcpy 安全 */
    memcpy(&frame[10], &imu, 36);
    /* 时间戳（小端）：标记 IMU 采样时刻，而非帧发送时刻 */
    frame[10 + 36 + 0] = (uint8_t)(sample_tick & 0xFFU);
    frame[10 + 36 + 1] = (uint8_t)((sample_tick >> 8)  & 0xFFU);
    frame[10 + 36 + 2] = (uint8_t)((sample_tick >> 16) & 0xFFU);
    frame[10 + 36 + 3] = (uint8_t)((sample_tick >> 24) & 0xFFU);

    uint16_t crc = crc16(frame, 10 + payload_len);
    write_le16(&frame[10 + payload_len], crc);

    DMA_UART1_TxEnqueue(frame, sizeof(frame));
}

/* 兼容：响应上位机 GET_IMU 请求（仍保留，便于即时查询/调试） */
static inline void IMU_SendReport(uint8_t seq, uint8_t src_addr)
{
    IMU_BuildAndSend(seq, src_addr);
}

/* 主动周期性上报 */
static void IMU_AutoReport(void)
{
    static uint8_t auto_seq = 0;
    IMU_BuildAndSend(auto_seq++, IMU_DST_PC_ADDR);
}

/* ========== 启动版本信息打印 ========== */
static void PrintBootBanner(void)
{
    char buf[96];
    int n = snprintf(buf, sizeof(buf),
                     "\r\n[BOOT] %s  build: %s %s\r\n",
                     FIRMWARE_VERSION, __DATE__, __TIME__);
    if (n > 0)
    {
        DMA_UART1_TxSend((const uint8_t *)buf, (uint16_t)n, 20);
    }
}

/* ========== ERR Report (FUNC=0x01, CMD=0x01) ========== */
#define FUNC_SYSTEM       (0x01U)
#define CMD_SYS_ERROR     (0x01U)
#define ERR_MAX_MSG_LEN   (60U)

void ERR_Report(const char *msg)
{
    static uint8_t err_seq = 0;
    uint16_t msg_len = 0;
    while (msg[msg_len] && msg_len < ERR_MAX_MSG_LEN) msg_len++;
    uint16_t payload_len = 4U + msg_len;
    uint8_t frame[10 + 4 + ERR_MAX_MSG_LEN + 2];
    uint32_t tick = HAL_GetTick();
    frame[0] = FRAME_SOF1; frame[1] = FRAME_SOF2;
    frame[2] = 0x10; frame[3] = err_seq++;
    frame[4] = 0x10; frame[5] = 0x00;
    frame[6] = FUNC_SYSTEM; frame[7] = CMD_SYS_ERROR;
    write_le16(&frame[8], payload_len);
    frame[10] = (uint8_t)(tick & 0xFF);
    frame[11] = (uint8_t)((tick >> 8) & 0xFF);
    frame[12] = (uint8_t)((tick >> 16) & 0xFF);
    frame[13] = (uint8_t)((tick >> 24) & 0xFF);
    memcpy(&frame[14], msg, msg_len);
    uint16_t crc = crc16(frame, 10 + payload_len);
    write_le16(&frame[10 + payload_len], crc);
    DMA_UART1_TxEnqueue(frame, 10 + payload_len + 2);
}

/* ========== 协议帧分发 (函数指针回调) ========== */

static void on_host_frame(const uint8_t *frame_buf, uint16_t frame_len, void *user)
{
    (void)user;
    if (frame_len < FRAME_HEADER_SIZE + FRAME_CRC_SIZE) return;

    const FrameHeader *header = (const FrameHeader *)frame_buf;
    const uint8_t *payload = frame_buf + FRAME_HEADER_SIZE;
    uint16_t payload_len = header->len;

    switch (header->func)
    {
        case FUNC_CHASSIS:
            switch (header->cmd)
            {
                case CMD_CHASSIS_SET_VEL:
                    if (payload_len >= 8)
                    {
                        MComm_VelCmd vel_cmd;
                        vel_cmd.v1 = read_le16s(&payload[0]);
                        vel_cmd.v2 = read_le16s(&payload[2]);
                        vel_cmd.v3 = read_le16s(&payload[4]);
                        vel_cmd.v4 = read_le16s(&payload[6]);
                        vel_cmd.accel = (payload_len >= 9) ? payload[8] : 0;
                        MComm_SendVelocity(&vel_cmd);
                    }
                    break;

                case CMD_CHASSIS_SET_POS:
                    if (payload_len >= 20)
                    {
                        MComm_PosCmd pos_cmd;
                        pos_cmd.p1 = read_le32s(&payload[0]);
                        pos_cmd.p2 = read_le32s(&payload[4]);
                        pos_cmd.p3 = read_le32s(&payload[8]);
                        pos_cmd.p4 = read_le32s(&payload[12]);
                        pos_cmd.rpm = read_le16(&payload[16]);
                        pos_cmd.accel = payload[18];
                        pos_cmd.absolute = (payload[19] != 0);
                        MComm_SendPosition(&pos_cmd);
                    }
                    break;

                case CMD_CHASSIS_STOP:
                    MComm_SendStop();
                    break;

                case CMD_CHASSIS_ENABLE:
                    if (payload_len >= 1) MComm_SendEnable(payload[0] != 0);
                    break;

                case CMD_CHASSIS_SYNC:
                    MComm_SendSync();
                    break;

                default: break;
            }
            break;

        case FUNC_SENSOR:
            if (header->cmd == CMD_SENSOR_GET_IMU)
            {
                IMU_SendReport(header->seq, header->src);
            }
            break;

        case FUNC_SERVO:
            /* 0x10~0x1F 段保留给 M5 接口慢速高精度电机 */
            if (header->cmd >= 0x10 && header->cmd <= 0x1F)
            {
                M5_HandleFrame(header->cmd, payload, payload_len);
            }
            else
            {
                TowerCrane_HandleFrame(header->cmd, payload, payload_len);
            }
            break;

        default: break;
    }
}

/* ========== 副机上报帧回调 (CRC 已校验) ========== */

static void on_slave_frame(const uint8_t *frame_buf, uint16_t frame_len, void *user)
{
    (void)user;
    /* 副机→主机的完整帧透传给上位机 */
    DMA_UART1_TxEnqueue(frame_buf, frame_len);
}

/* ========== 主函数 ========== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    MComm_Init();
    TowerCrane_Init();
    /* M5 慢速高精度电机：必须在 TowerCrane_Init() 之后 (USART3 已被 Lift_Init 配置) */
    M5_Init();

    /* 注册协议分发回调（替代弱符号机制） */
    protocol_set_dispatch_handler(on_host_frame, NULL);

    /* 副机透传：独立解析器 + CRC 校验 */
    static ProtocolParser slave_parser;
    protocol_parser_init(&slave_parser, on_slave_frame, NULL);

    /* 启动 USART1/USART2 DMA 接收 */
    DMA_UART1_Start();
    DMA_UART2_Start();

    /* 启动看门狗（放在 DMA 启动之后，避免启动期饿死） */
    MX_IWDG_Init();

    /* 启动指示 */
    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(100);
        IWDG_Feed();
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    /* 启动信息 */
    PrintBootBanner();

    uint32_t last_health_tick = 0;
    uint32_t last_imu_tick = 0;
    uint32_t last_imu_report_tick = 0;

    while (1)
    {
        const uint8_t *buf1, *buf2;
        uint16_t len1, len2;

        /* 上位机数据 → 全局解析器 */
        if (DMA_UART1_GetPendingData(&buf1, &len1, &buf2, &len2))
        {
            if (len1 > 0) protocol_feed_stream(buf1, len1);
            if (len2 > 0) protocol_feed_stream(buf2, len2);
        }

        /* 副机数据 → 独立解析器（CRC 校验后透传） */
        if (DMA_UART2_GetPendingData(&buf1, &len1, &buf2, &len2))
        {
            if (len1 > 0) protocol_parser_feed(&slave_parser, buf1, len1);
            if (len2 > 0) protocol_parser_feed(&slave_parser, buf2, len2);
        }

        uint32_t now = HAL_GetTick();

        /* 200Hz IMU 轮询 */
        if (now - last_imu_tick >= 5)
        {
            last_imu_tick = now;
            JY901B_PollI2C(&hi2c1);
        }

        /* 30Hz IMU 主动上报 */
        if (now - last_imu_report_tick >= IMU_AUTO_REPORT_PERIOD_MS)
        {
            last_imu_report_tick = now;
            IMU_AutoReport();
        }

        /* 1Hz 心跳 LED + alive 打印 */
        if (now - last_health_tick >= 1000)
        {
            last_health_tick = now;
            HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

            char alive_buf[40];
            int alive_len = snprintf(alive_buf, sizeof(alive_buf),
                                     "stm321alive T=%lums\r\n", (unsigned long)now);
            if (alive_len > 0)
                DMA_UART1_TxEnqueue((const uint8_t *)alive_buf, (uint16_t)alive_len);
        }

        /* 喂狗 */
        IWDG_Feed();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        for (volatile uint32_t i = 0; i < 100000; i++);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
