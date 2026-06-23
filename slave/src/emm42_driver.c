/**
 * @file emm42_driver.c
 * @brief EMM42 闭环步进电机 RS485 驱动实现 (副机专用)
 * 
 * 通过 USART3 + RS485 (PB1 方向控制) 与 4 个 EMM42 电机通信
 */

#include "emm42_driver.h"
#include "gpio.h"
#include "usart.h"

/* 内部 UART 句柄指针 */
static UART_HandleTypeDef *s_huart = NULL;

/* 发送超时 (ms) */
#define EMM42_TX_TIMEOUT  10U
/* 接收超时 (ms) — 115200 波特率下 8 字节 < 1ms，2ms 足够判超时 */
#define EMM42_RX_TIMEOUT  2U
/* TC (Transmission Complete) 等待超时 (ms)
 * 115200 下单字节 ≈ 87µs，最大帧 13 字节 ≈ 1.2ms，5ms 余量足够。
 * 之前用 while(){} 死等 TC，遇到 USART3 异常会永久卡死主循环
 * 触发 IWDG 反复复位，必须改为有超时返回。 */
#define EMM42_TC_TIMEOUT  5U

/* 帧间死区延时 (防止高波特率粘包) */
static inline void delay_us(uint32_t us)
{
    /* 72MHz 主频，每次循环约 4 个时钟周期 ≈ 55.6ns，18 次 ≈ 1µs */
    volatile uint32_t count = us * 18U;
    while (count--) {}
}

/* 带超时的 TC 等待，避免 USART 异常时永久阻塞 */
static HAL_StatusTypeDef EMM42_WaitTxComplete(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while (__HAL_UART_GET_FLAG(s_huart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - t0) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/**
 * @brief 发送 RS485 数据帧 (仅发送，发完切回 RX，附带帧间死区)
 */
static HAL_StatusTypeDef EMM42_SendFrame(const uint8_t *data, uint16_t len)
{
    if (s_huart == NULL || data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    RS485_SetTxMode();
    HAL_StatusTypeDef status = HAL_UART_Transmit(s_huart, (uint8_t *)data, len, EMM42_TX_TIMEOUT);
    /* 即使 Transmit 报错也务必切回 RX，避免 RS485 长期占用总线 */
    if (EMM42_WaitTxComplete(EMM42_TC_TIMEOUT) != HAL_OK && status == HAL_OK)
    {
        status = HAL_TIMEOUT;
    }
    RS485_SetRxMode();

    /* 帧间死区 150µs，防止电机端粘包 */
    delay_us(150);

    return status;
}

/**
 * @brief RS485 原子查询: 发送命令 + 阻塞接收响应
 *
 * 整个 TX→等TC→切RX→Receive 在主循环中顺序执行，
 * 不会被其他 RS485 操作打断，天然防踩踏。
 */
static HAL_StatusTypeDef EMM42_QueryFrame(const uint8_t *tx_data, uint16_t tx_len,
                                           uint8_t *rx_buf, uint16_t rx_len)
{
    if (s_huart == NULL) return HAL_ERROR;

    /* 清除 UART 接收缓冲区中的残留数据 */
    __HAL_UART_CLEAR_FLAG(s_huart, UART_FLAG_RXNE);
    (void)s_huart->Instance->DR;

    /* 发送查询命令 */
    RS485_SetTxMode();
    HAL_StatusTypeDef status = HAL_UART_Transmit(s_huart, (uint8_t *)tx_data, tx_len, EMM42_TX_TIMEOUT);
    if (status != HAL_OK)
    {
        RS485_SetRxMode();
        return status;
    }
    if (EMM42_WaitTxComplete(EMM42_TC_TIMEOUT) != HAL_OK)
    {
        RS485_SetRxMode();
        return HAL_TIMEOUT;
    }

    /* 切换到接收模式，阻塞等待电机响应 */
    RS485_SetRxMode();
    status = HAL_UART_Receive(s_huart, rx_buf, rx_len, EMM42_RX_TIMEOUT);

    /* 查询完成后也加帧间死区 */
    delay_us(150);

    return status;
}

void EMM42_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    RS485_SetRxMode();
}

EMM42_Status EMM42_Enable(uint8_t addr, bool enable, bool sync)
{
    uint8_t frame[6];
    frame[0] = addr;
    frame[1] = EMM42_CMD_ENABLE;
    frame[2] = 0xABU;
    frame[3] = enable ? 0x01U : 0x00U;
    frame[4] = sync ? 0x01U : 0x00U;
    frame[5] = EMM42_CHECKSUM;

    if (EMM42_SendFrame(frame, sizeof(frame)) != HAL_OK)
        return EMM42_ERR_TIMEOUT;
    return EMM42_OK;
}

EMM42_Status EMM42_SetSpeed(uint8_t addr, bool ccw, uint16_t rpm, uint8_t accel, bool sync)
{
    uint8_t frame[8];
    frame[0] = addr;
    frame[1] = EMM42_CMD_SPEED;
    frame[2] = ccw ? 0x01U : 0x00U;
    frame[3] = (uint8_t)(rpm >> 8);
    frame[4] = (uint8_t)(rpm & 0xFF);
    frame[5] = accel;
    frame[6] = sync ? 0x01U : 0x00U;
    frame[7] = EMM42_CHECKSUM;

    if (EMM42_SendFrame(frame, sizeof(frame)) != HAL_OK)
        return EMM42_ERR_TIMEOUT;
    return EMM42_OK;
}

EMM42_Status EMM42_MovePosition(uint8_t addr, bool ccw, uint16_t rpm, uint8_t accel,
                                 uint32_t pulses, bool absolute, bool sync)
{
    uint8_t frame[13];
    frame[0] = addr;
    frame[1] = EMM42_CMD_POSITION;
    frame[2] = ccw ? 0x01U : 0x00U;
    frame[3] = (uint8_t)(rpm >> 8);
    frame[4] = (uint8_t)(rpm & 0xFF);
    frame[5] = accel;
    frame[6] = (uint8_t)(pulses >> 24);
    frame[7] = (uint8_t)((pulses >> 16) & 0xFF);
    frame[8] = (uint8_t)((pulses >> 8) & 0xFF);
    frame[9] = (uint8_t)(pulses & 0xFF);
    frame[10] = absolute ? 0x01U : 0x00U;
    frame[11] = sync ? 0x01U : 0x00U;
    frame[12] = EMM42_CHECKSUM;

    if (EMM42_SendFrame(frame, sizeof(frame)) != HAL_OK)
        return EMM42_ERR_TIMEOUT;
    return EMM42_OK;
}

EMM42_Status EMM42_StopNow(uint8_t addr, bool sync)
{
    uint8_t frame[5];
    frame[0] = addr;
    frame[1] = EMM42_CMD_STOP;
    frame[2] = 0x98U;
    frame[3] = sync ? 0x01U : 0x00U;
    frame[4] = EMM42_CHECKSUM;

    if (EMM42_SendFrame(frame, sizeof(frame)) != HAL_OK)
        return EMM42_ERR_TIMEOUT;
    return EMM42_OK;
}

EMM42_Status EMM42_SyncStart(void)
{
    uint8_t frame[4] = {EMM42_BROADCAST, EMM42_CMD_SYNC, 0x66U, EMM42_CHECKSUM};

    if (EMM42_SendFrame(frame, sizeof(frame)) != HAL_OK)
        return EMM42_ERR_TIMEOUT;
    return EMM42_OK;
}

/**
 * @brief 设置 4 轮速度 (与 chassis3 一致的稳妥方案)
 *
 * 关键点:
 *   - sync=false: 每个电机收到 SetSpeed 后立即启动，不等广播 SyncStart
 *   - 每帧之间 HAL_Delay(5) 等待电机的 3 字节应答帧发完再发下一帧,
 *     避免我们把下一条命令撞到电机正在回传的响应帧上 (115200 波特率
 *     下 3 字节 ≈ 0.26ms, 5ms 余量足够)。
 *     未加该延迟时, 因为每个 SetSpeed 的内部死区只有 150µs,
 *     会出现 FR/RL 丢帧不转, 而 FL/RR 正常转动的对角模式。
 *   - 失败继续: 某个电机超时不阻塞其余电机
 *   - 返回值: 返回第一个失败的错误码 (便于上层判断)，全部成功返回 EMM42_OK
 */
#define EMM42_SETWHEEL_INTER_FRAME_MS  5U

EMM42_Status EMM42_SetWheelSpeeds(int16_t rpm_fl, int16_t rpm_fr,
                                   int16_t rpm_rl, int16_t rpm_rr,
                                   uint8_t accel)
{
    EMM42_Status first_err = EMM42_OK;
    EMM42_Status st;

    bool ccw_fl = (rpm_fl >= 0);
    uint16_t abs_fl = (uint16_t)(ccw_fl ? rpm_fl : -rpm_fl);
    st = EMM42_SetSpeed(EMM42_MOTOR_FL, ccw_fl, abs_fl, accel, false);
    if (st != EMM42_OK && first_err == EMM42_OK) first_err = st;
    HAL_Delay(EMM42_SETWHEEL_INTER_FRAME_MS);

    bool ccw_fr = (rpm_fr >= 0);
    uint16_t abs_fr = (uint16_t)(ccw_fr ? rpm_fr : -rpm_fr);
    st = EMM42_SetSpeed(EMM42_MOTOR_FR, ccw_fr, abs_fr, accel, false);
    if (st != EMM42_OK && first_err == EMM42_OK) first_err = st;
    HAL_Delay(EMM42_SETWHEEL_INTER_FRAME_MS);

    bool ccw_rl = (rpm_rl >= 0);
    uint16_t abs_rl = (uint16_t)(ccw_rl ? rpm_rl : -rpm_rl);
    st = EMM42_SetSpeed(EMM42_MOTOR_RL, ccw_rl, abs_rl, accel, false);
    if (st != EMM42_OK && first_err == EMM42_OK) first_err = st;
    HAL_Delay(EMM42_SETWHEEL_INTER_FRAME_MS);

    bool ccw_rr = (rpm_rr >= 0);
    uint16_t abs_rr = (uint16_t)(ccw_rr ? rpm_rr : -rpm_rr);
    st = EMM42_SetSpeed(EMM42_MOTOR_RR, ccw_rr, abs_rr, accel, false);
    if (st != EMM42_OK && first_err == EMM42_OK) first_err = st;

    return first_err;
}

/* ================================================================
 *  里程计读取功能 — RS485 原子操作 (发送查询 + 阻塞接收)
 * ================================================================ */

/**
 * @brief 内部通用读取 (8字节响应): 位置 / 脉冲数
 * 响应格式: addr + cmd + sign(1B) + data(4B, 大端) + 0x6B
 */
static EMM42_Status EMM42_ReadRegister(uint8_t addr, uint8_t cmd,
                                        uint8_t *sign, uint32_t *value)
{
    if (addr == 0 || addr > EMM42_MOTOR_COUNT || sign == NULL || value == NULL)
        return EMM42_ERR_PARAM;

    uint8_t tx[3] = { addr, cmd, EMM42_CHECKSUM };
    uint8_t rx[EMM42_READ_RESP_LEN];

    HAL_StatusTypeDef hal_st = EMM42_QueryFrame(tx, sizeof(tx), rx, EMM42_READ_RESP_LEN);

    if (hal_st != HAL_OK)
        return EMM42_ERR_TIMEOUT;

    if (rx[0] != addr || rx[1] != cmd || rx[7] != EMM42_CHECKSUM)
        return EMM42_ERR_PARAM;

    *sign  = rx[2];
    *value = ((uint32_t)rx[3] << 24) |
             ((uint32_t)rx[4] << 16) |
             ((uint32_t)rx[5] << 8)  |
             ((uint32_t)rx[6]);

    return EMM42_OK;
}

EMM42_Status EMM42_ReadPosition(uint8_t addr, uint8_t *sign, uint32_t *position)
{
    return EMM42_ReadRegister(addr, EMM42_CMD_READ_POS, sign, position);
}

EMM42_Status EMM42_ReadPulses(uint8_t addr, uint8_t *sign, uint32_t *pulses)
{
    return EMM42_ReadRegister(addr, EMM42_CMD_READ_PULSES, sign, pulses);
}

EMM42_Status EMM42_ReadAllPositions(uint8_t signs[EMM42_MOTOR_COUNT],
                                     uint32_t positions[EMM42_MOTOR_COUNT])
{
    static const uint8_t addrs[EMM42_MOTOR_COUNT] = {
        EMM42_MOTOR_FL, EMM42_MOTOR_FR, EMM42_MOTOR_RL, EMM42_MOTOR_RR
    };

    for (uint8_t i = 0; i < EMM42_MOTOR_COUNT; i++)
    {
        EMM42_Status st = EMM42_ReadPosition(addrs[i], &signs[i], &positions[i]);
        if (st != EMM42_OK)
            return st;
    }
    return EMM42_OK;
}

/**
 * @brief 读取单个电机实时转速
 * 发送: addr + 0x35 + 0x6B (3字节)
 * 响应: addr + 0x35 + sign(1B) + rpm_h + rpm_l + 0x6B (6字节)
 */
EMM42_Status EMM42_ReadSpeed(uint8_t addr, uint8_t *sign, uint16_t *rpm)
{
    if (addr == 0 || addr > EMM42_MOTOR_COUNT || sign == NULL || rpm == NULL)
        return EMM42_ERR_PARAM;

    uint8_t tx[3] = { addr, EMM42_CMD_READ_SPEED, EMM42_CHECKSUM };
    uint8_t rx[EMM42_READ_SPEED_LEN];

    HAL_StatusTypeDef hal_st = EMM42_QueryFrame(tx, sizeof(tx), rx, EMM42_READ_SPEED_LEN);

    if (hal_st != HAL_OK)
        return EMM42_ERR_TIMEOUT;

    if (rx[0] != addr || rx[1] != EMM42_CMD_READ_SPEED || rx[5] != EMM42_CHECKSUM)
        return EMM42_ERR_PARAM;

    *sign = rx[2];
    *rpm  = ((uint16_t)rx[3] << 8) | rx[4];

    return EMM42_OK;
}

/**
 * @brief 容错式轮询全部4个电机转速
 * 
 * 即使某个电机超时也继续读取剩余电机，返回失败掩码。
 * 失败的电机对应的 signs[i] / rpms[i] 不会被修改，
 * 调用者可预填缓存值实现容错。
 * 
 * @return 失败掩码: 0=全部成功, bit0=FL, bit1=FR, bit2=RL, bit3=RR
 */
uint8_t EMM42_ReadAllSpeeds(uint8_t signs[EMM42_MOTOR_COUNT],
                             uint16_t rpms[EMM42_MOTOR_COUNT])
{
    static const uint8_t addrs[EMM42_MOTOR_COUNT] = {
        EMM42_MOTOR_FL, EMM42_MOTOR_FR, EMM42_MOTOR_RL, EMM42_MOTOR_RR
    };

    uint8_t fail_mask = 0;

    for (uint8_t i = 0; i < EMM42_MOTOR_COUNT; i++)
    {
        EMM42_Status st = EMM42_ReadSpeed(addrs[i], &signs[i], &rpms[i]);
        if (st != EMM42_OK)
        {
            fail_mask |= (1U << i);
        }
    }
    return fail_mask;
}
