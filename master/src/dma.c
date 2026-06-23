/**
 * @file dma.c
 * @brief 主机 DMA 配置
 *
 * USART1 DMA: 接收上位机数据 (DMA1_Ch5 RX, DMA1_Ch4 TX)
 * USART2 DMA: 接收副机里程计 (DMA1_Ch6 RX) + 发送指令 (DMA1_Ch7 TX)
 *
 * 关键设计：
 *   - RX：环形 DMA + IDLE 中断 + read/write 指针，临界区保护避免读写竞争
 *   - TX：FIFO 队列 + DMA TC 中断驱动，非阻塞入队，避免主循环忙等
 */

#include "dma.h"
#include "usart.h"
#include <string.h>
#include <stdbool.h>

#define RX1_DMA_BUF_SIZE 1024
#define RX2_DMA_BUF_SIZE 512

/* TX FIFO：单一环形字节缓冲，记录每帧长度 */
#define TX1_BUF_SIZE     2048
#define TX1_QUEUE_DEPTH  16

#define TX2_BUF_SIZE     1024
#define TX2_QUEUE_DEPTH  16

/* ========== DMA handles ========== */

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* ========== USART1 RX 环形 DMA 缓冲区 ========== */

static uint8_t rx1_dma_buf[RX1_DMA_BUF_SIZE];
static volatile uint16_t rx1_write_pos = 0;
static volatile uint16_t rx1_read_pos = 0;
static volatile uint32_t rx1_overrun = 0;

/* ========== USART2 RX 环形 DMA 缓冲区 ========== */

static uint8_t rx2_dma_buf[RX2_DMA_BUF_SIZE];
static volatile uint16_t rx2_write_pos = 0;
static volatile uint16_t rx2_read_pos = 0;
static volatile uint32_t rx2_overrun = 0;

/* ========== TX FIFO 结构 ========== */

typedef struct {
    uint16_t offset;
    uint16_t length;
} TxFrameDesc;

typedef struct {
    uint8_t      *buf;
    uint16_t      buf_size;
    TxFrameDesc  *queue;
    uint16_t      queue_depth;
    volatile uint16_t q_head;     /* 入队位置 */
    volatile uint16_t q_tail;     /* 出队位置 */
    volatile uint16_t buf_head;   /* 缓冲写入位置 */
    volatile uint16_t buf_tail;   /* 缓冲已发送位置 */
    volatile bool     busy;
    UART_HandleTypeDef *huart;
} TxFifo;

static uint8_t      tx1_buf[TX1_BUF_SIZE];
static TxFrameDesc  tx1_queue[TX1_QUEUE_DEPTH];
static TxFifo tx1_fifo;

static uint8_t      tx2_buf[TX2_BUF_SIZE];
static TxFrameDesc  tx2_queue[TX2_QUEUE_DEPTH];
static TxFifo tx2_fifo;

static void TxFifo_Init(TxFifo *f, UART_HandleTypeDef *huart,
                        uint8_t *buf, uint16_t buf_size,
                        TxFrameDesc *queue, uint16_t queue_depth)
{
    f->buf = buf;
    f->buf_size = buf_size;
    f->queue = queue;
    f->queue_depth = queue_depth;
    f->q_head = f->q_tail = 0;
    f->buf_head = f->buf_tail = 0;
    f->busy = false;
    f->huart = huart;
}

/* 启动队首帧的 DMA 发送（必须在临界区内或 busy=false 时调用） */
static void TxFifo_TryStart(TxFifo *f)
{
    if (f->busy) return;
    if (f->q_head == f->q_tail) return;  /* 队列空 */

    TxFrameDesc *d = &f->queue[f->q_tail];
    f->busy = true;
    if (HAL_UART_Transmit_DMA(f->huart, &f->buf[d->offset], d->length) != HAL_OK)
    {
        /* 启动失败：丢弃此帧，尝试下一帧 */
        f->busy = false;
        f->buf_tail = (uint16_t)((d->offset + d->length) % f->buf_size);
        f->q_tail = (uint16_t)((f->q_tail + 1) % f->queue_depth);
    }
}

/* TC 中断回调：当前帧已发完，推进队列 */
static void TxFifo_OnComplete(TxFifo *f)
{
    if (f->q_head == f->q_tail) { f->busy = false; return; }
    TxFrameDesc *d = &f->queue[f->q_tail];
    f->buf_tail = (uint16_t)((d->offset + d->length) % f->buf_size);
    f->q_tail = (uint16_t)((f->q_tail + 1) % f->queue_depth);
    f->busy = false;
    TxFifo_TryStart(f);
}

/* 计算 FIFO 可用字节数 */
static uint16_t TxFifo_FreeBytes(const TxFifo *f)
{
    uint16_t head = f->buf_head;
    uint16_t tail = f->buf_tail;
    if (head >= tail)
        return (uint16_t)(f->buf_size - (head - tail) - 1);
    else
        return (uint16_t)(tail - head - 1);
}

static uint16_t TxFifo_QueueCount(const TxFifo *f)
{
    return (uint16_t)((f->q_head + f->queue_depth - f->q_tail) % f->queue_depth);
}

/* 非阻塞入队：帧必须在缓冲区中物理连续（DMA 要求） */
static bool TxFifo_Enqueue(TxFifo *f, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len >= f->buf_size) return false;

    bool ok = false;
    __disable_irq();
    {
        if (TxFifo_QueueCount(f) < (uint16_t)(f->queue_depth - 1))
        {
            uint16_t head = f->buf_head;
            uint16_t tail = f->buf_tail;
            uint16_t write_at = 0xFFFFu;

            if (head >= tail)
            {
                /* 可用：[head, buf_size) 和 [0, tail) */
                if ((uint16_t)(f->buf_size - head) >= len + (uint16_t)(tail == 0 ? 1 : 0))
                {
                    /* 末尾连续放得下；注意当 tail==0 时不能让 head 追到 tail */
                    write_at = head;
                }
                else if (tail > len)  /* 必须严格大于，保证 head 不会等于 tail */
                {
                    write_at = 0;
                }
            }
            else
            {
                /* head < tail，可用：[head, tail) */
                if ((uint16_t)(tail - head) > len)
                {
                    write_at = head;
                }
            }

            if (write_at != 0xFFFFu)
            {
                TxFrameDesc *d = &f->queue[f->q_head];
                d->offset = write_at;
                d->length = len;
                memcpy(&f->buf[write_at], data, len);
                f->buf_head = (uint16_t)((write_at + len) % f->buf_size);
                f->q_head = (uint16_t)((f->q_head + 1) % f->queue_depth);
                ok = true;
                TxFifo_TryStart(f);
            }
        }
    }
    __enable_irq();
    return ok;
}

/* ========== DMA 初始化 ========== */

void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* --- USART1 RX DMA (DMA1_Channel5) --- */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart1_rx);
    __HAL_LINKDMA(&huart1, hdmarx, hdma_usart1_rx);

    /* --- USART1 TX DMA (DMA1_Channel4) --- */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    /* --- USART2 RX DMA (DMA1_Channel6) --- */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    /* --- USART2 TX DMA (DMA1_Channel7) --- */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&hdma_usart2_tx);
    __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

    /* NVIC */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

    /* TX FIFO 初始化 */
    TxFifo_Init(&tx1_fifo, &huart1, tx1_buf, TX1_BUF_SIZE, tx1_queue, TX1_QUEUE_DEPTH);
    TxFifo_Init(&tx2_fifo, &huart2, tx2_buf, TX2_BUF_SIZE, tx2_queue, TX2_QUEUE_DEPTH);
}

/* ========== USART1 (上位机) ========== */

void DMA_UART1_Start(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, rx1_dma_buf, RX1_DMA_BUF_SIZE);
}

bool DMA_UART1_TxEnqueue(const uint8_t *data, uint16_t len)
{
    return TxFifo_Enqueue(&tx1_fifo, data, len);
}

bool DMA_UART1_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!DMA_UART1_TxEnqueue(data, len))
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return false;
    }
    return true;
}

void DMA_UART1_RxIdleHandler(void)
{
    uint16_t wp = (uint16_t)(RX1_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    /* 检测覆盖：若距离 read_pos 不到一整圈则正常 */
    rx1_write_pos = wp;
}

void DMA_UART1_TxCpltHandler(void)
{
    TxFifo_OnComplete(&tx1_fifo);
}

bool DMA_UART1_GetPendingData(const uint8_t **buf1, uint16_t *len1,
                               const uint8_t **buf2, uint16_t *len2)
{
    *buf1 = NULL; *len1 = 0;
    *buf2 = NULL; *len2 = 0;

    uint16_t wp, rp, pending;
    __disable_irq();
    /* 直接采样 DMA 当前写位置：CNDTR 比 rx1_write_pos(仅 IDLE 更新)更准 */
    wp = (uint16_t)(RX1_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    rx1_write_pos = wp;
    rp = rx1_read_pos;
    __enable_irq();

    if (wp == rp) return false;

    pending = (wp > rp) ? (uint16_t)(wp - rp)
                        : (uint16_t)(RX1_DMA_BUF_SIZE - rp + wp);

    /* 容量逼近判定：留 32B 余量，否则视为已发生覆盖 */
    if (pending >= (uint16_t)(RX1_DMA_BUF_SIZE - 32))
    {
        rx1_overrun++;
    }

    if (wp > rp)
    {
        *buf1 = &rx1_dma_buf[rp];
        *len1 = (uint16_t)(wp - rp);
    }
    else
    {
        *buf1 = &rx1_dma_buf[rp];
        *len1 = (uint16_t)(RX1_DMA_BUF_SIZE - rp);
        if (wp > 0)
        {
            *buf2 = &rx1_dma_buf[0];
            *len2 = wp;
        }
    }

    __disable_irq();
    rx1_read_pos = wp;
    /* 后置校验：消费期间 DMA 推进的距离若 >= 上一次读到的剩余空间，
       说明 DMA 已绕回并覆盖即将由消费者解析的数据 */
    uint16_t cur_wp = (uint16_t)(RX1_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx));
    uint16_t advance = (uint16_t)((cur_wp + RX1_DMA_BUF_SIZE - wp) % RX1_DMA_BUF_SIZE);
    if (advance >= (uint16_t)(RX1_DMA_BUF_SIZE - pending - 1))
    {
        rx1_overrun++;
    }
    __enable_irq();
    return true;
}

uint32_t DMA_UART1_GetRxOverrunCount(void)
{
    return rx1_overrun;
}

/* ========== USART2 (副机) ========== */

void DMA_UART2_Start(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, rx2_dma_buf, RX2_DMA_BUF_SIZE);
}

bool DMA_UART2_TxEnqueue(const uint8_t *data, uint16_t len)
{
    return TxFifo_Enqueue(&tx2_fifo, data, len);
}

bool DMA_UART2_TxSend(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (!DMA_UART2_TxEnqueue(data, len))
    {
        if ((HAL_GetTick() - start) >= timeout_ms)
            return false;
    }
    return true;
}

void DMA_UART2_RxIdleHandler(void)
{
    rx2_write_pos = (uint16_t)(RX2_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
}

void DMA_UART2_TxCpltHandler(void)
{
    TxFifo_OnComplete(&tx2_fifo);
}

bool DMA_UART2_GetPendingData(const uint8_t **buf1, uint16_t *len1,
                               const uint8_t **buf2, uint16_t *len2)
{
    *buf1 = NULL; *len1 = 0;
    *buf2 = NULL; *len2 = 0;

    uint16_t wp, rp, pending;
    __disable_irq();
    /* 直接采样 CNDTR，避免依赖仅 IDLE 才更新的缓存值 */
    wp = (uint16_t)(RX2_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
    rx2_write_pos = wp;
    rp = rx2_read_pos;
    __enable_irq();

    if (wp == rp) return false;

    pending = (wp > rp) ? (uint16_t)(wp - rp)
                        : (uint16_t)(RX2_DMA_BUF_SIZE - rp + wp);

    /* 容量逼近判定：留 32B 余量 */
    if (pending >= (uint16_t)(RX2_DMA_BUF_SIZE - 32))
    {
        rx2_overrun++;
    }

    if (wp > rp)
    {
        *buf1 = &rx2_dma_buf[rp];
        *len1 = (uint16_t)(wp - rp);
    }
    else
    {
        *buf1 = &rx2_dma_buf[rp];
        *len1 = (uint16_t)(RX2_DMA_BUF_SIZE - rp);
        if (wp > 0)
        {
            *buf2 = &rx2_dma_buf[0];
            *len2 = wp;
        }
    }

    __disable_irq();
    rx2_read_pos = wp;
    /* 后置校验：消费期间 DMA 是否绕回覆盖 */
    uint16_t cur_wp = (uint16_t)(RX2_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx));
    uint16_t advance = (uint16_t)((cur_wp + RX2_DMA_BUF_SIZE - wp) % RX2_DMA_BUF_SIZE);
    if (advance >= (uint16_t)(RX2_DMA_BUF_SIZE - pending - 1))
    {
        rx2_overrun++;
    }
    __enable_irq();
    return true;
}

uint32_t DMA_UART2_GetRxOverrunCount(void)
{
    return rx2_overrun;
}
