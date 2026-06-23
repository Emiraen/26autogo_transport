/**
 * @file servo.c
 * @brief DS3225 数字舵机 PWM 驱动实现
 *
 *  - TIM3 通用定时器，APB1 *2 = 72MHz；预分频 71 → 1MHz tick
 *    ARR=19999 → 周期 20ms (50Hz)
 *  - TIM1 高级定时器，APB2 = 72MHz；同样 PSC=71，并需 MOE 主输出使能
 */

#include "servo.h"
#include <math.h>

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim3;

/* 当前角度缓存 */
static float s_servo_angle[SERVO_ID_COUNT] = {0};

/* 物料盘工位索引 */
static uint8_t s_plate_slot = 0;

/* 爪子开/合角度 */
static float s_claw_open_deg  = CLAW_OPEN_DEG_DEFAULT;
static float s_claw_close_deg = CLAW_CLOSE_DEG_DEFAULT;
static bool  s_claw_closed    = false;

/* 各路角度上限:
 *   PLATE 物料盘允许超过 270°(放开 SERVO_ANGLE_MAX_DEG 限位)
 *   BASE / CLAW 仍走 270° 安全限位
 */
static inline float servo_max_deg(ServoId id)
{
    return (id == SERVO_ID_PLATE) ? PLATE_ANGLE_MAX_DEG : SERVO_ANGLE_MAX_DEG;
}

static inline uint16_t servo_max_pulse_us(ServoId id)
{
    return (uint16_t)(SERVO_PULSE_MIN_US + servo_max_deg(id) * SERVO_US_PER_DEG + 0.5f);
}

/* 角度→脉宽 (per-servo 钳位) */
static inline uint16_t deg_to_pulse_us(ServoId id, float deg)
{
    float max_d = servo_max_deg(id);
    if (deg < 0.0f) deg = 0.0f;
    if (deg > max_d) deg = max_d;
    return (uint16_t)(SERVO_PULSE_MIN_US + deg * SERVO_US_PER_DEG + 0.5f);
}

/* ----------- 底层 PWM 初始化 ----------- */

static void servo_gpio_init(void)
{
    GPIO_InitTypeDef gi = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA6 = TIM3_CH1, PA7 = TIM3_CH2, PA8 = TIM1_CH1，全部复用推挽 */
    gi.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gi);
}

static void servo_tim3_init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;          /* 1MHz tick */
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000 - 1;          /* 20ms */
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = SERVO_PULSE_MIN_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1);  /* PA6 底座 */
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2);  /* PA7 物料盘 */

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

static void servo_tim1_init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 72 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 20000 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = SERVO_PULSE_MIN_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1);

    /* 高级定时器需主输出使能 */
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/* ----------- API ----------- */

void Servo_Init(void)
{
    servo_gpio_init();
    servo_tim3_init();
    servo_tim1_init();

    /* 默认安全位置 */
    Servo_SetAngle(SERVO_ID_BASE,  BASE_HOME_DEG);
    Servo_SetAngle(SERVO_ID_PLATE, PLATE_HOME_DEG);
    Servo_ClawSet(false);   /* 初始张开 */
    s_plate_slot = 0;
}

bool Servo_SetPulseUs(ServoId id, uint16_t pulse_us)
{
    uint16_t max_pulse = (id < SERVO_ID_COUNT) ? servo_max_pulse_us(id) : SERVO_PULSE_MAX_US;
    if (pulse_us < SERVO_PULSE_MIN_US) pulse_us = SERVO_PULSE_MIN_US;
    if (pulse_us > max_pulse)          pulse_us = max_pulse;

    switch (id)
    {
        case SERVO_ID_BASE:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
            break;
        case SERVO_ID_PLATE:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_us);
            break;
        case SERVO_ID_CLAW:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
            break;
        default: return false;
    }

    /* 反推角度缓存 */
    s_servo_angle[id] = ((float)pulse_us - SERVO_PULSE_MIN_US) / SERVO_US_PER_DEG;
    return true;
}

bool Servo_SetAngle(ServoId id, float deg)
{
    if (id >= SERVO_ID_COUNT) return false;
    return Servo_SetPulseUs(id, deg_to_pulse_us(id, deg));
}

float Servo_GetAngle(ServoId id)
{
    if (id >= SERVO_ID_COUNT) return 0.0f;
    return s_servo_angle[id];
}

/* ====== 物料盘 ====== */

void Servo_PlateGotoSlot(uint8_t slot_index)
{
    slot_index %= PLATE_SLOT_COUNT;
    s_plate_slot = slot_index;
    Servo_SetAngle(SERVO_ID_PLATE, PLATE_HOME_DEG + slot_index * PLATE_STEP_DEG);
}

void Servo_PlateNextSlot(bool forward)
{
    if (forward)
        s_plate_slot = (uint8_t)((s_plate_slot + 1U) % PLATE_SLOT_COUNT);
    else
        s_plate_slot = (uint8_t)((s_plate_slot + PLATE_SLOT_COUNT - 1U) % PLATE_SLOT_COUNT);
    Servo_SetAngle(SERVO_ID_PLATE, PLATE_HOME_DEG + s_plate_slot * PLATE_STEP_DEG);
}

uint8_t Servo_PlateGetSlot(void) { return s_plate_slot; }

/* ====== 底座 ====== */

void Servo_BaseSetWorking(bool working)
{
    Servo_SetAngle(SERVO_ID_BASE, working ? BASE_WORK_DEG : BASE_HOME_DEG);
}

/* ====== 爪子 ====== */

void Servo_SetClawAngles(float open_deg, float close_deg)
{
    s_claw_open_deg  = open_deg;
    s_claw_close_deg = close_deg;
}

void Servo_ClawSet(bool close)
{
    s_claw_closed = close;
    Servo_SetAngle(SERVO_ID_CLAW, close ? s_claw_close_deg : s_claw_open_deg);
}

bool Servo_ClawIsClosed(void) { return s_claw_closed; }
