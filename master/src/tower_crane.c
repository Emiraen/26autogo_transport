/**
 * @file tower_crane.c
 * @brief 塔吊高层控制 - 整合舵机 + 升降电机 (主机侧实现)
 *
 * 重要：所有阻塞等待统一走 TC_DelayMs()，期间会持续喂狗（IWDG 2.4s 超时）。
 *      否则 PickAndPlace 全流程 ≈5.6s 的累计 HAL_Delay 必然触发看门狗复位。
 */

#include "tower_crane.h"
#include "servo.h"
#include "lift_motor.h"
#include "byte_utils.h"

/**
 * @brief 阻塞等待 ms 毫秒，期间每 50ms 喂一次狗。
 *
 * 直接替代 HAL_Delay()。粒度 50ms 远小于 IWDG 超时(2.4s)，
 * 能可靠避免长流程被看门狗复位，同时保持与 HAL_Delay 一致的调用语义。
 */
static void TC_DelayMs(uint32_t ms)
{
    const uint32_t step = 50U;
    uint32_t start = HAL_GetTick();
    /* 先喂一次狗，避免进入函数前已经接近 timeout */
    IWDG_Feed();
    while ((HAL_GetTick() - start) < ms)
    {
        uint32_t elapsed = HAL_GetTick() - start;
        uint32_t remain  = (elapsed >= ms) ? 0U : (ms - elapsed);
        uint32_t slice   = (remain < step) ? remain : step;
        if (slice > 0U) HAL_Delay(slice);
        IWDG_Feed();
    }
}

void TowerCrane_Init(void)
{
    Servo_Init();
    Lift_Init();

    /* 上电默认/待机状态:
     *   - 底座 HOME (钩爪不在料盘上方，位于送料位)
     *   - 爪子张开
     *   - 物料盘归 0 号工位
     *   - 钩爪保持在塔吊最上方 (Lift_Init 中已记账为 TOP)
     */
    Servo_BaseSetWorking(false);
    Servo_PlateGotoSlot(0);
    Servo_ClawSet(false);
    /* 注意：此函数在 IWDG_Init 之前调用，理论上不会被狗咬。
     * 但为了维持调用一致性，后续若调整 init 顺序也安全。 */
    HAL_Delay(500);
}

/* ===== 原子动作 ===== */

void TowerCrane_RotatePlate(bool forward)
{
    Servo_PlateNextSlot(forward);
    TC_DelayMs(TC_DELAY_PLATE_MS);
}

void TowerCrane_PlateGoto(uint8_t slot)
{
    Servo_PlateGotoSlot(slot);
    TC_DelayMs(TC_DELAY_PLATE_MS);
}

void TowerCrane_BaseToWork(void)
{
    Servo_BaseSetWorking(true);
    TC_DelayMs(TC_DELAY_BASE_MS);
}

void TowerCrane_BaseToHome(void)
{
    Servo_BaseSetWorking(false);
    TC_DelayMs(TC_DELAY_BASE_MS);
}

void TowerCrane_ClawOpen(void)
{
    Servo_ClawSet(false);
    TC_DelayMs(TC_DELAY_CLAW_MS);
}

void TowerCrane_ClawClose(void)
{
    Servo_ClawSet(true);
    TC_DelayMs(TC_DELAY_CLAW_MS);
}

void TowerCrane_LiftTo(float mm)
{
    Lift_MoveToHeight(mm);
    TC_DelayMs(TC_DELAY_LIFT_MS);
}

void TowerCrane_LiftStop(void)
{
    Lift_StopNow();
}

/* ===== 完整流程 ===== */

void TowerCrane_PickAndPlace(void)
{
    /* 默认/待机: 底座 HOME, 爪子张开, 钩爪在 TOP */

    /* 1. 爪子张开 (确保安全) */
    TowerCrane_ClawOpen();

    /* 2. 底座转到工作位 —— 钩爪对准料盘正上方 */
    TowerCrane_BaseToWork();

    /* 3. 从 TOP 下降到料盘高度 (BOTTOM) */
    TowerCrane_LiftTo(LIFT_HEIGHT_BOTTOM_MM);

    /* 4. 闭合爪子夹取物料 */
    TowerCrane_ClawClose();

    /* 5. 上升到塔吊顶部 */
    TowerCrane_LiftTo(LIFT_HEIGHT_TOP_MM);

    /* 6. 底座回 HOME (送料位) */
    TowerCrane_BaseToHome();

    /* 7. 张开爪子放下物料，保持在 TOP 作为待机位 */
    TowerCrane_ClawOpen();
}

void TowerCrane_NextSlotAndPick(void)
{
    TowerCrane_RotatePlate(true);
    TowerCrane_PickAndPlace();
}

/* ===== 串口分发 ===== */

void TowerCrane_HandleFrame(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    switch (cmd)
    {
        case TC_CMD_SERVO_SET_ANGLE:
            if (len >= 3)
            {
                uint8_t id = payload[0];
                int16_t angle_x10 = read_le16s(&payload[1]);
                if (id < SERVO_ID_COUNT)
                {
                    Servo_SetAngle((ServoId)id, angle_x10 / 10.0f);
                }
            }
            break;

        case TC_CMD_PLATE_NEXT:
            if (len >= 1) Servo_PlateNextSlot(payload[0] != 0);
            else          Servo_PlateNextSlot(true);
            break;

        case TC_CMD_PLATE_GOTO:
            if (len >= 1) Servo_PlateGotoSlot(payload[0]);
            break;

        case TC_CMD_BASE_SET:
            if (len >= 1) Servo_BaseSetWorking(payload[0] != 0);
            break;

        case TC_CMD_CLAW_SET:
            if (len >= 1) Servo_ClawSet(payload[0] != 0);
            break;

        case TC_CMD_LIFT_MOVE:
            if (len >= 2)
            {
                int16_t mm_x10 = read_le16s(&payload[0]);
                Lift_MoveToHeight(mm_x10 / 10.0f);
            }
            break;

        case TC_CMD_LIFT_STOP:
            Lift_StopNow();
            break;

        case TC_CMD_PICK_PLACE:
            TowerCrane_PickAndPlace();
            break;

        case TC_CMD_NEXT_AND_PICK:
            TowerCrane_NextSlotAndPick();
            break;

        default:
            break;
    }
}
