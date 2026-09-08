/**
 * @file    bucket_unload.c
 * @brief   后斗卸载控制模块实现
 *
 *   - 纯时序控制, 无外部反馈依赖 (不需查询后斗/后门完成状态)
 *   - 每阶段计时到达后自动切换到下一阶段
 *   - DONE 态保持 bucket_cycle=0 / door=0 (安全态), 等待 motion_strategy 复位
 *
 */
#include "bucket_unload.h"

static BucketState_t bucket_state    = BUCKET_IDLE;  /* 当前后斗状态 */
static uint32_t      bucket_timer_ms = 0u;            /* 当前阶段累计的毫秒数 */

void BucketUnload_Start(void)
{
    /* 启动卸载: 进入 LIFTING 态 (升后斗), 阶段计时清零 */
    bucket_state    = BUCKET_LIFTING;
    bucket_timer_ms = 0u;
}

void BucketUnload_Update(uint32_t elapsed_ms)
{
    /* 累加阶段时间 */
    bucket_timer_ms += elapsed_ms;

    /* 根据当前状态判断是否切换到下一阶段 */
    switch (bucket_state) {
        case BUCKET_LIFTING:
            /* 升后斗阶段: 到 2.0s 切换到开门卸货 */
            if (bucket_timer_ms >= BUCKET_LIFT_DURATION_MS) {
                bucket_state    = BUCKET_UNLOADING;
                bucket_timer_ms = 0u;     /* 新阶段计时清零 */
            }
            break;

        case BUCKET_UNLOADING:
            /* 开门卸货阶段: 到 4.0s 切换到关门 */
            if (bucket_timer_ms >= BUCKET_UNLOAD_DURATION_MS) {
                bucket_state    = BUCKET_CLOSING;
                bucket_timer_ms = 0u;
            }
            break;

        case BUCKET_CLOSING:
            /* 关门阶段: 到 0.5s 切换到完成 */
            if (bucket_timer_ms >= BUCKET_DOOR_CLOSE_MS) {
                bucket_state    = BUCKET_DONE;
                bucket_timer_ms = 0u;
            }
            break;

        default:
            /* IDLE / DONE: 无动作, 等待外部 Start 或 Reset */
            break;
    }
}

BucketState_t BucketUnload_GetState(void)
{
    return bucket_state;
}

uint8_t BucketUnload_GetLiftSignal(void)
{
    /* 仅 LIFTING 态输出升起信号, 其他态 (含 UNLOADING) 保持下降/原位
     * 这样后斗升到位后就放下, 不一直举着, 节能且安全 */
    return (bucket_state == BUCKET_LIFTING) ? 1u : 0u;
}

uint8_t BucketUnload_GetDoorOpenSignal(void)
{
    /* 仅 UNLOADING 态输出开门信号, 让物块有时间滑出再关门 */
    return (bucket_state == BUCKET_UNLOADING) ? 1u : 0u;
}

void BucketUnload_Reset(void)
{
    /* 复位: 回到 IDLE, 计时清零, 所有信号归零 */
    bucket_state    = BUCKET_IDLE;
    bucket_timer_ms = 0u;
}
