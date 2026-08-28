/**
 * @file    brush_control.c
 * @brief   滚刷控制模块实现
 *
 *   - 纯时序控制, 无外部反馈依赖
 *   - DONE 与 TIMEOUT 区分: motion_strategy 仅在 DONE 时增加收集计数
 *   - 超时判据优先于完成判据:
 *     BRUSH_TIMEOUT_MS (3000) > BRUSH_ROTATE_DURATION_MS (1200)
 *     先判 timeout 防止卡住时误判 DONE
 *
 */
#include "brush_control.h"

static BrushState_t brush_state     = BRUSH_IDLE;  /* 当前滚刷状态 */
static uint32_t     brush_timer_ms  = 0u;          /* 累计经过的毫秒数 */

void BrushControl_Start(void)
{
    /* 启动旋转: 进入 RUNNING 态, 计时清零 */
    brush_state    = BRUSH_RUNNING;
    brush_timer_ms = 0u;
}

void BrushControl_Update(uint32_t elapsed_ms)
{
    /* 累加时间 */
    brush_timer_ms += elapsed_ms;

    /* 仅在 RUNNING 态判断是否结束 */
    if (brush_state == BRUSH_RUNNING) {
        /* 超时优先判 (BRUSH_TIMEOUT_MS=3000 > BRUSH_ROTATE_DURATION_MS=1200):
         * 先判超时, 防止卡住时误判为正常完成 */
        if (brush_timer_ms >= BRUSH_TIMEOUT_MS) {
            brush_state = BRUSH_TIMEOUT;
        } else if (brush_timer_ms >= BRUSH_ROTATE_DURATION_MS) {
            brush_state = BRUSH_DONE;
        }
    }
}

BrushState_t BrushControl_GetState(void)
{
    return brush_state;
}

uint8_t BrushControl_GetSignal(void)
{
    /* RUNNING 态输出 1 (旋转中), 其余态输出 0 (停止)
     * 这是给外部电机驱动的"开/关"信号 */
    return (brush_state == BRUSH_RUNNING) ? 1u : 0u;
}

void BrushControl_Reset(void)
{
    /* 复位: 回到 IDLE, 计时清零 */
    brush_state    = BRUSH_IDLE;
    brush_timer_ms = 0u;
}
