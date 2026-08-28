/**
 * @file    brush_control.h
 * @brief   滚刷控制模块: 非阻塞定时旋转控制
 *
 *   IDLE (空闲)
 *     ↓ Start()
 *   RUNNING (旋转中, brush_cycle=1)
 *     ↓ 1.2s 到 → DONE (正常完成)
 *     ↓ 3.0s 到 → TIMEOUT (超时未完成, 可能卡住)
 *   DONE/TIMEOUT 态 brush_cycle=0 (停止)
 *
 */
#ifndef BRUSH_CONTROL_H
#define BRUSH_CONTROL_H

#include "motion_config.h"

/**
 * @brief  滚刷状态枚举
 */
typedef enum {
    BRUSH_IDLE = 0,     /* 空闲, 未启动 */
    BRUSH_RUNNING,       /* 正在旋转, brush_cycle=1 */
    BRUSH_DONE,         /* 正常完成 (达到 BRUSH_ROTATE_DURATION_MS) */
    BRUSH_TIMEOUT       /* 超时未完成 (达到 BRUSH_TIMEOUT_MS) */
} BrushState_t;

/**
 * @brief  启动滚刷旋转 (非阻塞)
 *
 * 【作用】
 *   进入 RUNNING 态, brush_cycle 信号置 1, 内部计时器清零。
 *   之后外部需周期调用 BrushControl_Update 推进时间。
 */
void BrushControl_Start(void);

/**
 * @brief  滚刷控制周期更新
 *   累加经过的时间, RUNNING 态下判断是否该结束:
 *     - 累加到 1.2 秒 (BRUSH_ROTATE_DURATION_MS) → 进入 DONE
 *     - 累加到 3.0 秒 (BRUSH_TIMEOUT_MS) → 进入 TIMEOUT
 *
 * @param  elapsed_ms  自上次调用以来经过的时间 (毫秒)
 *
 * @note   超时优先于完成判 (timeout > duration, 先判 timeout 防误判 DONE):
 */
void BrushControl_Update(uint32_t elapsed_ms);

/** @brief 查询滚刷当前状态 (IDLE/RUNNING/DONE/TIMEOUT) */
BrushState_t BrushControl_GetState(void);

/**
 * @brief  查询滚刷旋转控制信号
 * @return 1=正在旋转 (RUNNING 态), 0=停止 (其他态)
 * @note   本返回值由 motion_strategy 写入 MotionCommand_t.brush_cycle,
 *         传给外部电机驱动控制 PWM
 */
uint8_t BrushControl_GetSignal(void);

/** @brief 复位滚刷控制到 IDLE (停止+清零计时) */
void BrushControl_Reset(void);

#endif /* BRUSH_CONTROL_H */
