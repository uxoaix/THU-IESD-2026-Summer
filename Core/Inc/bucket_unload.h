/**
 * @file    bucket_unload.h
 * @brief   后斗卸载控制模块: 非阻塞时序控制 (升后斗→开门卸货→关门)
 *
 *   收集满 10 个物块, 车找到黑色卸货区后, 要执行一连串动作卸货:
 *     ① 升起后斗 (2 秒)——把后斗抬高, 让物块能滑出来
 *     ② 打开后门 (4 秒)——物块从后门滑出落到黑区
 *     ③ 关上后门 (0.5 秒)——准备下一轮收集
 *   本模块负责这 3 步的"顺序计时", 不直接驱动执行器。
 *
 *   IDLE (空闲)
 *     ↓ Start()
 *   LIFTING (升后斗, bucket_cycle=1, door=0)  持续 2.0s
 *     ↓ 时间到
 *   UNLOADING (开门卸货, bucket_cycle=0, door=1)  持续 4.0s
 *     ↓ 时间到
 *   CLOSING (关门, bucket_cycle=0, door=0)  持续 0.5s
 *     ↓ 时间到
 *   DONE (完成, bucket_cycle=0, door=0)  等待 motion_strategy 复位
 *
 *   bucket_cycle:     1=升起后斗, 0=保持原位
 *   door_cycle:       1=开后门 (物块滑出), 0=关后门
 */
#ifndef BUCKET_UNLOAD_H
#define BUCKET_UNLOAD_H

#include "motion_config.h"

/**
 * @brief  后斗状态枚举 (5 个阶段)
 */
typedef enum {
    BUCKET_IDLE = 0,       /* 空闲 */
    BUCKET_LIFTING,        /* 升后斗 (bucket_cycle=1) */
    BUCKET_UNLOADING,      /* 开门卸货 (door=1, 持续 BUCKET_UNLOAD_DURATION_MS) */
    BUCKET_CLOSING,        /* 关门 (door=0, 持续 BUCKET_DOOR_CLOSE_MS) */
    BUCKET_DONE            /* 完成 */
} BucketState_t;

/** @brief 启动卸载序列 (非阻塞), 进入 LIFTING 态 */
void BucketUnload_Start(void);

/**
 * @brief  后斗卸载周期更新
 *
 * 【作用】
 *   累加经过的时间, 每阶段到时自动切换到下一阶段:
 *     LIFTING (2.0s) → UNLOADING (4.0s) → CLOSING (0.5s) → DONE
 *
 * @param  elapsed_ms  自上次调用以来经过的时间 (毫秒)
 *
 * 【使用场景】
 *   motion_strategy 在每个 50ms 周期调用一次, 推进后斗时序
 */
void BucketUnload_Update(uint32_t elapsed_ms);

/** @brief 查询后斗当前状态 (IDLE/LIFTING/UNLOADING/CLOSING/DONE) */
BucketState_t BucketUnload_GetState(void);

/**
 * @brief  查询后斗升降信号
 * @return 1=升起 (LIFTING 态), 0=保持/下降 (其他态)
 * @note   本返回值由 motion_strategy 写入 MotionCommand_t.bucket_cycle
 */
uint8_t BucketUnload_GetLiftSignal(void);

/**
 * @brief  查询后门开闭信号
 * @return 1=开 (UNLOADING 态), 0=关 (其他态)
 * @note   本返回值由 motion_strategy 写入 MotionCommand_t.door_cycle
 */
uint8_t BucketUnload_GetDoorOpenSignal(void);

/** @brief 复位后斗控制到 IDLE (停止+清零计时) */
void BucketUnload_Reset(void);

#endif /* BUCKET_UNLOAD_H */
