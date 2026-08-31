/**
 * @file    block_alignment.h
 * @brief   物块对准模块 (水平模糊控制 + 垂直俯仰跟踪)
 *
 * 【两部分功能】
 *   ① 水平对准 (alignment_error → 车体角速度修正): 7档模糊控制
 *   ② 垂直跟踪 (y_offset_px → 摄像头俯仰角): 比例控制 + 死区 + 限速, 让摄像头俯仰把物块拉到视野垂直中央
 *
 * 【7 个档位 (模糊集) — 水平用】
 *   NB (极大偏左)  NM (中等偏左)  NS (稍偏左)
 *   ZE (正中)
 *   PS (稍偏右)  PM (中等偏右)  PB (极大偏右)
 *
 * 【输入输出】
 *   水平: 输入扣除安装偏移后的误差 (像素, [-160, +160]) → 输出角速度修正
 *   垂直: 输入 y_offset_px (像素, 负=上, 正=下) → 控制俯仰舵机角度
 */
#ifndef BLOCK_ALIGNMENT_H
#define BLOCK_ALIGNMENT_H

#include "motion_config.h"

/* 外部依赖 (俯仰舵机驱动) *
 *
 * 【已知不匹配 M2/M11】当前 ActuatorServos 只支持触发 0→180→0 往复, 无"置位到指定角度"API,
 *   故 Servo_Camera_Tilt 暂无实际执行方。需舵机团队扩展 SetAngle(id, angle) API 才能让
 *   摄像头俯仰跟踪真正生效 (见 motion_config.h §20)。在此之前, 本模块逻辑保留,
 *   内部状态 s_tilt_angle 仍维护, 调用为空操作 (由驱动层提供空实现)。 */

/**
 * @brief  控制摄像头俯仰舵机转到指定角度
 * @param  angle_deg  目标角度 (度, 90=正前方, <90=仰视, >90=俯视)
 * @note   非阻塞 (不等转完)。由舵机驱动模块实现。
 */
void Servo_Camera_Tilt(uint16_t angle_deg);

/* 水平对准 (模糊控制) */

/**
 * @brief   模糊控制物块对准
 *
 * 【作用】
 *   输入摄像头看到的"目标偏左偏右多少像素", 输出"应该转多快的角速度"。
 *
 * @param   x_offset_px  已扣除TRACKING_CENTER_OFFSET_PX的对准误差
 *                    - 负数 = 目标偏左 → 返回正数 (左转把目标拉回中央)
 *                    - 正数 = 目标偏右 → 返回负数 (右转把目标拉回中央)
 *                    - 0    = 目标居中 → 返回 0 (不转)
 * @return  角速度修正 (rad/s, 正=逆时针/左转, 负=顺时针/右转)
 *
 * 【使用场景】
 *   motion_strategy 的 PRE_CENTERING 和 TARGET_TRACKING 状态调用本函数,
 *   把返回值传给 Motion_Rotate() 让车原地微调或边走边修正。
 *
 * @note    死区内 (|x_offset_px| <= TRACKING_DEADZONE_PX) 返回0。
 *          输出限幅在 ±1.0 rad/s (MAX_ANGULAR_SPEED_RAD_S)。
 */
float BlockAlignment_GetAngularCorrection(int16_t x_offset_px);

/* 垂直跟踪 (俯仰舵机) */

/**
 * @brief  复位摄像头俯仰角到正前方 (90°)
 *
 * 【作用】
 *   把内部记录的当前俯仰角清为 CAMERA_TILT_CENTER_DEG, 并立即发送指令到舵机。
 *
 * 【使用场景】
 *   ① 系统初始化 / 紧急停车 (MotionStrategy_Init / Stop)
 *   ② 进入追踪状态首周期 (TARGET_TRACKING 入口)
 *   ③ 目标丢失回到搜索 (ROTATE_SEARCH 前)
 *   ④ 进入新一轮任务 (MOTION_STATE_INIT)
 *
 * @note   非阻塞, 舵机会自行平滑转到目标角度。
 */
void BlockAlignment_ResetCameraTilt(void);

/**
 * @brief  根据物块垂直偏移更新摄像头俯仰角
 *
 * 【作用】 (TARGET_TRACKING 状态, 物块已检测到时每周期调用)
 *   ① 死区判断: |y_offset_px| < DEADZONE → 保持当前角度, 不调整 (防抖)
 *   ② 比例控制: correction = y_offset_px × KP
 *      y > 0 (物块偏下) → 增大角度 (俯视)
 *      y < 0 (物块偏上) → 减小角度 (仰视)
 *   ③ 限幅: 角度限制在 [MIN, MAX] 范围内
 *   ④ 限速: 单周期变化 ≤ MAX_STEP (防舵机猛甩/视野抖动)
 *   ⑤ 若角度有整数变化才发送指令 (减少总线流量)
 *
 * @param  y_offset_px  目标垂直偏移 (像素, 负=偏上, 正=偏下, 0=居中)
 *
 * @note   非阻塞。算法仅含乘法+比较, 资源消耗极低。
 *         限速保证即使 y_offset_px 突变, 舵机也平滑跟随, 不会视野抖动。
 *         死区保证目标在中央附近时不会反复微调。
 */
void BlockAlignment_UpdateCameraTilt(int16_t y_offset_px);

#endif /* BLOCK_ALIGNMENT_H */
