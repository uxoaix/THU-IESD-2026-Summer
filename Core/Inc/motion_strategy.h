#ifndef MOTION_STRATEGY_H
#define MOTION_STRATEGY_H

#include "motion_config.h"

void MotionStrategy_Init(void);
void MotionStrategy_Update(const VisionData_t *vision,
                           const WheelFeedback_t *wheels,
                           const VisionWallData_t *vision_wall,
                           const VisionArrivalData_t *vision_arrival,
                           uint32_t elapsed_ms,
                           MotionCommand_t *command);
void Motion_Line(float linear_speed, MotionCommand_t *out);
void Motion_Rotate(float angular_speed, MotionCommand_t *out);
void Motion_Drive(float linear_speed, float angular_speed,
                  MotionCommand_t *out);
void Motion_StopOutput(MotionCommand_t *out);
void MotionStrategy_Stop(void);
void MotionStrategy_RequestReturn(void);
/* 顺时针原地旋转指定角度 (°)，用IMU融合航向闭环。 */
void MotionStrategy_RequestRotateCw(float target_deg);
/* 蓝牙单次任务(返航/掉头)完成标志，读取后自动清零。 */
uint8_t MotionStrategy_TakeTaskComplete(void);
MotionState_t MotionStrategy_GetState(void);
const char *MotionStrategy_GetStateName(void);
uint8_t MotionStrategy_GetCollectedCount(void);

#endif
