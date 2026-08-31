#ifndef MOTION_STRATEGY_H
#define MOTION_STRATEGY_H

#include "motion_config.h"

void MotionStrategy_Init(void);
void MotionStrategy_Update(const VisionData_t *vision,
                           const WheelFeedback_t *wheels,
                           const WallSensorData_t *wall,
                           uint32_t elapsed_ms,
                           MotionCommand_t *command);
void Motion_Line(float linear_speed, MotionCommand_t *out);
void Motion_Rotate(float angular_speed, MotionCommand_t *out);
void Motion_Drive(float linear_speed, float angular_speed,
                  MotionCommand_t *out);
void Motion_StopOutput(MotionCommand_t *out);
void MotionStrategy_Stop(void);
MotionState_t MotionStrategy_GetState(void);
const char *MotionStrategy_GetStateName(void);
uint8_t MotionStrategy_GetCollectedCount(void);
uint8_t MotionStrategy_IsReverseHead(void);

#endif
