#ifndef __WHEEL_SPEED_CONTROL_H__
#define __WHEEL_SPEED_CONTROL_H__

#include "motion_config.h"
#include <stdint.h>

typedef enum {
  WHEEL_SPEED_FAULT_NONE = 0,
  WHEEL_SPEED_FAULT_ENCODER_REVERSED = 1,
  WHEEL_SPEED_FAULT_NO_FEEDBACK = 2
} WheelSpeedFault_t;

/*
 * 四轮底层速度闭环。上层只下发带符号目标速度：
 * 正数=整车前进方向，负数=整车后退方向，0=停止。
 */
void WheelSpeedControl_Init(void);
void WheelSpeedControl_SetTargetMmS(WheelId_t wheel,
                                    float signed_target_mm_s);
void WheelSpeedControl_Run(float dt_s, uint8_t enabled);
void WheelSpeedControl_StopAll(void);
void WheelSpeedControl_ResetFault(WheelId_t wheel);

float WheelSpeedControl_GetTargetMmS(WheelId_t wheel);
float WheelSpeedControl_GetSpeedMmS(WheelId_t wheel);
int16_t WheelSpeedControl_GetDuty(WheelId_t wheel);
WheelSpeedFault_t WheelSpeedControl_GetFault(WheelId_t wheel);

#endif /* __WHEEL_SPEED_CONTROL_H__ */
