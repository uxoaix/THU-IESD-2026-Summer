#ifndef IMU_ODOMETRY_H
#define IMU_ODOMETRY_H

#include "motion_config.h"
#include "sensor_fusion.h"

/*
 * IMU + 编码器融合里程计调试模块。
 * 与自动/手动模式解耦：手动驾驶时也持续更新 home_x/home_y 与航向。
 */
void ImuOdometry_Init(void);
void ImuOdometry_Update(const WheelFeedback_t *wheels, float dt_s);

/* 当前位置设为原点 (0,0)，航向归零。蓝牙 SET_HOME 调用。 */
void ImuOdometry_SetHome(void);
void ImuOdometry_BeginReturn(void);
/* 返航途中绕墙: 车右偏多少, 锁定的返航方位角就跟着偏多少 (负 = 右旋)。 */
void ImuOdometry_NudgeReturnBearing(float delta_rad);

uint8_t ImuOdometry_IsOriginSet(void);
uint8_t ImuOdometry_IsImuAlive(void);

float ImuOdometry_GetHeadingRad(void);
float ImuOdometry_GetLinearVelocityCmS(void);
float ImuOdometry_GetYawRateRadS(void);

int16_t ImuOdometry_GetXcm(void);
int16_t ImuOdometry_GetYcm(void);
float ImuOdometry_GetDistanceCm(void);
int16_t ImuOdometry_GetHeadingDeg(void);
int16_t ImuOdometry_GetBearingDeg(void);

/* 未经标度校准的分源航向 (deg)，用于判断标度误差来自 IMU 还是编码器：
 * 原地转已知角度后，谁的增量偏离实际转角，问题就出在谁身上。 */
int16_t ImuOdometry_GetImuYawDeg(void);
int16_t ImuOdometry_GetEncHeadingDeg(void);

uint8_t ImuOdometry_GetReturnCommand(float *linear_cm_s, float *angular_rad_s);
void ImuOdometry_GetFusionState(FusionState_t *out);

#endif
