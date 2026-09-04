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

/* 当前位置设为原点 (0,0)，航向归零。开机初始化与蓝牙 SET_HOME 调用。
 * 航向归零会切断绝对航向的连续性，任务途中不要用。 */
void ImuOdometry_SetHome(void);
/* 只把 (0,0) 挪到当前位置，不动航向。卸货时标记下一轮原点用。 */
void ImuOdometry_MoveOriginHere(void);

/* 返航直线段：锁定朝向原点的方位角 → 每周期取指令 → 遇墙退避后右旋一档。
 * GetReturnCommand 返回 1 表示已进到 HOME_ARRIVAL_RADIUS_CM 内。 */
void ImuOdometry_BeginReturn(void);
void ImuOdometry_NudgeReturnBearing(float delta_rad);
uint8_t ImuOdometry_GetReturnCommand(float heading_rad,
                                     float *linear_cm_s,
                                     float *angular_rad_s);

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

void ImuOdometry_GetFusionState(FusionState_t *out);

#endif
