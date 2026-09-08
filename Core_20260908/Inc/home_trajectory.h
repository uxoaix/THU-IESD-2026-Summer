#ifndef HOME_TRAJECTORY_H
#define HOME_TRAJECTORY_H

#include <stdint.h>

void HomeTrajectory_Init(void);
/* right_offset_cm: 原点相对车当前位置的右向偏移, 0 = 车脚下即原点。 */
void HomeTrajectory_SetOrigin(float heading_rad, float right_offset_cm);
uint8_t HomeTrajectory_IsOriginSet(void);
void HomeTrajectory_Update(float linear_cm_s, float heading_rad, float dt_s);

/* 返航直线段: 锁定方位角 → 每周期取指令 → 遇墙退避后把方位角右旋一档。
 *
 * 两个航向参数分工固定, 别搞混:
 *   heading_rad     融合航向, 与 (x,y) 同源, 只用于锁定那一刻的几何折算;
 *   imu_heading_rad 纯 IMU 航向, 锁定之后的角度闭环只吃它 (见 .c 的说明)。
 * GetReturnCommand 里 heading_rad 只在"调用方漏调 BeginReturn"的兜底分支用到。 */
void HomeTrajectory_BeginReturn(float heading_rad, float imu_heading_rad);
void HomeTrajectory_NudgeReturnBearing(float delta_rad);
uint8_t HomeTrajectory_GetReturnCommand(float heading_rad,
                                        float imu_heading_rad,
                                        float *linear_cm_s,
                                        float *angular_rad_s);

int16_t HomeTrajectory_GetXcm(void);
int16_t HomeTrajectory_GetYcm(void);
float HomeTrajectory_GetDistanceCm(void);
float HomeTrajectory_GetReturnBearingRad(void);

#endif /* HOME_TRAJECTORY_H */
