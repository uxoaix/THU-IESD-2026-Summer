#ifndef HOME_TRAJECTORY_H
#define HOME_TRAJECTORY_H

#include <stdint.h>

void HomeTrajectory_Init(void);
void HomeTrajectory_SetOrigin(float heading_rad);
uint8_t HomeTrajectory_IsOriginSet(void);
void HomeTrajectory_Update(float linear_cm_s, float heading_rad, float dt_s);

/* 返航直线段: 锁定方位角 → 每周期取指令 → 遇墙退避后把方位角右旋一档。 */
void HomeTrajectory_BeginReturn(void);
void HomeTrajectory_NudgeReturnBearing(float delta_rad);
uint8_t HomeTrajectory_GetReturnCommand(float heading_rad,
                                        float *linear_cm_s,
                                        float *angular_rad_s);

int16_t HomeTrajectory_GetXcm(void);
int16_t HomeTrajectory_GetYcm(void);
float HomeTrajectory_GetDistanceCm(void);
float HomeTrajectory_GetReturnBearingRad(void);

#endif /* HOME_TRAJECTORY_H */
