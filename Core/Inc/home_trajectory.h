#ifndef HOME_TRAJECTORY_H
#define HOME_TRAJECTORY_H

#include <stdint.h>

void HomeTrajectory_Init(void);
void HomeTrajectory_SetOrigin(float heading_rad);
void HomeTrajectory_BeginReturn(void);
void HomeTrajectory_NudgeReturnBearing(float delta_rad);
uint8_t HomeTrajectory_IsOriginSet(void);
void HomeTrajectory_Update(float linear_cm_s, float heading_rad, float dt_s);
uint8_t HomeTrajectory_GetReturnCommand(float heading_rad,
                                        float *linear_cm_s,
                                        float *angular_rad_s);
int16_t HomeTrajectory_GetXcm(void);
int16_t HomeTrajectory_GetYcm(void);
float HomeTrajectory_GetDistanceCm(void);
int16_t HomeTrajectory_GetRelativeHeadingDeg(float heading_rad);
int16_t HomeTrajectory_GetReturnBearingDeg(void);

#endif
