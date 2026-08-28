#ifndef __ULTRASONIC_FRONT_H__
#define __ULTRASONIC_FRONT_H__

#include "motion_config.h"

void UltrasonicFront_Init(void);
void UltrasonicFront_Run(uint32_t now_ms);
void UltrasonicFront_OnEchoEdge(void);
void UltrasonicFront_GetLatest(WallSensorData_t *wall);

#endif
