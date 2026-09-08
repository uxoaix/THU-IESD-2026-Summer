#ifndef __MOTOR_PE14_H__
#define __MOTOR_PE14_H__

#include <stdint.h>

void MotorPe14_Init(void);
void MotorPe14_SetDuty(int16_t duty);
void MotorPe14_Stop(void);

#endif /* __MOTOR_PE14_H__ */
