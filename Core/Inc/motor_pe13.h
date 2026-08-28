#ifndef __MOTOR_PE13_H__
#define __MOTOR_PE13_H__

#include <stdint.h>

void MotorPe13_Init(void);
void MotorPe13_SetDuty(int16_t duty);
void MotorPe13_Stop(void);

#endif /* __MOTOR_PE13_H__ */
