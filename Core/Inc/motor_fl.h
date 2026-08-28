#ifndef __MOTOR_FL_H__
#define __MOTOR_FL_H__

#include <stdint.h>

/* 左前轮 L298N 通道：PE11 PWM + PE2/PE3 方向 */
void MotorFl_Init(void);
void MotorFl_SetDuty(int16_t duty);
void MotorFl_Stop(void);
void MotorFl_Brake(void);

/* 停止TIM1_CH2并把PE11、PE2、PE3强制为低电平 */
void MotorFl_Disable(void);

#endif /* __MOTOR_FL_H__ */
