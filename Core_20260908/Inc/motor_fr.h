#ifndef __MOTOR_FR_H__
#define __MOTOR_FR_H__

#include <stdint.h>

/* 右前轮 L298N 通道：PE9 PWM + PE0/PE1 方向。引脚定义见 board_fr.h */

/* 启动 PE9 的 PWM，并把另外三路 EN 交回下拉禁用状态 */
void MotorFr_Init(void);

/* duty 范围 ±FR_PWM_MAX，正数前进，负数后退，0 为滑行（IN1 = IN2 = 0） */
void MotorFr_SetDuty(int16_t duty);

/* 滑行停车，等价于 MotorFr_SetDuty(0) */
void MotorFr_Stop(void);

/* 能耗制动：IN1 = IN2 = 1 并给满占空比，让电机两端短接 */
void MotorFr_Brake(void);

/* 停止TIM1_CH1并把PE9、PE0、PE1强制为低电平 */
void MotorFr_Disable(void);

#endif /* __MOTOR_FR_H__ */
