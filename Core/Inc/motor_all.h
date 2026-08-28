#ifndef __MOTOR_ALL_H__
#define __MOTOR_ALL_H__

#include <stdint.h>

typedef enum {
  MOTOR_PE9 = 0,
  MOTOR_PE11,
  MOTOR_PE13,
  MOTOR_PE14,
  MOTOR_CHANNEL_COUNT
} MotorId_t;

/* modes按PE9、PE11、PE13、PE14顺序传入；0通道保持硬件低电平 */
void MotorAll_Init(const uint8_t modes[MOTOR_CHANNEL_COUNT]);
void MotorAll_SetDuty(MotorId_t id, int16_t duty);
void MotorAll_Stop(MotorId_t id);
void MotorAll_StopAll(void);

#endif /* __MOTOR_ALL_H__ */
