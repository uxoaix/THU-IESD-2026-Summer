#ifndef __ACTUATOR_SERVOS_H__
#define __ACTUATOR_SERVOS_H__

#include "motion_config.h"

typedef enum {
  ACTUATOR_SERVO_DOOR = 0,    /* PA6 / TIM3_CH1 */
  ACTUATOR_SERVO_CAMERA,      /* PA7 / TIM3_CH2 */
  ACTUATOR_SERVO_BUCKET,      /* PB0 / TIM3_CH3 */
  ACTUATOR_SERVO_BRUSH,       /* PB1 / TIM3_CH4 */
  ACTUATOR_SERVO_COUNT
} ActuatorServoId_t;

void ActuatorServos_Init(void);
void ActuatorServos_TriggerCycle(ActuatorServoId_t servo,
                                 uint32_t now_ms);
void ActuatorServos_Apply(const MotionCommand_t *command,
                          uint32_t now_ms);
void ActuatorServos_Run(uint32_t now_ms);
uint8_t ActuatorServos_IsBusy(ActuatorServoId_t servo);
/* 立即置到指定角度并保持；若该通道正在往复则忽略。返回1表示已执行。 */
uint8_t ActuatorServos_SetAngle(ActuatorServoId_t servo,
                               uint16_t angle_deg);
void ActuatorServos_Stop(void);
#endif /* __ACTUATOR_SERVOS_H__ */
