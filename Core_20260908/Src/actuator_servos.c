#include "actuator_servos.h"
#include "tim.h"

/*
 * 三路SG90和滚刷MG995分别使用独立脉宽参数；各执行器的START/END_DEG与
 * OUTBOUND/RETURN_MS已集中到
 * motion_config.h §21, 本文件不再重复定义, 避免"改了一处忘了另一处"。
 * actuator_servos.h 已 #include motion_config.h, 故本文件可直接使用这些宏。
 */

typedef enum {
  SERVO_CYCLE_IDLE = 0,
  SERVO_CYCLE_AT_END,
  SERVO_CYCLE_RETURNING
} ServoCycleState_t;

typedef struct {
  uint32_t channel;
  uint16_t start_deg;
  uint16_t end_deg;
  uint16_t outbound_ms;
  uint16_t return_ms;
  uint16_t min_pulse_us;
  uint16_t max_pulse_us;
  uint16_t angle_limit_deg;
  uint32_t state_tick;
  ServoCycleState_t state;
} ServoCycle_t;

static ServoCycle_t s_servo[ACTUATOR_SERVO_COUNT] = {
  {
    TIM_CHANNEL_1, DOOR_START_DEG, DOOR_END_DEG,
    DOOR_OUTBOUND_MS, DOOR_RETURN_MS,
    SG90_MIN_PULSE_US, SG90_MAX_PULSE_US, SG90_ANGLE_LIMIT_DEG,
    0U, SERVO_CYCLE_IDLE
  },
  {
    TIM_CHANNEL_2, CAMERA_START_DEG, CAMERA_END_DEG,
    CAMERA_OUTBOUND_MS, CAMERA_RETURN_MS,
    SG90_MIN_PULSE_US, SG90_MAX_PULSE_US, SG90_ANGLE_LIMIT_DEG,
    0U, SERVO_CYCLE_IDLE
  },
  {
    TIM_CHANNEL_3, BUCKET_START_DEG, BUCKET_END_DEG,
    BUCKET_OUTBOUND_MS, BUCKET_RETURN_MS,
    SG90_MIN_PULSE_US, SG90_MAX_PULSE_US, SG90_ANGLE_LIMIT_DEG,
    0U, SERVO_CYCLE_IDLE
  },
  {
    TIM_CHANNEL_4, BRUSH_START_DEG, BRUSH_END_DEG,
    BRUSH_OUTBOUND_MS, BRUSH_RETURN_MS,
    MG995_MIN_PULSE_US, MG995_MAX_PULSE_US, MG995_ANGLE_LIMIT_DEG,
    0U, SERVO_CYCLE_IDLE
  }
};

static uint16_t AngleToPulse(const ServoCycle_t *servo,
                             uint16_t angle_deg)
{
  if (angle_deg > servo->angle_limit_deg)
  {
    angle_deg = servo->angle_limit_deg;
  }

  return (uint16_t)(servo->min_pulse_us +
    ((uint32_t)(servo->max_pulse_us - servo->min_pulse_us) *
     angle_deg / servo->angle_limit_deg));
}

static void SetAngleRaw(const ServoCycle_t *servo, uint16_t angle_deg)
{
  __HAL_TIM_SET_COMPARE(&htim3,
                        servo->channel,
                        AngleToPulse(servo, angle_deg));
}

void ActuatorServos_Init(void)
{
  uint8_t i;

  for (i = 0U; i < ACTUATOR_SERVO_COUNT; i++)
  {
    s_servo[i].state = SERVO_CYCLE_IDLE;
    s_servo[i].state_tick = 0U;
    SetAngleRaw(&s_servo[i], s_servo[i].start_deg);
    HAL_TIM_PWM_Start(&htim3, s_servo[i].channel);
  }
}

void ActuatorServos_TriggerCycle(ActuatorServoId_t servo_id,
                                 uint32_t now_ms)
{
  ServoCycle_t *servo;

  if ((uint8_t)servo_id >= ACTUATOR_SERVO_COUNT)
  {
    return;
  }

  servo = &s_servo[servo_id];
  if (servo->state != SERVO_CYCLE_IDLE)
  {
    return;
  }

  SetAngleRaw(servo, servo->end_deg);
  servo->state_tick = now_ms;
  servo->state = SERVO_CYCLE_AT_END;
}

void ActuatorServos_Apply(const MotionCommand_t *command,
                          uint32_t now_ms)
{
  if (command->door_cycle != 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_DOOR, now_ms);
  }
  if (command->camera_cycle != 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_CAMERA, now_ms);
  }
  if (command->bucket_cycle != 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BUCKET, now_ms);
  }
  if (command->brush_cycle != 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BRUSH, now_ms);
  }
}

void ActuatorServos_Run(uint32_t now_ms)
{
  uint8_t i;

  for (i = 0U; i < ACTUATOR_SERVO_COUNT; i++)
  {
    ServoCycle_t *servo = &s_servo[i];

    if ((servo->state == SERVO_CYCLE_AT_END) &&
        ((uint32_t)(now_ms - servo->state_tick) >=
         servo->outbound_ms))
    {
      SetAngleRaw(servo, servo->start_deg);
      servo->state_tick = now_ms;
      servo->state = SERVO_CYCLE_RETURNING;
    }
    else if ((servo->state == SERVO_CYCLE_RETURNING) &&
             ((uint32_t)(now_ms - servo->state_tick) >=
              servo->return_ms))
    {
      servo->state = SERVO_CYCLE_IDLE;
    }
  }
}

uint8_t ActuatorServos_IsBusy(ActuatorServoId_t servo)
{
  if ((uint8_t)servo >= ACTUATOR_SERVO_COUNT)
  {
    return 0U;
  }
  return (s_servo[servo].state != SERVO_CYCLE_IDLE) ? 1U : 0U;
}

uint8_t ActuatorServos_IsReturning(ActuatorServoId_t servo)
{
  if ((uint8_t)servo >= ACTUATOR_SERVO_COUNT)
  {
    return 0U;
  }
  return (s_servo[servo].state == SERVO_CYCLE_RETURNING) ? 1U : 0U;
}

uint8_t ActuatorServos_SetAngle(ActuatorServoId_t servo_id,
                                uint16_t angle_deg)
{
  ServoCycle_t *servo;
  if ((uint8_t)servo_id >= ACTUATOR_SERVO_COUNT)
  {
    return 0U;
  }
  servo = &s_servo[servo_id];
  if (servo->state != SERVO_CYCLE_IDLE)
  {
    return 0U;
  }
  SetAngleRaw(servo, angle_deg);
  return 1U;
}

void ActuatorServos_Stop(void)
{
  uint8_t i;

  for (i = 0U; i < ACTUATOR_SERVO_COUNT; i++)
  {
    SetAngleRaw(&s_servo[i], s_servo[i].start_deg);
    s_servo[i].state = SERVO_CYCLE_IDLE;
    s_servo[i].state_tick = 0U;
  }
}

/* 旧接口仅为保持链接兼容；自动任务不再调用摄像头俯仰跟踪。 */
void Servo_Camera_Tilt(uint16_t angle_deg)
{
  (void)ActuatorServos_SetAngle(ACTUATOR_SERVO_CAMERA, angle_deg);
}

