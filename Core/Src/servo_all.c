#include "servo_all.h"
#include "tim.h"

/*
 * TIM3计数频率1 MHz，ARR=19999：
 *   1个计数=1 us，周期=20 ms（50 Hz）。
 * 500~2500 us映射为0~180度。
 */
#define SERVO_MIN_US             500U
#define SERVO_MAX_US             2500U
#define SERVO_MAX_ANGLE          180U
#define SERVO_STEP_MS            20U
#define SERVO_STEP_DEG           1U

static uint16_t s_angle;
static int8_t s_direction;
static uint32_t s_step_tick;

static uint16_t AngleToPulse(uint16_t angle)
{
  if (angle > SERVO_MAX_ANGLE)
  {
    angle = SERVO_MAX_ANGLE;
  }

  return (uint16_t)(SERVO_MIN_US +
                    ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * angle /
                     SERVO_MAX_ANGLE));
}

static void SetAllChannels(uint16_t angle)
{
  uint16_t pulse = AngleToPulse(angle);

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse); /* PA6，小舵机 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse); /* PA7，小舵机 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse); /* PB0，小舵机 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pulse); /* PB1，大舵机 */
}

void ServoAll_Init(void)
{
  s_angle = 0U;
  s_direction = 1;
  s_step_tick = HAL_GetTick();

  SetAllChannels(s_angle);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

void ServoAll_Run(uint32_t now_ms)
{
  uint32_t steps;

  if ((uint32_t)(now_ms - s_step_tick) < SERVO_STEP_MS)
  {
    return;
  }

  steps = (uint32_t)(now_ms - s_step_tick) / SERVO_STEP_MS;
  s_step_tick += steps * SERVO_STEP_MS;

  while (steps-- > 0U)
  {
    if (s_direction > 0)
    {
      if (s_angle < SERVO_MAX_ANGLE)
      {
        s_angle += SERVO_STEP_DEG;
      }
      if (s_angle >= SERVO_MAX_ANGLE)
      {
        s_angle = SERVO_MAX_ANGLE;
        s_direction = -1;
      }
    }
    else
    {
      if (s_angle > SERVO_STEP_DEG)
      {
        s_angle -= SERVO_STEP_DEG;
      }
      else
      {
        s_angle = 0U;
        s_direction = 1;
      }
    }
  }

  SetAllChannels(s_angle);
}
