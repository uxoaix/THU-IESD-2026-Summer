/*
 * 【文件说明】 motor_pe13.c —— 后左轮电机驱动 (PE13)
 *
 * 【职责】后左轮 PWM + 方向驱动, 和 motor_pe14.c 几乎一模一样,
 *        只是硬件绑定换成 PE13 PWM (TIM1_CH3) + PE4/PE5 方向。
 *
 * 【特殊之处 —— 初始化顺序依赖】
 *   MotorFl_Init() 会调用 MotorFl_DisableRearChannels(), 它会把 PE13/PE14
 *   的 PWM 口暂时改成普通 GPIO 输出低电平。所以 MotorPe13_Init() 必须
 *   在 MotorFl_Init() 之后调用 —— 这样它才能把 PE13 重新切回 AF_PP 复用输出。
 *
 * 【硬件绑定】
 *   PWM  → PE13 / TIM1_CH3
 *   IN1  → PE4
 *   IN2  → PE5
 *
 * 【其他细节同 motor_pe14.c】 —— 占空比钳位 ±FR_PWM_MAX,
 *        SetDirection static 私有, SetDuty 里统一处理方向 + 写 CCR 寄存器。
 */

#include "motor_pe13.h"
#include "board_fr.h"

/* 私有方向设置函数 —— IN1/IN2 电平组合决定转向 */
static void SetDirection(GPIO_PinState in1, GPIO_PinState in2)
{
  HAL_GPIO_WritePin(PE13_IN1_PORT, PE13_IN1_PIN, in1);
  HAL_GPIO_WritePin(PE13_IN2_PORT, PE13_IN2_PIN, in2);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorPe13_Init
 * 【作用】恢复 PE13 的 TIM1 复用输出 + 启动 PWM
 * 【前置条件】MotorFl_Init() 必须先执行过, 否则 PE13 还在 GPIO 模式
 * ------------------------------------------------------------------------- */
void MotorPe13_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* MotorFl_Init会把后轮EN压低，因此在这里恢复PE13的TIM1复用输出。 */
  gpio.Pin = PE13_PWM_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PE13_PWM_PORT, &gpio);

  SetDirection(GPIO_PIN_RESET, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(PE13_PWM_TIM, PE13_PWM_CHANNEL, 0);
  HAL_TIM_PWM_Start(PE13_PWM_TIM, PE13_PWM_CHANNEL);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorPe13_SetDuty —— 后左轮占空比 + 方向设置 (逻辑同 PE14)
 * 【参数】duty - 正数正转, 负数反转, 0 刹停; 超范围自动钳位
 * ------------------------------------------------------------------------- */
void MotorPe13_SetDuty(int16_t duty)
{
  uint16_t pulse;

  if (duty > (int16_t)FR_PWM_MAX)
  {
    duty = (int16_t)FR_PWM_MAX;
  }
  else if (duty < -(int16_t)FR_PWM_MAX)
  {
    duty = -(int16_t)FR_PWM_MAX;
  }

  if (duty > 0)
  {
    SetDirection(GPIO_PIN_SET, GPIO_PIN_RESET);
    pulse = (uint16_t)duty;
  }
  else if (duty < 0)
  {
    SetDirection(GPIO_PIN_RESET, GPIO_PIN_SET);
    pulse = (uint16_t)(-duty);
  }
  else
  {
    SetDirection(GPIO_PIN_RESET, GPIO_PIN_RESET);
    pulse = 0U;
  }

  __HAL_TIM_SET_COMPARE(PE13_PWM_TIM, PE13_PWM_CHANNEL, pulse);
}

/* 后左轮刹停 —— SetDuty(0) */
void MotorPe13_Stop(void)
{
  MotorPe13_SetDuty(0);
}
