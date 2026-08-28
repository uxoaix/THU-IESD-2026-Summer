/*
 * 【文件说明】 motor_fr.c —— 前右轮电机驱动 (FR = Front Right)
 *
 * 【职责】前右轮的 PWM + 方向驱动。结构和 motor_fl.c 完全对称:
 *        都有 SetDirPins + DisableOtherChannels + Init + SetDuty + Stop + Brake + Disable。
 *
 * 【类比理解】四个电机文件就像四栋房子, 每栋结构一样 (函数列表相同),
 *        只是 "地址" 不同 (PE9/PE11/PE13/PE14)。统一接口让上层 MotorAll_SetDuty
 *        可以一视同仁地调用, 不需要关心具体是哪个轮。
 *
 * 【硬件绑定】
 *   PWM  → PE11 / TIM1_CH2
 *   IN1  → PE2
 *   IN2  → PE3
 *
 * 【初始化要点】DisableOtherChannels 把后轮 EN 脚设为 INPUT + NOPULL,
 *        让 L298N 板上的 10kΩ 下拉把后轮通道钳死在禁用状态, 避免悬空。
 */

#include "motor_fr.h"
#include "board_fr.h"

/* 私有方向设置 —— IN1/IN2 决定转向 (同 motor_fl.c) */
static void MotorFr_SetDirPins(GPIO_PinState in1, GPIO_PinState in2)
{
  HAL_GPIO_WritePin(FR_IN1_PORT, FR_IN1_PIN, in1);
  HAL_GPIO_WritePin(FR_IN2_PORT, FR_IN2_PIN, in2);
}

/*
 * 本阶段只驱动两个前轮。把后轮 PE13/PE14 从 TIM1 复用推挽改回输入，
 * 让 EN 端的 10 kΩ 下拉把两个后轮通道钳在禁用状态。
 */
static void MotorFr_DisableOtherChannels(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin  = REAR_EN_PINS;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(REAR_EN_PORT, &gpio);

  HAL_GPIO_WritePin(REAR_IN_PORT, REAR_IN_PINS, GPIO_PIN_RESET);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFr_Init
 * 【作用】前右轮上电初始化 —— 关后轮 EN → 方向归零 → 0 占空比启动 PWM
 * 【上电顺序】先方向后 PWM, 避免方向脚还不稳定时 PWM 已经输出触发 L298N
 * ------------------------------------------------------------------------- */
void MotorFr_Init(void)
{
  MotorFr_DisableOtherChannels();

  /* 电气表要求的上电顺序：先给出方向电平，再以 0 占空比启动 PWM */
  MotorFr_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(FR_PWM_TIM, FR_PWM_CHANNEL, 0);

  HAL_TIM_PWM_Start(FR_PWM_TIM, FR_PWM_CHANNEL);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFr_SetDuty —— 前右轮占空比 + 方向
 * 【参数】duty - 正数正转, 负数反转, 0 刹停; 钳位到 [-FR_PWM_MAX, +FR_PWM_MAX]
 * ------------------------------------------------------------------------- */
void MotorFr_SetDuty(int16_t duty)
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
    MotorFr_SetDirPins(GPIO_PIN_SET, GPIO_PIN_RESET);
    pulse = (uint16_t)duty;
  }
  else if (duty < 0)
  {
    MotorFr_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_SET);
    pulse = (uint16_t)(-duty);
  }
  else
  {
    MotorFr_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_RESET);
    pulse = 0U;
  }

  __HAL_TIM_SET_COMPARE(FR_PWM_TIM, FR_PWM_CHANNEL, pulse);
}

/* 前右轮自由停转 —— SetDuty(0) */
void MotorFr_Stop(void)
{
  MotorFr_SetDuty(0);
}

/* 前右轮短路刹车 —— IN1+IN2 都拉高, PWM 满载, 电机被抱死 */
void MotorFr_Brake(void)
{
  MotorFr_SetDirPins(GPIO_PIN_SET, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(FR_PWM_TIM, FR_PWM_CHANNEL, (uint32_t)FR_PWM_MAX);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFr_Disable
 * 【作用】彻底关闭前右轮 —— 停 PWM + 方向归零 + PWM 口切普通 GPIO 低电平
 * 【细节】FR_PWM_PIN (PE11) 从 AF_PP 改成 Output_PP 低电平, 主动压死 L298N EN 脚
 * ------------------------------------------------------------------------- */
void MotorFr_Disable(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_TIM_PWM_Stop(FR_PWM_TIM, FR_PWM_CHANNEL);
  __HAL_TIM_SET_COMPARE(FR_PWM_TIM, FR_PWM_CHANNEL, 0);
  MotorFr_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_RESET);

  gpio.Pin = FR_PWM_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FR_PWM_PORT, &gpio);
  HAL_GPIO_WritePin(FR_PWM_PORT, FR_PWM_PIN, GPIO_PIN_RESET);
}

