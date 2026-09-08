/*
 * 【文件说明】 motor_pe14.c —— 后右轮电机驱动 (PE14)
 *
 * 【职责】本模块负责后右轮的单向 PWM + 双方向 IO 驱动。
 *        它是 L298N 四路里的第 4 路，对应 PE14 PWM (TIM1_CH4) + PE6/PE7 方向。
 *        整个文件逻辑非常统一：一个私有的方向设置函数 + 一个占空比设置函数。
 *
 * 【类比理解】
 *   - TIM1_CH4 就像水管龙头，控制水流粗细 (PWM 占空比)
 *   - PE6/PE7 就像双向阀门：SET/RESET = 正转, RESET/SET = 反转, 都是 RESET = 刹停
 *   - PE14 是 PWM 输出口 (AF_PP 复用推挽), 不用的时候要切回普通 GPIO 避免悬空
 *
 * 【硬件绑定】
 *   PWM  → PE14 / TIM1_CH4 (Full Remap 后)
 *   IN1  → PE6
 *   IN2  → PE7
 *   最大占空比 → FR_PWM_MAX (在 board_fr.h / motion_config.h 定义)
 *
 * 【注意事项】
 *   - 占空比会被钳位在 [-FR_PWM_MAX, +FR_PWM_MAX] 之间, 传超了会自动裁剪
 *   - SetDirection 用 static 修饰, 只能被本文件内部的 SetDuty 调用, 外部看不到
 */

#include "motor_pe14.h"
#include "board_fr.h"

/* --------------------------------------------------------------------------
 * 【函数】SetDirection
 * 【作用】设置电机方向脚 IN1/IN2 的电平组合, 决定正转 / 反转 / 刹停
 * 【参数】in1 - IN1 电平 (GPIO_PIN_SET 或 GPIO_PIN_RESET)
 *         in2 - IN2 电平 (GPIO_PIN_SET 或 GPIO_PIN_RESET)
 * 【返回】无
 * 【时机】每次 SetDuty 被调用时根据占空比正负来调用
 * ------------------------------------------------------------------------- */
static void SetDirection(GPIO_PinState in1, GPIO_PinState in2)
{
  HAL_GPIO_WritePin(PE14_IN1_PORT, PE14_IN1_PIN, in1);
  HAL_GPIO_WritePin(PE14_IN2_PORT, PE14_IN2_PIN, in2);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorPe14_Init
 * 【作用】初始化后右轮电机 —— 配置 PWM 引脚 + 清零方向 + 启动 PWM
 * 【参数】无
 * 【返回】无
 * 【调用时机】main.c 的 MX_GPIO_Init() 之后, 首次调用 SetDuty 之前
 * 【上电顺序】先配 PE14 为 AF_PP → 方向脚拉低 (刹车) → 以 0 占空比启动 PWM
 * ------------------------------------------------------------------------- */
void MotorPe14_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = PE14_PWM_PIN;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PE14_PWM_PORT, &gpio);

  SetDirection(GPIO_PIN_RESET, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(PE14_PWM_TIM, PE14_PWM_CHANNEL, 0);
  HAL_TIM_PWM_Start(PE14_PWM_TIM, PE14_PWM_CHANNEL);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorPe14_SetDuty
 * 【作用】设置后右轮的方向和 PWM 占空比
 * 【参数】duty - 目标占空比, 正数=正转, 负数=反转, 0=刹停
 *               取值范围 [-FR_PWM_MAX, +FR_PWM_MAX], 超出会被钳位
 * 【返回】无
 * 【核心逻辑】
 *   1. 先把 duty 钳位到合法范围
 *   2. 根据 duty 正负设置 IN1/IN2 方向
 *   3. 用 __HAL_TIM_SET_COMPARE 写 TIM1_CCR4 寄存器
 * ------------------------------------------------------------------------- */
void MotorPe14_SetDuty(int16_t duty)
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

  __HAL_TIM_SET_COMPARE(PE14_PWM_TIM, PE14_PWM_CHANNEL, pulse);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorPe14_Stop
 * 【作用】让后右轮刹停 —— 等价于 SetDuty(0)
 * 【参数】无
 * 【返回】无
 * ------------------------------------------------------------------------- */
void MotorPe14_Stop(void)
{
  MotorPe14_SetDuty(0);
}
