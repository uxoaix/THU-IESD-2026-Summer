/*
 * 【文件说明】 motor_fl.c —— 前左轮电机驱动 (FL = Front Left)
 *
 * 【职责】前左轮的 PWM + 方向驱动。结构和 motor_pe14.c 类似，
 *        但额外多了一个 Disable 函数 + Brake 函数 + DisableRearChannels 初始化逻辑。
 *
 * 【类比理解】
 *   - FL = Front Left (前左), 是 L298N 四路里的第 1 路
 *   - TIM1_CH1 (PE9) 是 PWM, PE0/PE1 是 IN1/IN2 方向
 *   - DisableRearChannels 就像告诉 L298N "先把另外两路关掉, 我只开前左这一路"
 *
 * 【硬件绑定】
 *   PWM  → PE9 / TIM1_CH1
 *   IN1  → PE0
 *   IN2  → PE1
 *   REAR_EN/IN → 后轮的 EN 和 IN1/IN2 脚, 初始化时全部拉低禁用
 *
 * 【额外功能 (比 motor_pe14.c 多)】
 *   - MotorFl_Brake   : IN1+IN2 都拉高 + PWM 满载, 相当于短路刹车
 *   - MotorFl_Disable : 彻底停转 + 把 PWM 口切回普通 GPIO 输出避免悬空
 */

#include "motor_fl.h"
#include "board_fr.h"

/* 设置 IN1/IN2 方向脚 —— 正转 (SET/RESET), 反转 (RESET/SET), 刹停 (RESET/RESET) */
static void MotorFl_SetDirPins(GPIO_PinState in1, GPIO_PinState in2)
{
  HAL_GPIO_WritePin(FL_IN1_PORT, FL_IN1_PIN, in1);
  HAL_GPIO_WritePin(FL_IN2_PORT, FL_IN2_PIN, in2);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFl_DisableRearChannels
 * 【作用】初始化时把后轮的 EN 和 IN 全部拉低, 确保只有前轮在工作
 * 【原因】硬件上 4 路电机共享 L298N, 如果不主动关后轮的 EN,
 *         它们会悬空被 L298N 误触发
 * ------------------------------------------------------------------------- */
static void MotorFl_DisableRearChannels(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = REAR_EN_PINS;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(REAR_EN_PORT, &gpio);
  HAL_GPIO_WritePin(REAR_EN_PORT, REAR_EN_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(REAR_IN_PORT, REAR_IN_PINS, GPIO_PIN_RESET);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFl_Init
 * 【作用】前左轮上电初始化 —— 关后轮 → 方向归零 → 启动 PWM
 * 【调用时机】main.c 里 MotorAll_Init 之前
 * ------------------------------------------------------------------------- */
void MotorFl_Init(void)
{
  MotorFl_DisableRearChannels();
  MotorFl_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(FL_PWM_TIM, FL_PWM_CHANNEL, 0);
  HAL_TIM_PWM_Start(FL_PWM_TIM, FL_PWM_CHANNEL);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFl_SetDuty
 * 【作用】设置前左轮占空比 + 方向
 * 【参数】duty - 正数=正转, 负数=反转, 0=刹停, 会被钳位到 [-FR_PWM_MAX, +FR_PWM_MAX]
 * ------------------------------------------------------------------------- */
void MotorFl_SetDuty(int16_t duty)
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
    MotorFl_SetDirPins(GPIO_PIN_SET, GPIO_PIN_RESET);
    pulse = (uint16_t)duty;
  }
  else if (duty < 0)
  {
    MotorFl_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_SET);
    pulse = (uint16_t)(-duty);
  }
  else
  {
    MotorFl_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_RESET);
    pulse = 0U;
  }

  __HAL_TIM_SET_COMPARE(FL_PWM_TIM, FL_PWM_CHANNEL, pulse);
}

/* 让前左轮自由停转 —— 等价于 SetDuty(0) */
void MotorFl_Stop(void)
{
  MotorFl_SetDuty(0);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFl_Brake
 * 【作用】短路刹车 —— IN1+IN2 同时拉高, PWM 满载
 * 【效果】电机两端被 L298N 同时接高, 形成短路, 轮子被 "抱死"
 * 【区别】Stop() 是让电机断电自由滑行; Brake() 是主动刹车
 * ------------------------------------------------------------------------- */
void MotorFl_Brake(void)
{
  MotorFl_SetDirPins(GPIO_PIN_SET, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(FL_PWM_TIM, FL_PWM_CHANNEL, (uint32_t)FR_PWM_MAX);
}

/* --------------------------------------------------------------------------
 * 【函数】MotorFl_Disable
 * 【作用】彻底关闭电机 —— 停 PWM + 方向归零 + PWM 口切普通 GPIO 输出
 * 【细节】FL_PWM_PIN (PE9) 从 AF_PP 改成 Output_PP 低电平,
 *         不依赖 L298N 板上的 10kΩ 下拉, 直接把 EN 压死
 * ------------------------------------------------------------------------- */
void MotorFl_Disable(void)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_TIM_PWM_Stop(FL_PWM_TIM, FL_PWM_CHANNEL);
  __HAL_TIM_SET_COMPARE(FL_PWM_TIM, FL_PWM_CHANNEL, 0);
  MotorFl_SetDirPins(GPIO_PIN_RESET, GPIO_PIN_RESET);

  /* 不依赖L298N板上的下拉，直接把ENB/PE11压低 */
  gpio.Pin = FL_PWM_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FL_PWM_PORT, &gpio);
  HAL_GPIO_WritePin(FL_PWM_PORT, FL_PWM_PIN, GPIO_PIN_RESET);
}
