/*
 * 【文件说明】 motor_all.c —— 四路电机统一驱动 (全局封装)
 *
 * 【职责】把 FL / FR / PE13 / PE14 四个独立电机模块统一收敛到一个接口层。
 *        上层代码 (如运动策略) 只需要调 MotorAll_SetDuty(MOTOR_ID_FR, 1000)
 *        就可以控制任意一路电机, 不用关心具体硬件绑定。
 *
 * 【设计思路 —— 数据驱动】
 *   - s_hw[] 是一个硬件配置表, 每行记录一个电机的 PWM 通道 + 端口 + IN1/IN2 脚
 *   - 初始化时先把四路 PWM 全部关闭, 然后根据 modes[] 数组只启用需要的通道
 *   - SetDuty 根据 MotorId_t 查表找到对应的硬件配置, 然后统一处理
 *
 * 【类比理解】
 *   四个独立电机文件 = 四台独立的单缸发动机
 *   motor_all.c = 一台四缸发动机的总控台, 一个拨杆控制哪缸工作、转速多少
 *
 * 【硬件绑定】TIM1 Full Remap 后: CH1→PE9, CH2→PE11, CH3→PE13, CH4→PE14
 *   s_hw[0] = FL (前左) → PE9 PWM, PE0 IN1, PE1 IN2
 *   s_hw[1] = FR (前右) → PE11 PWM, PE2 IN1, PE3 IN2
 *   s_hw[2] = PE13 (后左) → PE13 PWM, PE4 IN1, PE5 IN2
 *   s_hw[3] = PE14 (后右) → PE14 PWM, PE6 IN1, PE7 IN2
 */

#include "motor_all.h"
#include "board_fr.h"

/* MotorHw_t —— 硬件配置表的一行, 描述一个电机需要的所有端口/引脚信息 */
typedef struct {
  uint32_t channel;       /* TIM 通道号 (TIM_CHANNEL_1/2/3/4) */
  GPIO_TypeDef *pwm_port; /* PWM 输出口 (都是 GPIOE) */
  uint16_t pwm_pin;       /* PWM 输出脚 (PE9/PE11/PE13/PE14) */
  GPIO_TypeDef *in1_port; /* IN1 方向口 */
  uint16_t in1_pin;       /* IN1 方向脚 */
  GPIO_TypeDef *in2_port; /* IN2 方向口 */
  uint16_t in2_pin;       /* IN2 方向脚 */
} MotorHw_t;

/* 四路电机的硬编码配置表 —— 用索引 0~3 对应 MotorId_t */
static const MotorHw_t s_hw[MOTOR_CHANNEL_COUNT] = {
  {TIM_CHANNEL_1, GPIOE, GPIO_PIN_9,  GPIOE, GPIO_PIN_0, GPIOE, GPIO_PIN_1},
  {TIM_CHANNEL_2, GPIOE, GPIO_PIN_11, GPIOE, GPIO_PIN_2, GPIOE, GPIO_PIN_3},
  {TIM_CHANNEL_3, GPIOE, GPIO_PIN_13, GPIOE, GPIO_PIN_4, GPIOE, GPIO_PIN_5},
  {TIM_CHANNEL_4, GPIOE, GPIO_PIN_14, GPIOE, GPIO_PIN_6, GPIOE, GPIO_PIN_7}
};

/* s_enabled[i] = 1 表示第 i 路电机被启用, 0 表示禁用 (SetDuty 会直接 return) */
static uint8_t s_enabled[MOTOR_CHANNEL_COUNT];

/* --------------------------------------------------------------------------
 * 【函数】SetDirection —— 根据占空比正负设置 IN1/IN2
 * 【参数】hw - 电机硬件配置指针; duty - 占空比 (决定方向)
 * ------------------------------------------------------------------------- */
static void SetDirection(const MotorHw_t *hw, int16_t duty)
{
  if (duty > 0)
  {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
  }
  else if (duty < 0)
  {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, GPIO_PIN_RESET);
  }
}

/* --------------------------------------------------------------------------
 * 【函数】MotorAll_Init
 * 【作用】四路电机统一初始化 —— 先全关, 再按 modes[] 逐路启用
 * 【参数】modes - 长度为 MOTOR_CHANNEL_COUNT 的数组, 非零表示启用该通道
 * 【调用时机】main.c 的 MX_TIM1_Init + GPIO_Init 之后
 * 【关键步骤】
 *   1. 写 TIM1 寄存器把四路 PWM 全部清零 (CCER=0, MOE=0, CEN=0)
 *   2. 把 PE9/11/13/14 暂时切回普通 GPIO 低电平 (让 EN 不会悬空)
 *   3. 把 PE0~PE7 全部拉低 (IN1/IN2 归零, 方向中立)
 *   4. 遍历 modes[], 对启用的通道恢复 AF_PP + HAL_TIM_PWM_Start
 * ------------------------------------------------------------------------- */
void MotorAll_Init(const uint8_t modes[MOTOR_CHANNEL_COUNT])
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t i;

  /*
   * MX_TIM1_Init已经完成Full Remap。这里先把四路EN全部变成普通低电平，
   * 再只把启用通道恢复为复用推挽，避免OFF通道悬空。
   */
  TIM1->CCER = 0U;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  TIM1->BDTR &= ~(uint32_t)TIM_BDTR_MOE;
  TIM1->CR1 &= ~(uint32_t)TIM_CR1_CEN;

  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &gpio);
  HAL_GPIO_WritePin(GPIOE, gpio.Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOE,
                    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                    GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                    GPIO_PIN_RESET);

  for (i = 0U; i < MOTOR_CHANNEL_COUNT; i++)
  {
    s_enabled[i] = (modes[i] != 0U) ? 1U : 0U;
    if (s_enabled[i] != 0U)
    {
      gpio.Pin = s_hw[i].pwm_pin;
      gpio.Mode = GPIO_MODE_AF_PP;
      gpio.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(s_hw[i].pwm_port, &gpio);
      HAL_TIM_PWM_Start(&htim1, s_hw[i].channel);
    }
  }
}

/* --------------------------------------------------------------------------
 * 【函数】MotorAll_SetDuty —— 通用占空比设置 (数据驱动, 查表定位硬件)
 * 【参数】id - MotorId_t (0=FL, 1=FR, 2=PE13, 3=PE14)
 *         duty - 正数正转, 负数反转, 0 刹停, 钳位 ±FR_PWM_MAX
 * 【返回】无; 如果 id 非法或通道未启用, 安全 return 不做事
 * ------------------------------------------------------------------------- */
void MotorAll_SetDuty(MotorId_t id, int16_t duty)
{
  uint16_t pulse;

  if ((id >= MOTOR_CHANNEL_COUNT) || (s_enabled[id] == 0U))
  {
    return;
  }

  if (duty > (int16_t)FR_PWM_MAX)
  {
    duty = (int16_t)FR_PWM_MAX;
  }
  else if (duty < -(int16_t)FR_PWM_MAX)
  {
    duty = -(int16_t)FR_PWM_MAX;
  }

  SetDirection(&s_hw[id], duty);
  pulse = (duty >= 0) ? (uint16_t)duty : (uint16_t)(-duty);
  __HAL_TIM_SET_COMPARE(&htim1, s_hw[id].channel, pulse);
}

/* 单通道刹停 —— 等价于 SetDuty(id, 0) */
void MotorAll_Stop(MotorId_t id)
{
  MotorAll_SetDuty(id, 0);
}

/* 把所有启用的通道全部刹停 —— 紧急停止用 */
void MotorAll_StopAll(void)
{
  uint8_t i;

  for (i = 0U; i < MOTOR_CHANNEL_COUNT; i++)
  {
    MotorAll_Stop((MotorId_t)i);
  }
}
