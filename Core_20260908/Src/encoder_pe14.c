/*
 * 【文件说明】 encoder_pe14.c —— 后右轮编码器采集 (PE14 电机)
 *
 * 【职责】和 encoder_pe13.c 完全对称, 只是硬件绑定不同:
 *        PE14_ENC_TIM 替代 PE13_ENC_TIM, PE14_ENC_SIGN 替代 PE13_ENC_SIGN。
 *        算法一模一样: 采样 → 算增量 → 换算线速度 (mm/s)。
 *
 * 【类比理解】两个后轮就像人的两只脚, 每个脚有独立的 "步数计数器"。
 *
 * 【硬件绑定】
 *   编码器 TIM → PE14_ENC_TIM (在 board_fr.h 定义)
 *   方向修正 → PE14_ENC_SIGN (+1 或 -1, 电机装反时用 -1)
 *   换算常量 → FR_ENC_COUNTS_PER_REV, FR_WHEEL_CIRC_MM
 *
 * 【注意事项】PE14_ENC_SIGN 如果是 -1, 说明这个后轮的编码器方向和前轮相反,
 *           软件里乘一下 -1 就正过来了, 不用改硬件。
 */

#include "encoder_pe14.h"
#include "board_fr.h"

/* s_last_count: 上次采样的 TIM 计数器值; s_speed_mm_s: 换算好的线速度 */
static uint16_t s_last_count;
static float s_speed_mm_s;

/* --------------------------------------------------------------------------
 * 【函数】EncoderPe14_Init —— 启动 TIM 编码器 + 清零内部状态
 * ------------------------------------------------------------------------- */
void EncoderPe14_Init(void)
{
  HAL_TIM_Encoder_Start(PE14_ENC_TIM, TIM_CHANNEL_ALL);
  EncoderPe14_Reset();
}

/* --------------------------------------------------------------------------
 * 【函数】EncoderPe14_Update —— 采样计数器 + 换算线速度 (算法同 PE13)
 * 【参数】dt_s - 采样周期 (秒), 传 0 会跳过计算
 * ------------------------------------------------------------------------- */
void EncoderPe14_Update(float dt_s)
{
  uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(PE14_ENC_TIM);
  int16_t delta = (int16_t)(count - s_last_count) * PE14_ENC_SIGN;

  s_last_count = count;
  if (dt_s > 0.0f)
  {
    float revolutions = (float)delta / FR_ENC_COUNTS_PER_REV;
    s_speed_mm_s = revolutions / dt_s * FR_WHEEL_CIRC_MM;
  }
}

/* 只读获取后右轮最近一次的线速度 (mm/s) */
float EncoderPe14_GetSpeedMmS(void)
{
  return s_speed_mm_s;
}

/* 清零 TIM 计数器和所有内部状态 */
void EncoderPe14_Reset(void)
{
  __HAL_TIM_SET_COUNTER(PE14_ENC_TIM, 0);
  s_last_count = 0U;
  s_speed_mm_s = 0.0f;
}
