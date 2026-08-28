/*
 * 【文件说明】 encoder_fr.c —— 前右轮编码器采集
 *
 * 【职责】和 encoder_fl.c 完全对称, 只是硬件绑定换成前右轮的 TIM 和 SIGN。
 *        同样维护 s_delta / s_total / s_rpm / s_mm_s 四个状态量。
 *
 * 【算法关键 —— 无符号计数回绕处理】
 *   TIM 编码器计数器是 uint16_t (0~65535 循环)。用 (int16_t)(cnt - s_last_cnt)
 *   可以自动吸收 16-bit 回绕 —— 只要单次增量不超过 ±32767。
 *   280 rpm 时 10ms 只有约 44 个计数, 余量非常充足。
 */

#include "encoder_fr.h"
#include "board_fr.h"

/* 前右轮编码器的内部状态 —— 同 encoder_fl.c 的五个 static 变量 */
static uint16_t s_last_cnt;
static int16_t  s_delta;
static int32_t  s_total;
static float    s_rpm;
static float    s_mm_s;

/* --------------------------------------------------------------------------
 * 【函数】EncoderFr_Init —— 启动编码器 TIM + 清零状态
 * ------------------------------------------------------------------------- */
void EncoderFr_Init(void)
{
  HAL_TIM_Encoder_Start(FR_ENC_TIM, TIM_CHANNEL_ALL);
  EncoderFr_Reset();
}

/* --------------------------------------------------------------------------
 * 【函数】EncoderFr_Update —— 采样 + 换算 (同前左轮算法)
 * 【参数】dt_s - 采样周期 (秒)
 * ------------------------------------------------------------------------- */
void EncoderFr_Update(float dt_s)
{
  uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(FR_ENC_TIM);
  float rev;

  /* 无符号相减再转 int16，16 位回绕会被自然吸收，前提是单周期增量不超过 ±32767。
     280 rpm 时 10 ms 只有约 44 个计数，余量足够 */
  s_delta = (int16_t)(cnt - s_last_cnt) * FR_ENC_SIGN;
  s_last_cnt = cnt;
  s_total += (int32_t)s_delta;

  if (dt_s <= 0.0f)
  {
    return;
  }

  rev = (float)s_delta / FR_ENC_COUNTS_PER_REV;
  s_rpm  = rev / dt_s * 60.0f;
  s_mm_s = rev / dt_s * FR_WHEEL_CIRC_MM;
}

/* 最近一次增量 (计数) */
int16_t EncoderFr_GetDelta(void)
{
  return s_delta;
}

/* 累计里程 (计数) */
int32_t EncoderFr_GetTotal(void)
{
  return s_total;
}

/* 转速 (rpm) */
float EncoderFr_GetRpm(void)
{
  return s_rpm;
}

/* 线速度 (mm/s) */
float EncoderFr_GetSpeedMmS(void)
{
  return s_mm_s;
}

/* 全部清零 —— TIM 计数器 + 所有内部变量 */
void EncoderFr_Reset(void)
{
  __HAL_TIM_SET_COUNTER(FR_ENC_TIM, 0);
  s_last_cnt = 0U;
  s_delta = 0;
  s_total = 0;
  s_rpm = 0.0f;
  s_mm_s = 0.0f;
}
