/*
 * 【文件说明】 encoder_pe13.c —— 后左轮编码器采集 (PE13 电机)
 *
 * 【职责】每 ~10ms 被上层调用一次 Update(), 读取 TIM 编码器计数器,
 *        算出两次采样间的增量 (delta), 再换算成线速度 (mm/s)。
 *
 * 【类比理解】
 *   - TIM 编码器计数器像汽车里程表, 每次 Update 相当于拍照
 *   - delta = "本次读数 - 上次读数", 表示这段时间轮子转了多少格
 *   - speed = 转的圈数 × 轮周长 / 时间, 就像算汽车速度
 *
 * 【硬件绑定】
 *   编码器 TIM → PE13_ENC_TIM (具体是 TIM2/5/8 哪一个由 board_fr.h 决定)
 *   方向修正 → PE13_ENC_SIGN (+1 或 -1, 电机装反时用 -1)
 *   换算常量 → FR_ENC_COUNTS_PER_REV (每转计数), FR_WHEEL_CIRC_MM (轮周长mm)
 *
 * 【注意事项】
 *   - s_last_count 和 s_speed_mm_s 是 static 局部变量, 只在本文件可见
 *   - EncoderPe13_Update(dt_s) 的 dt_s 传 0 会跳过速度计算 (安全保护)
 */

#include "encoder_pe13.h"
#include "board_fr.h"

/* s_last_count: 上次采样时的 TIM 计数器值, 用来算差值
 * s_speed_mm_s: 最近一次换算出来的线速度 (单位 mm/s) */
static uint16_t s_last_count;
static float s_speed_mm_s;

/* --------------------------------------------------------------------------
 * 【函数】EncoderPe13_Init
 * 【作用】启动 TIM 编码器接口 + 清零内部状态
 * 【参数】无
 * 【返回】无
 * 【调用时机】MotorPe13_Init 之后, 主循环 while(1) 之前
 * ------------------------------------------------------------------------- */
void EncoderPe13_Init(void)
{
  HAL_TIM_Encoder_Start(PE13_ENC_TIM, TIM_CHANNEL_ALL);
  EncoderPe13_Reset();
}

/* --------------------------------------------------------------------------
 * 【函数】EncoderPe13_Update
 * 【作用】采样编码器计数器 + 换算线速度
 * 【参数】dt_s - 两次 Update 之间的时间间隔 (秒), 通常 0.01 (10ms)
 * 【返回】无 (结果存 s_speed_mm_s, 通过 GetSpeedMmS 读取)
 * 【核心算法】
 *   delta = (当前计数 - 上次计数) × PE13_ENC_SIGN
 *   转数  = delta / FR_ENC_COUNTS_PER_REV
 *   速度  = 转数 / dt_s × FR_WHEEL_CIRC_MM
 * ------------------------------------------------------------------------- */
void EncoderPe13_Update(float dt_s)
{
  uint16_t count = (uint16_t)__HAL_TIM_GET_COUNTER(PE13_ENC_TIM);
  int16_t delta = (int16_t)(count - s_last_count) * PE13_ENC_SIGN;

  s_last_count = count;
  if (dt_s > 0.0f)
  {
    float revolutions = (float)delta / FR_ENC_COUNTS_PER_REV;
    s_speed_mm_s = revolutions / dt_s * FR_WHEEL_CIRC_MM;
  }
}

/* 只读获取最近一次计算出来的线速度, 单位 mm/s */
float EncoderPe13_GetSpeedMmS(void)
{
  return s_speed_mm_s;
}

/* --------------------------------------------------------------------------
 * 【函数】EncoderPe13_Reset
 * 【作用】把 TIM 计数器和内部变量全部清零 —— 上电或切模式时调用
 * 【参数】无
 * 【返回】无
 * ------------------------------------------------------------------------- */
void EncoderPe13_Reset(void)
{
  __HAL_TIM_SET_COUNTER(PE13_ENC_TIM, 0);
  s_last_count = 0U;
  s_speed_mm_s = 0.0f;
}
