/*
 * 【文件说明】 encoder_fl.c —— 前左轮编码器采集
 *
 * 【职责】每 ~10ms 调用 Update(), 读 TIM 计数器 → 算 delta → 转 rpm 和线速度。
 *        比 PE13/PE14 编码器多维护一个 s_total (累计计数), 方便上位机算总里程。
 *
 * 【类比理解】
 *   - s_delta = 这 10ms 轮子转了多少格 (瞬时)
 *   - s_total = 从上电到现在总共转了多少格 (累计, 像汽车里程表)
 *   - s_rpm / s_mm_s = 转速 / 线速度, 给上层 PID 控制器用
 *
 * 【硬件绑定】编码器的 TIM 实例由 board_fr.h 里 FL_ENC_TIM 决定,
 *        FL_ENC_SIGN (+1/-1) 用于修正电机装反的情况。
 *        FR_ENC_COUNTS_PER_REV 和 FR_WHEEL_CIRC_MM 是换算常量。
 */

#include "encoder_fl.h"
#include "board_fr.h"

/* s_last_cnt: 上次计数; s_delta: 本次增量; s_total: 累计计数 (像里程表)
 * s_rpm: 转速; s_mm_s: 线速度 (mm/s) */
static uint16_t s_last_cnt;
static int16_t  s_delta;
static int32_t  s_total;
static float    s_rpm;
static float    s_mm_s;

/* --------------------------------------------------------------------------
 * 【函数】EncoderFl_Init —— 启动编码器 TIM + 清零所有内部状态
 * ------------------------------------------------------------------------- */
void EncoderFl_Init(void)
{
  HAL_TIM_Encoder_Start(FL_ENC_TIM, TIM_CHANNEL_ALL);
  EncoderFl_Reset();
}

/* --------------------------------------------------------------------------
 * 【函数】EncoderFl_Update
 * 【作用】采样编码器计数器 + 算增量 + 累计里程 + 换算 rpm 和 mm/s
 * 【参数】dt_s - 采样周期 (秒), 通常 0.01
 * 【算法】
 *   delta   = (当前计数 - 上次计数) × FL_ENC_SIGN
 *   total  += delta
 *   rpm     = (delta / COUNTS_PER_REV) / dt_s × 60
 *   mm/s    = (delta / COUNTS_PER_REV) / dt_s × WHEEL_CIRC_MM
 * ------------------------------------------------------------------------- */
void EncoderFl_Update(float dt_s)
{
  uint16_t cnt = (uint16_t)__HAL_TIM_GET_COUNTER(FL_ENC_TIM);
  float rev;

  s_delta = (int16_t)(cnt - s_last_cnt) * FL_ENC_SIGN;
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

/* 只读获取最近一次采样的增量 (计数), 给 PID 做前馈参考 */
int16_t EncoderFl_GetDelta(void)
{
  return s_delta;
}

/* 只读获取累计计数 (从 Reset 开始), 上位机可用来算总里程 */
int32_t EncoderFl_GetTotal(void)
{
  return s_total;
}

/* 只读获取转速 (rpm) */
float EncoderFl_GetRpm(void)
{
  return s_rpm;
}

/* 只读获取线速度 (mm/s), 最常用的速度接口 */
float EncoderFl_GetSpeedMmS(void)
{
  return s_mm_s;
}

/* 清零 TIM 计数器和所有内部状态, 相当于里程表归零 */
void EncoderFl_Reset(void)
{
  __HAL_TIM_SET_COUNTER(FL_ENC_TIM, 0);
  s_last_cnt = 0U;
  s_delta = 0;
  s_total = 0;
  s_rpm = 0.0f;
  s_mm_s = 0.0f;
}
