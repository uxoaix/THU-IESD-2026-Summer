/*
 * 【文件说明】 encoder_all.c —— 四路编码器统一采集 (全局封装)
 *
 * 【职责】和 motor_all.c 对称 —— 把四个独立编码器模块收敛到一个接口层。
 *        上层调 EncoderAll_Update(0.01) 就能一次性更新四路速度。
 *
 * 【数据驱动设计】
 *   s_enc[] 是四个 EncoderState_t 的数组, 每个元素记录:
 *     - .timer        : 绑定的 TIM 实例 (&htim2 / &htim4 / &htim5 / &htim8)
 *     - .last_count   : 上次计数器值
 *     - .delta/total/rpm/speed_mm_s : 五个状态量 (同独立模块)
 *     - .sign         : 方向修正 (+1 或 -1, 初始化时从 signs[] 数组传入)
 *     - .enabled      : 通道是否启用
 *
 * 【初始化流程】
 *   EncoderAll_Init(modes[], signs[]) —— 遍历 0~3:
 *     1. 写 .sign 和 .enabled
 *     2. 清零 TIM 计数器
 *     3. enabled=1 时调用 HAL_TIM_Encoder_Start 启动编码器接口
 */

#include "encoder_all.h"
#include "board_fr.h"

/* EncoderState_t —— 每路编码器的完整内部状态 (比独立模块多了 .timer/.sign/.enabled) */
typedef struct {
  TIM_HandleTypeDef *timer;  /* 绑定的 TIM 实例指针 */
  uint16_t last_count;       /* 上次计数器值 */
  int16_t delta;             /* 本次增量 */
  int32_t total;             /* 累计计数 (里程表) */
  float rpm;                 /* 转速 */
  float speed_mm_s;          /* 线速度 (mm/s) */
  int8_t sign;               /* 方向修正 +1/-1 */
  uint8_t enabled;           /* 通道是否启用 */
} EncoderState_t;

/* 四路编码器的 TIM 绑定表 —— 索引 0~3 对应 MotorId_t, 硬件定时器由 board_fr.h 决定 */
static EncoderState_t s_enc[MOTOR_CHANNEL_COUNT] = {
  {.timer = &htim2},
  {.timer = &htim4},
  {.timer = &htim5},
  {.timer = &htim8}
};

/* --------------------------------------------------------------------------
 * 【函数】EncoderAll_Init
 * 【作用】四路编码器统一初始化 —— 配置 sign/enabled + 清计数器 + 启动接口
 * 【参数】modes - 非零表示启用该通道 (长度 MOTOR_CHANNEL_COUNT)
 *         signs - 方向修正 +1/-1 (长度 MOTOR_CHANNEL_COUNT)
 * 【调用时机】对应电机 MotorAll_Init 之后
 * ------------------------------------------------------------------------- */
void EncoderAll_Init(const uint8_t modes[MOTOR_CHANNEL_COUNT],
                     const int8_t signs[MOTOR_CHANNEL_COUNT])
{
  uint8_t i;

  for (i = 0U; i < MOTOR_CHANNEL_COUNT; i++)
  {
    EncoderState_t *enc = &s_enc[i];

    enc->enabled = (modes[i] != 0U) ? 1U : 0U;
    enc->sign = signs[i];
    enc->last_count = 0U;
    enc->delta = 0;
    enc->total = 0;
    enc->rpm = 0.0f;
    enc->speed_mm_s = 0.0f;
    __HAL_TIM_SET_COUNTER(enc->timer, 0);

    if (enc->enabled != 0U)
    {
      HAL_TIM_Encoder_Start(enc->timer, TIM_CHANNEL_ALL);
    }
  }
}

/* --------------------------------------------------------------------------
 * 【函数】EncoderAll_Update
 * 【作用】统一采样四路编码器 —— 遍历 s_enc[], 对每个 enabled 通道更新
 * 【参数】dt_s - 采样周期 (秒), 传 0 安全返回不做事
 * ------------------------------------------------------------------------- */
void EncoderAll_Update(float dt_s)
{
  uint8_t i;

  if (dt_s <= 0.0f)
  {
    return;
  }

  for (i = 0U; i < MOTOR_CHANNEL_COUNT; i++)
  {
    EncoderState_t *enc = &s_enc[i];
    uint16_t count;
    float revolutions;

    if (enc->enabled == 0U)
    {
      continue;
    }

    count = (uint16_t)__HAL_TIM_GET_COUNTER(enc->timer);
    enc->delta = (int16_t)(count - enc->last_count) * enc->sign;
    enc->last_count = count;
    enc->total += (int32_t)enc->delta;

    revolutions = (float)enc->delta / FR_ENC_COUNTS_PER_REV;
    enc->rpm = revolutions / dt_s * 60.0f;
    enc->speed_mm_s = revolutions / dt_s * FR_WHEEL_CIRC_MM;
  }
}

/* 四路编码器的 Get 系列 —— 安全包装, id 非法或通道禁用时返回 0 */
int16_t EncoderAll_GetDelta(MotorId_t id)
{
  return (id < MOTOR_CHANNEL_COUNT) ? s_enc[id].delta : 0;
}

int32_t EncoderAll_GetTotal(MotorId_t id)
{
  return (id < MOTOR_CHANNEL_COUNT) ? s_enc[id].total : 0;
}

float EncoderAll_GetRpm(MotorId_t id)
{
  return (id < MOTOR_CHANNEL_COUNT) ? s_enc[id].rpm : 0.0f;
}

float EncoderAll_GetSpeedMmS(MotorId_t id)
{
  return (id < MOTOR_CHANNEL_COUNT) ? s_enc[id].speed_mm_s : 0.0f;
}
