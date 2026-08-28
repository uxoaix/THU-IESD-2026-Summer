/*
 * 【文件说明】 wheel_speed_control.c —— 四路电机 PID 转速闭环
 *
 * 【职责】整个小车的 "运动底座"。上层给目标速度 (mm/s), 本模块算 PWM duty
 *        → 直接调四个电机。内部对每路电机独立维护一个 SpeedController_t,
 *        内含 PID 状态 + 滤波 + Boost + 保护。
 *
 * 【三态运行状态机 (每轮电机独立)】
 *   SPEED_RUN_IDLE  —— 目标为 0 / 电机禁用 / 有 fault → duty=0, 积分清零
 *   SPEED_RUN_BOOST —— 刚启动或刚变向 → 固定大 duty (START_BOOST_DUTY) 推一把
 *   SPEED_RUN_PID   —— 稳态运行 → 真正跑 PID (ff + P + I + D, + 积分抗饱和)
 *
 * 【类比理解】
 *   - 刚启动像推秋千: 先用力推一把 (Boost), 然后靠自然摆动 (PID)
 *   - 目标速度的正负 → 决定 active_direction (+1/-1)
 *   - 速度滤波: speed_filtered += α × (raw - filtered), α=WHEEL_SPEED_FILTER_ALPHA
 *
 * 【保护逻辑】
 *   1. 编码器反向保护: speed_filtered < -WHEEL_REVERSE_LIMIT_MM_S 持续 N tick
 *      → fault = WHEEL_SPEED_FAULT_ENCODER_REVERSED (电机装反或接线错)
 *   2. 无反馈保护: duty 很大但 speed 接近 0 持续 N tick
 *      → fault = WHEEL_SPEED_FAULT_NO_FEEDBACK (编码器断了或电机堵转)
 *   3. fault 发生: duty=0, 直到上层 ResetFault
 *
 * 【实车轮位映射】
 *   左前=PE13，左后=PE14，右前=PE9，右后=PE11。
 *   编码器和电机均按该物理轮位绑定到同一个 WheelId_t。
 *   这个映射在函数内部硬编码, 改硬件时只动 switch-case 里的几行。
 *
 * 【参数来源】motion_config.h §22 统一管理, 本文件直接 WHEEL_* 宏引用。
 */

#include "wheel_speed_control.h"
#include "motion_config.h"   /* §22 电机 PID 参数集中于此 (WHEEL_PID_* / WHEEL_NO_FB_* 等) */
#include "board_fr.h"
#include "encoder_fl.h"
#include "encoder_fr.h"
#include "encoder_pe13.h"
#include "encoder_pe14.h"
#include "motor_fl.h"
#include "motor_fr.h"
#include "motor_pe13.h"
#include "motor_pe14.h"

/*
 * PID 增益 / 滤波系数 / 保护阈值 / 启动加速等参数原散落于此, 现已集中到
 * motion_config.h §22, 本文件直接引用 WHEEL_* 宏, 避免重复定义。
 * 唯一保留的派生宏 START_BOOST_DUTY 依赖 board_fr.h 的 FR_PWM_MAX (硬件相关,
 * 不宜放进纯算法的 motion_config.h), 用 §22 的 WHEEL_START_BOOST_DUTY_PCT 算出。
 */
#define START_BOOST_DUTY  ((int16_t)((int32_t)FR_PWM_MAX * WHEEL_START_BOOST_DUTY_PCT / 100))

/* SpeedRunState_t —— 每路电机独立的运行状态 (IDLE / BOOST / PID) */
typedef enum {
  SPEED_RUN_IDLE = 0,     /* 空闲: 目标 0 / 禁用 / fault → duty=0 */
  SPEED_RUN_BOOST,        /* 启动 Boost: 固定大 duty 推一把 */
  SPEED_RUN_PID           /* 稳态 PID 闭环 */
} SpeedRunState_t;

/* SpeedController_t —— 每路电机的完整 PID 控制器状态
 * 包含: 目标 + 速度原始值 + 滤波值 + PID 各项 + 保护状态 + 运行状态 */
typedef struct {
  float target_mm_s;          /* 目标线速度 (mm/s), 可正可负 */
  float raw_speed_mm_s;       /* 编码器原始速度 (mm/s), 每个 Run 更新 */
  float speed_filtered;      /* 速度低通滤波后的值 (用于 PID 输入) */
  float previous_speed;       /* 上一周期的 speed_filtered (算 D 项用) */
  float integral;             /* I 项累积值 (带抗饱和限幅) */
  float d_filtered;           /* D 项低通滤波后的值 */
  float ff_term;              /* 前馈项 (目前是常数 WHEEL_PID_FEEDFORWARD_DUTY) */
  float p_term;               /* P 项 = KP × error */
  float i_term;               /* I 项 = integral */
  float d_term;               /* D 项 = -KD × d_filtered */
  int16_t duty_magnitude;     /* duty 绝对值 (0 ~ FR_PWM_MAX), 不含方向 */
  int16_t signed_duty;        /* 带符号的 duty = direction × duty_magnitude, 直接下发给电机 */
  int8_t active_direction;    /* 当前方向 +1 / -1 / 0 (IDLE 时) */
  uint16_t boost_elapsed_ms;  /* Boost 已持续的时间 (ms), 到 WHEEL_START_BOOST_MS 后切 PID */
  uint16_t reverse_ticks;     /* 反向超速累计 tick (保护用) */
  uint16_t no_feedback_grace; /* 无反馈保护的 grace period tick (上电初期不触发) */
  uint16_t no_feedback_ticks; /* 无反馈累计 tick (保护用) */
  WheelSpeedFault_t fault;    /* 当前 fault 状态 */
  SpeedRunState_t run_state;  /* 运行状态: IDLE / BOOST / PID */
} SpeedController_t;

/* 四路电机的 PID 控制器实例 (索引 = WheelId_t) */
static SpeedController_t s_controller[WHEEL_COUNT];

/* 工具函数 —— 把 value 钳位在 [min, max] 之间, 超出则返回边界值 */
static float Clamp(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

/* 工具函数 —— 求 float 的绝对值 */
static float AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

/* --------------------------------------------------------------------------
 * 【函数】ResetPidRuntime —— 切换方向 / 切 Boost→PID 时重置 PID 状态
 * 【做的事】积分清零 + D 项清零 + duty 清零, 避免方向切换瞬间的积分饱和
 * 【为什么】方向变了速度会从正变负, 但积分项还是上一个方向累积的值,
 *         如果不清零会突然产生一个巨大的反向 duty → 电机猛抖
 * ------------------------------------------------------------------------- */
static void ResetPidRuntime(SpeedController_t *ctrl,
                            float directional_speed)
{
  ctrl->speed_filtered = directional_speed;
  ctrl->previous_speed = directional_speed;
  ctrl->integral = 0.0f;
  ctrl->d_filtered = 0.0f;
  ctrl->ff_term = 0.0f;
  ctrl->p_term = 0.0f;
  ctrl->i_term = 0.0f;
  ctrl->d_term = 0.0f;
  ctrl->duty_magnitude = 0;
  ctrl->signed_duty = 0;
  ctrl->reverse_ticks = 0U;
  ctrl->no_feedback_grace = 0U;
  ctrl->no_feedback_ticks = 0U;
}

/* --------------------------------------------------------------------------
 * 【函数】UpdateFilteredSpeed —— 速度一阶低通滤波
 * 【公式】speed_filtered += α × (raw - speed_filtered), α = WHEEL_SPEED_FILTER_ALPHA
 * 【作用】编码器原始速度有噪声 (每 10ms 采样增量小), 滤波后 PID 更稳
 * ------------------------------------------------------------------------- */
static void UpdateFilteredSpeed(SpeedController_t *ctrl, float raw)
{
  ctrl->speed_filtered += WHEEL_SPEED_FILTER_ALPHA *
                          (raw - ctrl->speed_filtered);
}

/* --------------------------------------------------------------------------
 * 【函数】CheckProtection —— 反向保护 + 无反馈保护
 * 【保护 1 —— 编码器反向】
 *   speed_filtered < -WHEEL_REVERSE_LIMIT_MM_S 持续 WHEEL_REVERSE_LIMIT_TICKS
 *   → fault = WHEEL_SPEED_FAULT_ENCODER_REVERSED (电机装反 / 接线错)
 * 【保护 2 —— 无反馈】
 *   上电前 WHEEL_NO_FB_GRACE_TICKS tick 不检查 (电机还没转起来)
 *   之后 duty > WHEEL_NO_FB_DUTY 但 |speed| < WHEEL_NO_FB_SPEED_MM_S
 *   持续 WHEEL_NO_FB_LIMIT_TICKS → fault = WHEEL_SPEED_FAULT_NO_FEEDBACK
 *   (编码器断线 / 电机堵转)
 * ------------------------------------------------------------------------- */
static void CheckProtection(SpeedController_t *ctrl)
{
  if (ctrl->speed_filtered < -WHEEL_REVERSE_LIMIT_MM_S)
  {
    if (ctrl->reverse_ticks < WHEEL_REVERSE_LIMIT_TICKS)
    {
      ctrl->reverse_ticks++;
    }
    if (ctrl->reverse_ticks >= WHEEL_REVERSE_LIMIT_TICKS)
    {
      ctrl->fault = WHEEL_SPEED_FAULT_ENCODER_REVERSED;
    }
  }
  else
  {
    ctrl->reverse_ticks = 0U;
  }

  if (ctrl->no_feedback_grace < WHEEL_NO_FB_GRACE_TICKS)
  {
    ctrl->no_feedback_grace++;
    ctrl->no_feedback_ticks = 0U;
    return;
  }

  if ((ctrl->duty_magnitude > WHEEL_NO_FB_DUTY) &&
      (AbsFloat(ctrl->speed_filtered) < WHEEL_NO_FB_SPEED_MM_S))
  {
    if (ctrl->no_feedback_ticks < WHEEL_NO_FB_LIMIT_TICKS)
    {
      ctrl->no_feedback_ticks++;
    }
    if (ctrl->no_feedback_ticks >= WHEEL_NO_FB_LIMIT_TICKS)
    {
      ctrl->fault = WHEEL_SPEED_FAULT_NO_FEEDBACK;
    }
  }
  else
  {
    ctrl->no_feedback_ticks = 0U;
  }
}

/* --------------------------------------------------------------------------
 * 【函数】PidUpdate —— 真正的 PID 闭环计算 (Boost 阶段跳过, 只在 SPEED_RUN_PID 里调)
 * 【PID 公式】
 *   error = target - speed_filtered          (目标减实际)
 *   ff    = WHEEL_PID_FEEDFORWARD_DUTY      (前馈常量, 加速启动)
 *   p     = KP × error                       (比例项, 越快越大)
 *   d_raw = d/dt speed                       (微分项, 速度变化率)
 *   d     = -KD × LP(d_raw)                  (D 项滤波后, 反号因为 error=target-speed)
 *   i_cand = integral + KI × error × dt     (积分候选值)
 *   i_cand = Clamp(i_cand, I_MIN, I_MAX)     (积分限幅)
 *   out_raw = ff + p + i_cand + d            (四项加起来)
 *   output = Clamp(out_raw, 0, OUTPUT_MAX)   (输出限幅到正半轴)
 * 【积分抗饱和】只有在 output 没饱和时更新 integral, 或饱和方向与误差方向一致时才更新
 * ------------------------------------------------------------------------- */
static int16_t PidUpdate(SpeedController_t *ctrl,
                         float target_magnitude_mm_s,
                         float dt_s)
{
  float error;
  float derivative;
  float integral_candidate;
  float unsaturated;
  float output;

  CheckProtection(ctrl);
  if (ctrl->fault != WHEEL_SPEED_FAULT_NONE)
  {
    ctrl->duty_magnitude = 0;
    return 0;
  }

  error = target_magnitude_mm_s - ctrl->speed_filtered;
  ctrl->ff_term = WHEEL_PID_FEEDFORWARD_DUTY;
  ctrl->p_term = WHEEL_PID_KP * error;

  derivative = (ctrl->speed_filtered - ctrl->previous_speed) /
               dt_s;
  ctrl->previous_speed = ctrl->speed_filtered;
  ctrl->d_filtered += WHEEL_D_FILTER_ALPHA *
                      (derivative - ctrl->d_filtered);
  ctrl->d_term = -WHEEL_PID_KD * ctrl->d_filtered;

  integral_candidate = ctrl->integral +
                       WHEEL_PID_KI * error * dt_s;
  integral_candidate = Clamp(integral_candidate,
                             WHEEL_PID_I_MIN,
                             WHEEL_PID_I_MAX);

  unsaturated = ctrl->ff_term + ctrl->p_term +
                integral_candidate + ctrl->d_term;
  output = Clamp(unsaturated, 0.0f, WHEEL_PID_OUTPUT_MAX);

  if ((unsaturated == output) ||
      ((output >= WHEEL_PID_OUTPUT_MAX) && (error < 0.0f)) ||
      ((output <= 0.0f) && (error > 0.0f)))
  {
    ctrl->integral = integral_candidate;
  }
  ctrl->i_term = ctrl->integral;
  ctrl->duty_magnitude = (int16_t)output;
  return ctrl->duty_magnitude;
}

/* --------------------------------------------------------------------------
 * 【函数】ReadEncoderSpeed —— 查表读取指定轮子的编码器速度 (mm/s)
 * 【注意】WheelId_t 与实车轮位的映射集中在本 switch-case，
 *        并与下面 SetMotorDuty 保持严格对称。
 * ------------------------------------------------------------------------- */
static float ReadEncoderSpeed(WheelId_t wheel)
{
  switch (wheel)
  {
    case WHEEL_FRONT_LEFT:
      return EncoderPe13_GetSpeedMmS();     /* 实车左前：PE13 */
    case WHEEL_FRONT_RIGHT:
      return EncoderFr_GetSpeedMmS();       /* 实车右前：PE9 */
    case WHEEL_REAR_LEFT:
      return EncoderPe14_GetSpeedMmS();     /* 实车左后：PE14 */
    case WHEEL_REAR_RIGHT:
      return EncoderFl_GetSpeedMmS();       /* 实车右后：PE11 */
    default:
      return 0.0f;
  }
}

/* --------------------------------------------------------------------------
 * 【函数】SetMotorDuty —— 查表下发 duty 给指定轮子的电机
 * 【硬件方向修正】每个电机有个 DRIVE_SIGN (FR_DRIVE_SIGN / FL_DRIVE_SIGN 等),
 *        如果电机装反了就把这个宏定义成 -1, 在这里乘一下就正过来了
 *        signed_duty × DRIVE_SIGN → 传给 Motor*_SetDuty
 * ------------------------------------------------------------------------- */
static void SetMotorDuty(WheelId_t wheel, int16_t signed_duty)
{
  switch (wheel)
  {
    case WHEEL_FRONT_LEFT:
      MotorPe13_SetDuty((int16_t)(PE13_DRIVE_SIGN * signed_duty));
      break;
    case WHEEL_FRONT_RIGHT:
      MotorFr_SetDuty((int16_t)(FR_DRIVE_SIGN * signed_duty));
      break;
    case WHEEL_REAR_LEFT:
      MotorPe14_SetDuty((int16_t)(PE14_DRIVE_SIGN * signed_duty));
      break;
    case WHEEL_REAR_RIGHT:
      MotorFl_SetDuty((int16_t)(FL_DRIVE_SIGN * signed_duty));
      break;
    default:
      break;
  }
}

/* --------------------------------------------------------------------------
 * 【函数】WheelSpeedControl_Init —— 四路电机 + 四路编码器 + 控制器状态全部初始化
 * 【顺序】先电机 (Init + Stop), 再编码器 (Init), 最后把 s_controller[4] 清零
 * 【为什么先 Stop】电机 Init 后 PWM 已经启动了, 立刻 Stop 让 duty=0, 安全
 * ------------------------------------------------------------------------- */
void WheelSpeedControl_Init(void)
{
  uint8_t i;

  MotorFr_Init();
  MotorFl_Init();
  MotorPe13_Init();
  MotorPe14_Init();
  MotorFr_Stop();
  MotorFl_Stop();
  MotorPe13_Stop();
  MotorPe14_Stop();

  EncoderFr_Init();
  EncoderFl_Init();
  EncoderPe13_Init();
  EncoderPe14_Init();

  for (i = 0U; i < WHEEL_COUNT; i++)
  {
    SpeedController_t *ctrl = &s_controller[i];
    ctrl->target_mm_s = 0.0f;
    ctrl->raw_speed_mm_s = 0.0f;
    ctrl->active_direction = 0;
    ctrl->boost_elapsed_ms = 0U;
    ctrl->fault = WHEEL_SPEED_FAULT_NONE;
    ctrl->run_state = SPEED_RUN_IDLE;
    ResetPidRuntime(ctrl, 0.0f);
  }
}

/* 设置某一路的目标速度 (mm/s, 带正负), 直接写 s_controller[wheel].target_mm_s */
void WheelSpeedControl_SetTargetMmS(WheelId_t wheel,
                                    float signed_target_mm_s)
{
  if ((uint8_t)wheel < WHEEL_COUNT)
  {
    s_controller[wheel].target_mm_s = signed_target_mm_s;
  }
}

/* --------------------------------------------------------------------------
 * 【函数】WheelSpeedControl_Run —— 电机 PID 主循环, 每 10ms 调一次
 * 【做的事】
 *   1. 更新四路编码器 (Encoder*_Update)
 *   2. 对每路电机:
 *      a. 读编码器速度 → raw_speed
 *      b. enabled=0 / target≈0 / fault → IDLE, duty=0
 *      c. 目标方向变了 → ResetPidRuntime → 切 Boost
 *      d. 速度滤波 + Boost 或 PID 计算
 *      e. duty × direction → signed_duty → SetMotorDuty
 * 【三态状态机转换】IDLE → BOOST (方向变 / 启动) → PID (Boost 时间到)
 * ------------------------------------------------------------------------- */
void WheelSpeedControl_Run(float dt_s, uint8_t enabled)
{
  uint16_t elapsed_ms;
  uint8_t i;

  if (dt_s <= 0.0f)
  {
    return;
  }

  EncoderFr_Update(dt_s);
  EncoderFl_Update(dt_s);
  EncoderPe13_Update(dt_s);
  EncoderPe14_Update(dt_s);

  elapsed_ms = (uint16_t)(dt_s * 1000.0f + 0.5f);
  if (elapsed_ms == 0U)
  {
    elapsed_ms = 1U;
  }

  for (i = 0U; i < WHEEL_COUNT; i++)
  {
    SpeedController_t *ctrl = &s_controller[i];
    int8_t requested_direction;
    float directional_speed;

    ctrl->raw_speed_mm_s = ReadEncoderSpeed((WheelId_t)i);

    if ((enabled == 0U) ||
        (AbsFloat(ctrl->target_mm_s) < 1.0f) ||
        (ctrl->fault != WHEEL_SPEED_FAULT_NONE))
    {
      ctrl->run_state = SPEED_RUN_IDLE;
      ctrl->active_direction = 0;
      ctrl->boost_elapsed_ms = 0U;
      ctrl->duty_magnitude = 0;
      ctrl->signed_duty = 0;
      ctrl->integral = 0.0f;
      SetMotorDuty((WheelId_t)i, 0);
      continue;
    }

    requested_direction =
      (ctrl->target_mm_s >= 0.0f) ? 1 : -1;
    directional_speed = ctrl->raw_speed_mm_s *
                        (float)requested_direction;

    if ((ctrl->run_state == SPEED_RUN_IDLE) ||
        (ctrl->active_direction != requested_direction))
    {
      ctrl->active_direction = requested_direction;
      ctrl->boost_elapsed_ms = 0U;
      ctrl->run_state = SPEED_RUN_BOOST;
      ResetPidRuntime(ctrl, directional_speed);
    }

    UpdateFilteredSpeed(ctrl, directional_speed);

    if (ctrl->run_state == SPEED_RUN_BOOST)
    {
      ctrl->duty_magnitude = START_BOOST_DUTY;
      ctrl->boost_elapsed_ms =
        (uint16_t)(ctrl->boost_elapsed_ms + elapsed_ms);
      if (ctrl->boost_elapsed_ms >= WHEEL_START_BOOST_MS)
      {
        ctrl->run_state = SPEED_RUN_PID;
        ctrl->previous_speed = ctrl->speed_filtered;
        ctrl->integral = 0.0f;
      }
    }
    else
    {
      ctrl->duty_magnitude = PidUpdate(
        ctrl,
        AbsFloat(ctrl->target_mm_s),
        dt_s);
    }

    ctrl->signed_duty =
      (int16_t)(ctrl->active_direction *
                ctrl->duty_magnitude);
    SetMotorDuty((WheelId_t)i, ctrl->signed_duty);
  }
}

/* 紧急停止 —— 把所有轮 target=0, duty=0, run_state=IDLE, 电机停转 */
void WheelSpeedControl_StopAll(void)
{
  uint8_t i;

  for (i = 0U; i < WHEEL_COUNT; i++)
  {
    s_controller[i].target_mm_s = 0.0f;
    s_controller[i].signed_duty = 0;
    s_controller[i].duty_magnitude = 0;
    s_controller[i].run_state = SPEED_RUN_IDLE;
    SetMotorDuty((WheelId_t)i, 0);
  }
}

/* 清除某一路的 fault 状态, 恢复 IDLE, 重新等上层下发 target  */
void WheelSpeedControl_ResetFault(WheelId_t wheel)
{
  if ((uint8_t)wheel < WHEEL_COUNT)
  {
    SpeedController_t *ctrl = &s_controller[wheel];
    ctrl->fault = WHEEL_SPEED_FAULT_NONE;
    ctrl->run_state = SPEED_RUN_IDLE;
    ctrl->active_direction = 0;
    ResetPidRuntime(ctrl, ctrl->raw_speed_mm_s);
  }
}

/* 只读获取某轮的目标速度 (mm/s), wheel 非法返回 0 */
float WheelSpeedControl_GetTargetMmS(WheelId_t wheel)
{
  return ((uint8_t)wheel < WHEEL_COUNT)
       ? s_controller[wheel].target_mm_s : 0.0f;
}

/* 只读获取某轮的编码器原始速度 (mm/s, 未经滤波) */
float WheelSpeedControl_GetSpeedMmS(WheelId_t wheel)
{
  return ((uint8_t)wheel < WHEEL_COUNT)
       ? s_controller[wheel].raw_speed_mm_s : 0.0f;
}

/* 只读获取某轮当前下发的带符号 duty (正=正转, 负=反转) */
int16_t WheelSpeedControl_GetDuty(WheelId_t wheel)
{
  return ((uint8_t)wheel < WHEEL_COUNT)
       ? s_controller[wheel].signed_duty : 0;
}

/* 只读获取某轮的 fault 状态 (NONE / REVERSED / NO_FEEDBACK) */
WheelSpeedFault_t WheelSpeedControl_GetFault(WheelId_t wheel)
{
  return ((uint8_t)wheel < WHEEL_COUNT)
       ? s_controller[wheel].fault
       : WHEEL_SPEED_FAULT_NO_FEEDBACK;
}
