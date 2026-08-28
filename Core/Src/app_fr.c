#include "app_fr.h"
#include "board_fr.h"
#include "motor_fr.h"
#include "motor_fl.h"
#include "encoder_fr.h"
#include "encoder_fl.h"
#include "motor_pe13.h"
#include "motor_pe14.h"
#include "encoder_pe13.h"
#include "encoder_pe14.h"
#include "dbg_uart.h"

/*
 * PE9/PE11/PE13/PE14四电机：
 *   延时1 s -> 四侧60%开环助推0.8 s -> 四套PID各自保持300 mm/s
 *   PE9 ：PE0/PE1方向，TIM2编码器 PA15/PB3
 *   PE11：PE2/PE3方向，TIM4编码器 PB6/PB7
 *   PE13：PE4/PE5方向，TIM5编码器 PA0/PA1
 *   PE14：PE6/PE7方向，TIM8编码器 PC6/PC7
 */
#define TARGET_MM_S               300.0f
#define START_DELAY_MS            1000U
#define START_BOOST_MS            800U
#define START_BOOST_DUTY          ((int16_t)((int32_t)FR_PWM_MAX * 60 / 100))
#define LOOP_MS                   10U
#define PRINT_MS                  100U

#define PID_FEEDFORWARD_DUTY      1200.0f
#define PID_KP                    2.0f
#define PID_KI                    8.0f
#define PID_KD                    0.005f
#define SPEED_FILTER_ALPHA        0.25f
#define D_FILTER_ALPHA            0.20f
#define PID_I_MIN                 (-1000.0f)
#define PID_I_MAX                 1800.0f
#define PID_OUTPUT_MAX            3000.0f

/* 放宽保护：反向0.5 s；PID启动3 s后检查；大输出无反馈5 s锁停 */
#define REVERSE_LIMIT_MM_S        50.0f
#define REVERSE_LIMIT_TICKS       50U
#define NO_FB_GRACE_TICKS         300U
#define NO_FB_DUTY                2700
#define NO_FB_SPEED_MM_S          10.0f
#define NO_FB_LIMIT_TICKS         500U

typedef enum {
  RUN_DELAY = 0,
  RUN_BOOST,
  RUN_PID
} RunState_t;

typedef enum {
  FAULT_NONE = 0,
  FAULT_ENCODER_REVERSED = 1,
  FAULT_NO_FEEDBACK = 2
} FaultCode_t;

typedef struct {
  float speed_filtered;
  float previous_speed;
  float integral;
  float d_filtered;
  float ff_term;
  float p_term;
  float i_term;
  float d_term;
  int16_t duty;
  uint16_t reverse_ticks;
  uint16_t no_feedback_grace;
  uint16_t no_feedback_ticks;
  FaultCode_t fault;
} Controller_t;

static Controller_t s_pe9;
static Controller_t s_pe11;
static Controller_t s_pe13;
static Controller_t s_pe14;
static RunState_t s_state;
static uint32_t s_state_tick;
static uint32_t s_loop_tick;
static uint32_t s_print_tick;

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

static float AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static void Controller_Reset(Controller_t *ctrl, float speed)
{
  ctrl->speed_filtered = speed;
  ctrl->previous_speed = speed;
  ctrl->integral = 0.0f;
  ctrl->d_filtered = 0.0f;
  ctrl->ff_term = 0.0f;
  ctrl->p_term = 0.0f;
  ctrl->i_term = 0.0f;
  ctrl->d_term = 0.0f;
  ctrl->duty = 0;
  ctrl->reverse_ticks = 0U;
  ctrl->no_feedback_grace = 0U;
  ctrl->no_feedback_ticks = 0U;
  ctrl->fault = FAULT_NONE;
}

static void UpdateFilteredSpeed(Controller_t *ctrl, float raw)
{
  ctrl->speed_filtered += SPEED_FILTER_ALPHA *
                          (raw - ctrl->speed_filtered);
}

static void CheckProtection(Controller_t *ctrl)
{
  if (ctrl->speed_filtered < -REVERSE_LIMIT_MM_S)
  {
    if (ctrl->reverse_ticks < REVERSE_LIMIT_TICKS)
    {
      ctrl->reverse_ticks++;
    }
    if (ctrl->reverse_ticks >= REVERSE_LIMIT_TICKS)
    {
      ctrl->fault = FAULT_ENCODER_REVERSED;
    }
  }
  else
  {
    ctrl->reverse_ticks = 0U;
  }

  if (ctrl->no_feedback_grace < NO_FB_GRACE_TICKS)
  {
    ctrl->no_feedback_grace++;
    ctrl->no_feedback_ticks = 0U;
    return;
  }

  if ((ctrl->duty > NO_FB_DUTY) &&
      (AbsFloat(ctrl->speed_filtered) < NO_FB_SPEED_MM_S))
  {
    if (ctrl->no_feedback_ticks < NO_FB_LIMIT_TICKS)
    {
      ctrl->no_feedback_ticks++;
    }
    if (ctrl->no_feedback_ticks >= NO_FB_LIMIT_TICKS)
    {
      ctrl->fault = FAULT_NO_FEEDBACK;
    }
  }
  else
  {
    ctrl->no_feedback_ticks = 0U;
  }
}

static int16_t Pid_Update(Controller_t *ctrl, float dt_s)
{
  float error;
  float derivative;
  float integral_candidate;
  float unsaturated;
  float output;

  CheckProtection(ctrl);
  if (ctrl->fault != FAULT_NONE)
  {
    ctrl->duty = 0;
    return 0;
  }

  error = TARGET_MM_S - ctrl->speed_filtered;
  ctrl->ff_term = PID_FEEDFORWARD_DUTY;
  ctrl->p_term = PID_KP * error;

  derivative = (ctrl->speed_filtered - ctrl->previous_speed) / dt_s;
  ctrl->previous_speed = ctrl->speed_filtered;
  ctrl->d_filtered += D_FILTER_ALPHA *
                      (derivative - ctrl->d_filtered);
  ctrl->d_term = -PID_KD * ctrl->d_filtered;

  integral_candidate = ctrl->integral + PID_KI * error * dt_s;
  integral_candidate = Clamp(integral_candidate, PID_I_MIN, PID_I_MAX);

  unsaturated = ctrl->ff_term + ctrl->p_term +
                integral_candidate + ctrl->d_term;
  output = Clamp(unsaturated, 0.0f, PID_OUTPUT_MAX);

  if ((unsaturated == output) ||
      ((output >= PID_OUTPUT_MAX) && (error < 0.0f)) ||
      ((output <= 0.0f) && (error > 0.0f)))
  {
    ctrl->integral = integral_candidate;
  }
  ctrl->i_term = ctrl->integral;
  ctrl->duty = (int16_t)output;
  return ctrl->duty;
}

void AppFr_Init(void)
{
  uint32_t now;

  DbgUart_Init();
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
  Controller_Reset(&s_pe9, 0.0f);
  Controller_Reset(&s_pe11, 0.0f);
  Controller_Reset(&s_pe13, 0.0f);
  Controller_Reset(&s_pe14, 0.0f);

  now = HAL_GetTick();
  s_state = RUN_DELAY;
  s_state_tick = now;
  s_loop_tick = now;
  s_print_tick = now;

  DbgUart_Print("mode=FOUR_MOTOR_PID,target=300mm/s\r\n");
  DbgUart_Print("t,state,target,p9_raw,p9_filt,p9_duty,p9_fault,p11_raw,p11_filt,p11_duty,p11_fault,p13_raw,p13_filt,p13_duty,p13_fault,p14_raw,p14_filt,p14_duty,p14_fault\r\n");
}

void AppFr_Run(void)
{
  uint32_t now = HAL_GetTick();
  float dt_s;
  float pe9_raw;
  float pe11_raw;
  float pe13_raw;
  float pe14_raw;
  int16_t pe9_command = 0;
  int16_t pe11_command = 0;
  int16_t pe13_command = 0;
  int16_t pe14_command = 0;

  if ((uint32_t)(now - s_loop_tick) < LOOP_MS)
  {
    return;
  }

  dt_s = (float)(uint32_t)(now - s_loop_tick) / 1000.0f;
  s_loop_tick = now;

  EncoderFr_Update(dt_s);
  EncoderFl_Update(dt_s);
  EncoderPe13_Update(dt_s);
  EncoderPe14_Update(dt_s);
  pe9_raw = EncoderFr_GetSpeedMmS();
  pe11_raw = EncoderFl_GetSpeedMmS();
  pe13_raw = EncoderPe13_GetSpeedMmS();
  pe14_raw = EncoderPe14_GetSpeedMmS();
  UpdateFilteredSpeed(&s_pe9, pe9_raw);
  UpdateFilteredSpeed(&s_pe11, pe11_raw);
  UpdateFilteredSpeed(&s_pe13, pe13_raw);
  UpdateFilteredSpeed(&s_pe14, pe14_raw);

  if ((s_state == RUN_DELAY) &&
      ((uint32_t)(now - s_state_tick) >= START_DELAY_MS))
  {
    s_state = RUN_BOOST;
    s_state_tick = now;
  }
  else if ((s_state == RUN_BOOST) &&
           ((uint32_t)(now - s_state_tick) >= START_BOOST_MS))
  {
    s_state = RUN_PID;
    s_state_tick = now;
    s_pe9.previous_speed = s_pe9.speed_filtered;
    s_pe11.previous_speed = s_pe11.speed_filtered;
    s_pe13.previous_speed = s_pe13.speed_filtered;
    s_pe14.previous_speed = s_pe14.speed_filtered;
    s_pe9.integral = 0.0f;
    s_pe11.integral = 0.0f;
    s_pe13.integral = 0.0f;
    s_pe14.integral = 0.0f;
  }

  if (s_state == RUN_BOOST)
  {
    s_pe9.duty = START_BOOST_DUTY;
    s_pe11.duty = START_BOOST_DUTY;
    s_pe13.duty = START_BOOST_DUTY;
    s_pe14.duty = START_BOOST_DUTY;
  }
  else if (s_state == RUN_PID)
  {
    s_pe9.duty = Pid_Update(&s_pe9, dt_s);
    s_pe11.duty = Pid_Update(&s_pe11, dt_s);
    s_pe13.duty = Pid_Update(&s_pe13, dt_s);
    s_pe14.duty = Pid_Update(&s_pe14, dt_s);
  }

  pe9_command = (int16_t)(FR_DRIVE_SIGN * s_pe9.duty);
  pe11_command = (int16_t)(FL_DRIVE_SIGN * s_pe11.duty);
  pe13_command = (int16_t)(PE13_DRIVE_SIGN * s_pe13.duty);
  pe14_command = (int16_t)(PE14_DRIVE_SIGN * s_pe14.duty);
  MotorFr_SetDuty(pe9_command);
  MotorFl_SetDuty(pe11_command);
  MotorPe13_SetDuty(pe13_command);
  MotorPe14_SetDuty(pe14_command);

  if ((uint32_t)(now - s_print_tick) >= PRINT_MS)
  {
    s_print_tick = now;
    DbgUart_Printf("%lu,%u,%d,%ld,%ld,%d,%u,%ld,%ld,%d,%u,%ld,%ld,%d,%u,%ld,%ld,%d,%u\r\n",
                   (unsigned long)now,
                   (unsigned int)s_state,
                   (s_state >= RUN_BOOST) ? (int)TARGET_MM_S : 0,
                   (long)DbgUart_Scaled(pe9_raw, 1.0f),
                   (long)DbgUart_Scaled(s_pe9.speed_filtered, 1.0f),
                   (int)s_pe9.duty,
                   (unsigned int)s_pe9.fault,
                   (long)DbgUart_Scaled(pe11_raw, 1.0f),
                   (long)DbgUart_Scaled(s_pe11.speed_filtered, 1.0f),
                   (int)s_pe11.duty,
                   (unsigned int)s_pe11.fault,
                   (long)DbgUart_Scaled(pe13_raw, 1.0f),
                   (long)DbgUart_Scaled(s_pe13.speed_filtered, 1.0f),
                   (int)s_pe13.duty,
                   (unsigned int)s_pe13.fault,
                   (long)DbgUart_Scaled(pe14_raw, 1.0f),
                   (long)DbgUart_Scaled(s_pe14.speed_filtered, 1.0f),
                   (int)s_pe14.duty,
                   (unsigned int)s_pe14.fault);
  }
}
