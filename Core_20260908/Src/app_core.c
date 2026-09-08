#include "app_core.h"
#include "actuator_servos.h"
#include "dbg_uart.h"
#include "hc05_uart.h"
#include "imu_odometry.h"
#include "imu_driver.h"
#include "imu_processor.h"
#include "motion_strategy.h"
#include "openmv_uart.h"
#include "wheel_speed_control.h"
#include "stm32f1xx_hal.h"

#define CONTROL_PERIOD_MS 10U
#define START_DELAY_MS 1000U
#define TELEMETRY_PERIOD_MS 100U
/*
 * 上电自动进 AUTO 的延时 (DEFAULT_CONTROL_MODE=1 时生效)。
 *
 * 不在 AppCore_Init 里直接置 AUTO, 是因为 Init 跑在各外设刚初始化完的那一瞬:
 * IMU 还没出第一帧、OpenMV 还没握上手, 状态机第一个周期就开始判这些量, 拿到
 * 的是全零。留一段时间让它们进入稳态再起跑。
 *
 * 1.5s 已越过 START_DELAY_MS(1.0s), 所以起跑时轮子闭环早已使能, 状态机一发
 * 指令车就走; 若把本值压回 1.0s 以内, 头一段指令会被轮控丢掉。
 *
 * 这段时间里车处于手动态、轮子停着, 期间收到任何蓝牙指令 (STATUS 除外) 都会
 * 取消这次自启动 —— 上电就想手动接管时不必跟它抢。
 */
#define AUTO_START_DELAY_MS 1500U  /* 500 */

typedef enum {
  CONTROL_MANUAL = 0,
  CONTROL_AUTO = 1,
  CONTROL_ROTATE = 3
} ControlMode_t;

/* 蓝牙定角掉头与自动任务共用状态机主循环。 */
#define IS_MOTION_LOOP_MODE(mode) \
  ((mode) == CONTROL_AUTO || (mode) == CONTROL_ROTATE)

static ControlMode_t s_mode;
static MotionCommand_t s_command;
static WheelFeedback_t s_wheels;
static VisionData_t s_vision;
static VisionWallData_t s_vision_wall;
static VisionArrivalData_t s_vision_arrival;
static uint32_t s_start_tick, s_control_tick, s_motion_tick, s_telemetry_tick;
/* 1 = 还欠一次上电自启动 AUTO, 见 AUTO_START_DELAY_MS。 */
static uint8_t s_auto_start_pending;

static void ReadWheelFeedback(uint32_t elapsed_ms)
{
  static float previous[WHEEL_COUNT];
  uint8_t i;
  float dt = (float)elapsed_ms / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  for (i = 0U; i < WHEEL_COUNT; i++) {
    float now = WheelSpeedControl_GetSpeedMmS((WheelId_t)i) * 0.1f;
    s_wheels.speed_cm_s[i] = now;
    s_wheels.accel_cm_s2[i] = (now - previous[i]) / dt;
    previous[i] = now;
  }
}

static void ApplyTargets(const MotionCommand_t *command)
{
  uint8_t i;
  for (i = 0U; i < WHEEL_COUNT; i++) {
    WheelSpeedControl_SetTargetMmS((WheelId_t)i,
                                  command->target_speed_cm_s[i] * 10.0f);
  }
}

static void StopWheels(void)
{
  Motion_StopOutput(&s_command);
  ApplyTargets(&s_command);
  WheelSpeedControl_StopAll();
}

static void StopAndResetAuto(void)
{
  StopWheels();
  MotionStrategy_Init();
}

static void SetManualMotion(float v, float w)
{
  if (s_mode == CONTROL_ROTATE) {
    StopWheels();
    MotionStrategy_Stop();
    s_mode = CONTROL_MANUAL;
  } else if (s_mode != CONTROL_MANUAL) {
    ActuatorServos_Stop();
    StopAndResetAuto();
    s_mode = CONTROL_MANUAL;
  }
  Motion_Drive(v, w, &s_command);
  ApplyTargets(&s_command);
}

static void RunMotionLoop(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - s_motion_tick);
  s_motion_tick = now;
  OpenMvUart_GetLatest(&s_vision);
  OpenMvUart_GetWall(&s_vision_wall);
  OpenMvUart_GetArrival(&s_vision_arrival);
  ReadWheelFeedback(elapsed);
  MotionStrategy_Update(&s_vision, &s_wheels, &s_vision_wall,
                        &s_vision_arrival, elapsed, &s_command);
  ApplyTargets(&s_command);
  OpenMvUart_SetDetectionMode(s_command.detect_black_area);
  ActuatorServos_Apply(&s_command, now);
}

static void RunOdometryOnly(uint32_t now)
{
  uint32_t elapsed = (uint32_t)(now - s_motion_tick);
  float dt = (float)elapsed / 1000.0f;
  s_motion_tick = now;
  ReadWheelFeedback(elapsed);
  ImuOdometry_Update(&s_wheels, dt);
}

static void HandleBluetooth(uint32_t now)
{
  Hc05Event_t event;
  while ((event = Hc05Uart_TakeEvent()) != HC05_EVENT_NONE) {
    /* 人一插手就取消待执行的自启动，免得半秒后又被抢回 AUTO。
     * STATUS 只是索要遥测，不算插手。 */
    if (event != HC05_EVENT_STATUS) {
      s_auto_start_pending = 0U;
    }
    switch (event) {
    case HC05_EVENT_AUTO:
      ActuatorServos_Stop();
      StopAndResetAuto();
      s_mode = CONTROL_AUTO;
      s_motion_tick = now;
      break;
    case HC05_EVENT_MANUAL:
      ActuatorServos_Stop();
      MotionStrategy_Stop();
      StopWheels();
      s_mode = CONTROL_MANUAL;
      break;
    case HC05_EVENT_STOP:
      ActuatorServos_Stop();
      if (s_mode == CONTROL_AUTO) {
        StopAndResetAuto();
      } else {
        MotionStrategy_Stop();
        StopWheels();
      }
      s_mode = CONTROL_MANUAL;
      break;
    case HC05_EVENT_FORWARD:
      SetManualMotion(HC05_MANUAL_LINEAR_SPEED_CM_S, 0.0f); break;
    case HC05_EVENT_BACKWARD:
      SetManualMotion(-HC05_MANUAL_LINEAR_SPEED_CM_S, 0.0f); break;
    case HC05_EVENT_LEFT:
      SetManualMotion(TIGHT_TURN_LINEAR_SPEED_CM_S,
                      DEG_TO_RAD(TIGHT_TURN_ANGULAR_SPEED_DEG_S)); break;
    case HC05_EVENT_RIGHT:
      SetManualMotion(TIGHT_TURN_LINEAR_SPEED_CM_S,
                      -DEG_TO_RAD(TIGHT_TURN_ANGULAR_SPEED_DEG_S)); break;
    case HC05_EVENT_ROTATE_CW:
      SetManualMotion(0.0f, -HC05_MANUAL_ANGULAR_SPEED_RAD_S); break;
    case HC05_EVENT_ROTATE_CCW:
      SetManualMotion(0.0f, HC05_MANUAL_ANGULAR_SPEED_RAD_S); break;
    case HC05_EVENT_SERVO_DOOR:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        MotionStrategy_Stop();
        StopWheels();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_DOOR, now); break;
    case HC05_EVENT_SERVO_CAMERA:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        MotionStrategy_Stop();
        StopWheels();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_CAMERA, now); break;
    case HC05_EVENT_SERVO_BUCKET:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        MotionStrategy_Stop();
        StopWheels();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BUCKET, now); break;
    case HC05_EVENT_SERVO_BRUSH:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        MotionStrategy_Stop();
        StopWheels();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BRUSH, now); break;
    case HC05_EVENT_SET_HOME:
      ActuatorServos_Stop();
      MotionStrategy_Stop();
      StopWheels();
      ImuOdometry_SetHome();
      s_mode = CONTROL_MANUAL;
      break;
    case HC05_EVENT_ROTATE_180:
      ActuatorServos_Stop();
      StopWheels();
      MotionStrategy_RequestRotateCw(BT_ROTATE180_TARGET_DEG);
      s_mode = CONTROL_ROTATE;
      s_motion_tick = now;
      break;
    case HC05_EVENT_ROTATE_360:
      ActuatorServos_Stop();
      StopWheels();
      MotionStrategy_RequestRotateCw(BT_ROTATE360_TARGET_DEG);
      s_mode = CONTROL_ROTATE;
      s_motion_tick = now;
      break;
    case HC05_EVENT_STATUS:
      s_telemetry_tick = 0U; break;
    default: break;
    }
  }
}

/*
 * 遥测一行发完 (约 130 字节, 远低于 DbgUart 的 240 字节缓冲), 所以不再像以前
 * 那样拆成四条。字段顺序即固定的线上顺序。
 *
 * tick   HAL_GetTick(), 上电起的毫秒数。放在第一位是为了一眼看出 MCU 有没有
 *        复位: 舵机乱动、状态机莫名回到 STANDBY 这类"像是大脑错乱"的现象,
 *        如果同时 tick 跳回接近 0, 就是掉电复位 (供电问题), 往电机/舵机的
 *        电流尖峰上查; 如果 tick 一路平稳上涨, 那就不是复位, 得往信号线、
 *        PWM 时序、机械上查。没有这个字段就区分不了这两类原因。
 * state  状态机数值编号。会随 MotionState_t 增删而整体前移, 请对照
 *        motion_config.h 的枚举读, 别记死数字 (state_name 已不再发送)。
 * hdg_deg 融合航向 (IMU + 编码器卡尔曼融合, 过 FUSION_YAW_SCALE), 0~359 角度制,
 *        上电为 0, 左转(CCW)增大、右转(CW)减小, 0/360 处回绕。蓝牙 SET_HOME
 *        与 AUTO 起步会重新归零。定角旋转、掉头、里程计积分都用这一路。
 * imu_yaw 纯 IMU 航向, 不掺编码器, 只把模组 yaw 寄存器的增量展开累加, 再过
 *        IMU_YAW_SCALE(0.83)。零点、量程、方向、归零时机与 hdg_deg 完全一致,
 *        所以两者可以直接相减: 差值就是编码器打滑把融合航向拽偏的量。
 *        HOME_FOLLOW 锁定方位角之后的保向闭环走的是这一路。
 *        注意已乘 0.83, 想看模组原始读数要除回去。
 * bear_deg 返航直线段该朝的方向, 与 hdg_deg 同零点同量纲 (0~359)。返航中报
 *        HOME_PREPARE 锁定的方位角 (含遇墙退避累计的右旋量), 其余时候报当前
 *        位置指向原点的实时方位角。hdg_deg 减 bear_deg 就是航向偏差, 直接看
 *        得出车有没有对准家。
 * color  视觉目标类型: 0=无, 1=红, 2=黄, 3=黑色卸货区。
 *
 * 其余字段的取数函数都还在 (ImuOdometry_GetEncHeadingDeg / IsImuAlive 等),
 * 只是不再打印 —— 要临时加回某个观察量, 在下面的格式串里补一项即可。
 * 想定位标度误差归属就把 yaw_enc (纯编码器航向) 也加上, 原地转过已知角度后,
 * hdg_deg / imu_yaw / yaw_enc 三者谁的增量偏离实际转角, 问题就在谁身上。
 */
static void SendTelemetry(void)
{
  OpenMvUart_GetLatest(&s_vision);

  DbgUart_Printf("tick=%lu,count=%u,state=%u,home_x=%d,home_y=%d,hdg_deg=%d,"
                 "imu_yaw=%d,bear_deg=%d,color=%u,vision_ok=%lu,"
                 "vision_bad=%lu\n",
    (unsigned long)HAL_GetTick(),
    (unsigned int)MotionStrategy_GetCollectedCount(),
    (unsigned int)MotionStrategy_GetState(),
    (int)ImuOdometry_GetXcm(),
    (int)ImuOdometry_GetYcm(),
    (int)ImuOdometry_GetHeadingDeg(),
    (int)ImuOdometry_GetImuYawDeg(),
    (int)ImuOdometry_GetBearingDeg(),
    (unsigned int)s_vision.object_type,
    (unsigned long)OpenMvUart_GetValidFrameCount(),
    (unsigned long)OpenMvUart_GetInvalidFrameCount());
}

void AppCore_Init(void)
{
  DbgUart_Init();
  WheelSpeedControl_Init();
  ActuatorServos_Init();
  OpenMvUart_Init();
  Hc05Uart_Init();
  (void)IMU_Init();
  IMUProcessor_Init();
  ImuOdometry_Init();
  MotionStrategy_Init();
  /*
   * 一律先停在手动态，AUTO 交给 AppCore_Run 里的延时自启动，
   * 这样 DEFAULT_CONTROL_MODE=1 与蓝牙 AUTO 走的是同一条起跑路径。
   */
  s_mode = CONTROL_MANUAL;
  s_auto_start_pending = DEFAULT_CONTROL_MODE ? 1U : 0U;
  s_start_tick = HAL_GetTick();
  s_control_tick = s_motion_tick = s_telemetry_tick = s_start_tick;
  MotionStrategy_Stop();
  StopWheels();
}

void AppCore_Run(void)
{
  uint32_t now = HAL_GetTick();
  Hc05Uart_Process();
  HandleBluetooth(now);
  /* 上电自启动 AUTO。动作与蓝牙 AUTO 完全一致，包括对齐 s_motion_tick ——
   * 不对齐的话状态机第一个周期会拿到这半秒当 elapsed，计时全乱。 */
  if (s_auto_start_pending &&
      (uint32_t)(now - s_start_tick) >= AUTO_START_DELAY_MS) {
    s_auto_start_pending = 0U;
    ActuatorServos_Stop();
    StopAndResetAuto();
    s_mode = CONTROL_AUTO;
    s_motion_tick = now;
  }
  OpenMvUart_Process();
  ActuatorServos_Run(now);

  if ((uint32_t)(now - s_control_tick) >= CONTROL_PERIOD_MS) {
    float dt = (float)(uint32_t)(now - s_control_tick) / 1000.0f;
    s_control_tick = now;
    WheelSpeedControl_Run(dt,
      ((uint32_t)(now - s_start_tick) >= START_DELAY_MS) ? 1U : 0U);
  }

  if (IS_MOTION_LOOP_MODE(s_mode) &&
      (uint32_t)(now - s_motion_tick) >= MOTION_PERIOD_MS) {
    RunMotionLoop(now);
    if (s_mode != CONTROL_AUTO && MotionStrategy_TakeTaskComplete()) {
      s_mode = CONTROL_MANUAL;
    }
  } else if (s_mode == CONTROL_MANUAL &&
             (uint32_t)(now - s_motion_tick) >= MOTION_PERIOD_MS) {
    RunOdometryOnly(now);
  }

  if ((uint32_t)(now - s_telemetry_tick) >= TELEMETRY_PERIOD_MS) {
    s_telemetry_tick = now;
    SendTelemetry();
  }
}
