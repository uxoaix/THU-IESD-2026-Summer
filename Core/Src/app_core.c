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

typedef enum {
  CONTROL_MANUAL = 0,
  CONTROL_AUTO = 1,
  CONTROL_RETURN = 2,
  CONTROL_ROTATE = 3
} ControlMode_t;

/* 返航/掉头与自动任务共用状态机主循环。 */
#define IS_MOTION_LOOP_MODE(mode) \
  ((mode) == CONTROL_AUTO || (mode) == CONTROL_RETURN || \
   (mode) == CONTROL_ROTATE)

static ControlMode_t s_mode;
static MotionCommand_t s_command;
static WheelFeedback_t s_wheels;
static VisionData_t s_vision;
static VisionWallData_t s_vision_wall;
static VisionArrivalData_t s_vision_arrival;
static uint32_t s_start_tick, s_control_tick, s_motion_tick, s_telemetry_tick;

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
  if (s_mode == CONTROL_RETURN || s_mode == CONTROL_ROTATE) {
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
    case HC05_EVENT_RETURN:
      ActuatorServos_Stop();
      StopWheels();
      MotionStrategy_RequestReturn();
      s_mode = CONTROL_RETURN;
      s_motion_tick = now;
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

static void SendTelemetry(uint32_t now)
{
  float linear_cm_s;
  float angular_deg_s;
  const float rad_to_deg = 180.0f / 3.14159265358979f;

  OpenMvUart_GetLatest(&s_vision);
  OpenMvUart_GetWall(&s_vision_wall);
  OpenMvUart_GetArrival(&s_vision_arrival);

  linear_cm_s = ImuOdometry_GetLinearVelocityCmS();
  angular_deg_s = ImuOdometry_GetYawRateRadS() * rad_to_deg;

  /* 分四次发送：单条格式化结果已接近 DbgUart 的 240 字节缓冲上限，
   * 状态名较长或运行时间变大时会被截断，拆开后线上字节序列不变。 */
  DbgUart_Printf("S,%lu,mode=%u,v_cm_s_x10=%ld,w_deg_s_x10=%ld,count=%u,"
                 "state=%u,state_name=%s,\n",
    (unsigned long)now, (unsigned int)s_mode,
    (long)DbgUart_Scaled(linear_cm_s, 10.0f),
    (long)DbgUart_Scaled(angular_deg_s, 10.0f),
    (unsigned int)MotionStrategy_GetCollectedCount(),
    (unsigned int)MotionStrategy_GetState(),
    MotionStrategy_GetStateName());

  /* yaw_imu / yaw_enc 是未校准的分源航向，用来定位标度误差归属。 */
  DbgUart_Printf("home_x=%d,home_y=%d,hdg_deg=%d,bear_deg=%d,"
                 "imu=%u,yaw_imu=%d,yaw_enc=%d\n",
    (int)ImuOdometry_GetXcm(),
    (int)ImuOdometry_GetYcm(),
    (int)ImuOdometry_GetHeadingDeg(),
    (int)ImuOdometry_GetBearingDeg(),
    (unsigned int)ImuOdometry_IsImuAlive(),
    (int)ImuOdometry_GetImuYawDeg(),
    (int)ImuOdometry_GetEncHeadingDeg());

  DbgUart_Printf("det=%u,color=%u,cx=%u,cy=%u,xoff=%d,yoff=%d,"
                 "dist=%u,vision_ok=%lu,vision_bad=%lu\n",
    (unsigned int)s_vision.detected,
    (unsigned int)s_vision.object_type,
    (unsigned int)s_vision.center_x_px,
    (unsigned int)s_vision.center_y_px,
    (int)s_vision.x_offset_px, (int)s_vision.y_offset_px,
    (unsigned int)s_vision.distance_cm,
    (unsigned long)OpenMvUart_GetValidFrameCount(),
    (unsigned long)OpenMvUart_GetInvalidFrameCount());

  /* vwall_* 来自 OpenMV 的 W 帧 (蓝色边界墙)，是车上唯一的障碍感知。
   * vwall=1 表示已近到触发退避；vwall_pct 是蓝色占 ROI 的百分比，
   * 用它对照实测位置标定 OpenMV 侧的 WALL_FILL_ENTER/EXIT_PCT。
   * arv_* 来自 A 帧 (黑区到位)，只在黑区模式下刷新，物块模式恒为 0；
   * arv_pct 是黑区外框占画面的百分比，用它标定 BLACK_ARRIVAL_FILL_PCT。 */
  DbgUart_Printf("vwall=%u,vwall_pct=%u,vwall_ok=%u,wall_frames=%lu,"
                 "arv=%u,arv_pct=%u,arv_ok=%u,arv_frames=%lu\n",
    (unsigned int)s_vision_wall.blocked,
    (unsigned int)s_vision_wall.fill_pct,
    (unsigned int)s_vision_wall.valid,
    (unsigned long)OpenMvUart_GetWallFrameCount(),
    (unsigned int)s_vision_arrival.arrived,
    (unsigned int)s_vision_arrival.fill_pct,
    (unsigned int)s_vision_arrival.valid,
    (unsigned long)OpenMvUart_GetArrivalFrameCount());
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
  s_mode = DEFAULT_CONTROL_MODE ? CONTROL_AUTO : CONTROL_MANUAL;
  s_start_tick = HAL_GetTick();
  s_control_tick = s_motion_tick = s_telemetry_tick = s_start_tick;
  if (s_mode == CONTROL_MANUAL) {
    MotionStrategy_Stop();
    StopWheels();
  }
}

void AppCore_Run(void)
{
  uint32_t now = HAL_GetTick();
  Hc05Uart_Process();
  HandleBluetooth(now);
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
    SendTelemetry(now);
  }
}
