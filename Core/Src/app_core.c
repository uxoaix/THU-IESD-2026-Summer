#include "app_core.h"
#include "actuator_servos.h"
#include "dbg_uart.h"
#include "hc05_uart.h"
#include "imu_driver.h"
#include "imu_processor.h"
#include "motion_strategy.h"
#include "openmv_uart.h"
#include "sensor_fusion.h"
#include "ultrasonic_front.h"
#include "wheel_speed_control.h"
#include "stm32f1xx_hal.h"

#define CONTROL_PERIOD_MS 10U
#define START_DELAY_MS 1000U
#define TELEMETRY_PERIOD_MS 200U

typedef enum { CONTROL_MANUAL = 0, CONTROL_AUTO = 1 } ControlMode_t;
static ControlMode_t s_mode;
static MotionCommand_t s_command;
static WheelFeedback_t s_wheels;
static VisionData_t s_vision;
static WallSensorData_t s_wall;
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

static void StopAndResetAuto(void)
{
  Motion_StopOutput(&s_command);
  ApplyTargets(&s_command);
  WheelSpeedControl_StopAll();
  MotionStrategy_Stop();
}

static void SetManualMotion(float v, float w)
{
  if (s_mode != CONTROL_MANUAL) {
    ActuatorServos_Stop();
    StopAndResetAuto();
  }
  s_mode = CONTROL_MANUAL;
  Motion_Drive(v, w, &s_command);
  ApplyTargets(&s_command);
}

static void HandleBluetooth(uint32_t now)
{
  Hc05Event_t event;
  while ((event = Hc05Uart_TakeEvent()) != HC05_EVENT_NONE) {
    switch (event) {
    case HC05_EVENT_AUTO:
      /* 自动任务总是从舵机安全初始位和完整新任务开始。 */
      ActuatorServos_Stop();
      StopAndResetAuto();
      s_mode = CONTROL_AUTO;
      s_motion_tick = now;
      break;
    case HC05_EVENT_MANUAL:
      ActuatorServos_Stop();
      s_mode = CONTROL_MANUAL;
      StopAndResetAuto();
      break;
    case HC05_EVENT_STOP:
      /* STOP 为安全急停：底盘立即停 PWM，舵机回到配置的安全初始位。 */
      ActuatorServos_Stop();
      s_mode = CONTROL_MANUAL;
      StopAndResetAuto();
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
        StopAndResetAuto();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_DOOR, now); break;
    case HC05_EVENT_SERVO_CAMERA:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        StopAndResetAuto();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_CAMERA, now); break;
    case HC05_EVENT_SERVO_BUCKET:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        StopAndResetAuto();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BUCKET, now); break;
    case HC05_EVENT_SERVO_BRUSH:
      if (s_mode != CONTROL_MANUAL) {
        ActuatorServos_Stop();
        StopAndResetAuto();
      }
      s_mode = CONTROL_MANUAL;
      (void)ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BRUSH, now); break;
    case HC05_EVENT_STATUS:
      s_telemetry_tick = 0U; break;
    default: break;
    }
  }
}

static void SendTelemetry(uint32_t now)
{
  FusionState_t fusion;
  /* 手动停车模式也刷新视觉快照，便于只测试OpenMV→STM32传输。 */
  OpenMvUart_GetLatest(&s_vision);
  SensorFusion_GetState(&fusion);
  DbgUart_Printf("S,%lu,mode=%u,imu=%u,fused=%u,count=%u,state=%u,"
                 "det=%u,color=%u,cx=%u,cy=%u,xoff=%d,yoff=%d,"
                 "dist=%u,vision_ok=%lu,vision_bad=%lu,wall=%u\n",
    (unsigned long)now, (unsigned int)s_mode,
    (unsigned int)fusion.imu_alive, (unsigned int)fusion.fused,
    (unsigned int)MotionStrategy_GetCollectedCount(),
    (unsigned int)MotionStrategy_GetState(), (unsigned int)s_vision.detected,
    (unsigned int)s_vision.object_type,
    (unsigned int)s_vision.center_x_px,
    (unsigned int)s_vision.center_y_px,
    (int)s_vision.x_offset_px, (int)s_vision.y_offset_px,
    (unsigned int)s_vision.distance_cm,
    (unsigned long)OpenMvUart_GetValidFrameCount(),
    (unsigned long)OpenMvUart_GetInvalidFrameCount(),
    (unsigned int)s_wall.distance_cm);
}

void AppCore_Init(void)
{
  DbgUart_Init();
  WheelSpeedControl_Init();
  ActuatorServos_Init();
  OpenMvUart_Init();
  UltrasonicFront_Init();
  Hc05Uart_Init();
  (void)IMU_Init();
  IMUProcessor_Init();
  SensorFusion_Init();
  MotionStrategy_Init();
  s_mode = DEFAULT_CONTROL_MODE ? CONTROL_AUTO : CONTROL_MANUAL;
  s_start_tick = HAL_GetTick();
  s_control_tick = s_motion_tick = s_telemetry_tick = s_start_tick;
  if (s_mode == CONTROL_MANUAL) StopAndResetAuto();
}

void AppCore_Run(void)
{
  uint32_t now = HAL_GetTick();
  Hc05Uart_Process();
  HandleBluetooth(now);
  OpenMvUart_Process();
  UltrasonicFront_Run(now);
  ActuatorServos_Run(now);

  if ((uint32_t)(now - s_control_tick) >= CONTROL_PERIOD_MS) {
    float dt = (float)(uint32_t)(now - s_control_tick) / 1000.0f;
    s_control_tick = now;
    WheelSpeedControl_Run(dt,
      ((uint32_t)(now - s_start_tick) >= START_DELAY_MS) ? 1U : 0U);
  }

  if (s_mode == CONTROL_AUTO &&
      (uint32_t)(now - s_motion_tick) >= MOTION_PERIOD_MS) {
    uint32_t elapsed = (uint32_t)(now - s_motion_tick);
    s_motion_tick = now;
    OpenMvUart_GetLatest(&s_vision);
    UltrasonicFront_GetLatest(&s_wall);
    ReadWheelFeedback(elapsed);
    MotionStrategy_Update(&s_vision, &s_wheels, &s_wall, elapsed, &s_command);
    ApplyTargets(&s_command);
    OpenMvUart_SetDetectionMode(s_command.detect_black_area);
    ActuatorServos_Apply(&s_command, now);
  }

  if ((uint32_t)(now - s_telemetry_tick) >= TELEMETRY_PERIOD_MS) {
    s_telemetry_tick = now;
    SendTelemetry(now);
  }
}
