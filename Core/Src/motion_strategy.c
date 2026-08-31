#include "motion_strategy.h"
#include "actuator_servos.h"
#include "block_alignment.h"
#include "home_trajectory.h"
#include "sensor_fusion.h"
#include "ultrasonic_avoid.h"

#define PI_F 3.14159265358979f

static MotionState_t s_state;
static uint32_t s_state_ms;
static uint32_t s_missing_ms;
static uint32_t s_sample_ms;
static uint32_t s_tracking_forward_ms;
static uint8_t s_collected;
static uint8_t s_reverse_head;
/*
 * 黑区目标模式与摄像头反转解耦：返航阶段摄像头不旋转，
 * 车头仍是逻辑前方，但视觉目标要切换成黑色卸货区。
 */
static uint8_t s_black_target;
static uint8_t s_wall_black;
static uint8_t s_action_step;
static uint8_t s_small_count;
static float s_scan_heading;
static float s_turn_target;
static float s_move_cm;
static float s_move_target_cm;
static float s_prev_target[WHEEL_COUNT];

static float AbsF(float x) { return x < 0.0f ? -x : x; }
static float WrapPi(float x)
{
  while (x > PI_F) x -= 2.0f * PI_F;
  while (x < -PI_F) x += 2.0f * PI_F;
  return x;
}

/* 所有差速输出集中于此，轮位及底层机械符号不在应用层重映射。 */
void Motion_Drive(float v, float w, MotionCommand_t *out)
{
  float left = v - w * WHEEL_TRACK_WIDTH_CM * 0.5f;
  float right = v + w * WHEEL_TRACK_WIDTH_CM * 0.5f;
  float max = AbsF(left);
  if (AbsF(right) > max) max = AbsF(right);
  if (max > MAX_LINEAR_SPEED_CM_S) {
    left *= MAX_LINEAR_SPEED_CM_S / max;
    right *= MAX_LINEAR_SPEED_CM_S / max;
  }
  out->target_speed_cm_s[WHEEL_FRONT_LEFT] = left;
  out->target_speed_cm_s[WHEEL_REAR_LEFT] = left;
  out->target_speed_cm_s[WHEEL_FRONT_RIGHT] = right;
  out->target_speed_cm_s[WHEEL_REAR_RIGHT] = right;
}

void Motion_Line(float v, MotionCommand_t *out) { Motion_Drive(v, 0.0f, out); }
void Motion_Rotate(float w, MotionCommand_t *out) { Motion_Drive(0.0f, w, out); }
void Motion_StopOutput(MotionCommand_t *out)
{
  uint8_t i;
  for (i = 0U; i < WHEEL_COUNT; i++) out->target_speed_cm_s[i] = 0.0f;
}

static void LogicalDrive(float v, float w, MotionCommand_t *out)
{
  /*
   * 摄像头转到车尾后，逻辑“前进”映射为物理后退；车体偏航角的正方向
   * 不会因坐标系整体旋转 180°而改变，所以角速度不能反号。
   */
  if (s_reverse_head) Motion_Drive(-v, w, out);
  else Motion_Drive(v, w, out);
}

/*
 * 行进中的二次视觉对准：
 * 每个运动周期重新计算水平偏差。偏差越大，前进速度越低、转向修正越强；
 * 回到图像中心后恢复正常直行速度，避免高速前进时修正量不足。
 */
static int16_t GetAlignmentError(int16_t x_offset_px)
{
  return (int16_t)(x_offset_px - TRACKING_CENTER_OFFSET_PX);
}

static void TrackTargetWhileMoving(const VisionData_t *vision,
                                   MotionCommand_t *out)
{
  int16_t alignment_error = GetAlignmentError(vision->x_offset_px);
  float abs_offset = AbsF((float)alignment_error);
  float offset_ratio;
  float linear_ratio;
  float angular;

  if (abs_offset > (float)FUZZY_X_OFFSET_MAX_PX) {
    abs_offset = (float)FUZZY_X_OFFSET_MAX_PX;
  }
  offset_ratio = abs_offset / (float)FUZZY_X_OFFSET_MAX_PX;
  linear_ratio = 1.0f -
    (1.0f - TRACKING_MIN_LINEAR_RATIO) * offset_ratio;

  angular = BlockAlignment_GetAngularCorrection(alignment_error) *
            TRACKING_CORRECTION_GAIN;
  if (angular > MAX_ANGULAR_SPEED_RAD_S) {
    angular = MAX_ANGULAR_SPEED_RAD_S;
  } else if (angular < -MAX_ANGULAR_SPEED_RAD_S) {
    angular = -MAX_ANGULAR_SPEED_RAD_S;
  }

  LogicalDrive(TRACKING_LINEAR_SPEED_CM_S *
               TRACKING_SLOW_RATIO * linear_ratio,
               angular,
               out);
}

static void Enter(MotionState_t state)
{
  s_state = state;
  s_state_ms = 0U;
  s_action_step = 0U;
  if (state == MOTION_STATE_TARGET_TRACKING ||
      state == MOTION_STATE_BLACK_AREA_TRACK) {
    s_tracking_forward_ms = 0U;
  }
}

static void ResumeSearch(void)
{
  s_missing_ms = 0U;
  Enter(s_wall_black ? MOTION_STATE_BLACK_AREA_SEARCH
                     : MOTION_STATE_ROTATE_SEARCH);
}

void MotionStrategy_Init(void)
{
  uint8_t i;
  s_state = MOTION_STATE_INIT;
  s_state_ms = s_missing_ms = s_sample_ms = 0U;
  s_tracking_forward_ms = 0U;
  s_collected = s_reverse_head = s_wall_black = 0U;
  s_black_target = 0U;
  s_action_step = s_small_count = 0U;
  s_scan_heading = s_turn_target = 0.0f;
  s_move_cm = s_move_target_cm = 0.0f;
  for (i = 0U; i < WHEEL_COUNT; i++) s_prev_target[i] = 0.0f;
  HomeTrajectory_Init();
  UltrasonicAvoid_ResetScan();
  SensorFusion_ResetHeading();
}

void MotionStrategy_Stop(void) { MotionStrategy_Init(); }

void MotionStrategy_Update(const VisionData_t *vision,
                           const WheelFeedback_t *wheels,
                           const WallSensorData_t *wall,
                           uint32_t elapsed_ms,
                           MotionCommand_t *out)
{
  uint8_t i;
  float dt;
  float heading;
  uint8_t wanted_detected;
  if (!elapsed_ms) elapsed_ms = 1U;
  dt = (float)elapsed_ms / 1000.0f;
  for (i = 0U; i < WHEEL_COUNT; i++) {
    out->target_speed_cm_s[i] = 0.0f;
    out->target_accel_cm_s2[i] = 0.0f;
  }
  out->brush_cycle = out->bucket_cycle = out->door_cycle = 0U;
  out->camera_cycle = 0U;
  out->detect_black_area = s_black_target;

  SensorFusion_Update(wheels, dt);
  heading = SensorFusion_GetHeading();
  s_state_ms += elapsed_ms;
  if (HomeTrajectory_IsOriginSet()) {
    HomeTrajectory_Update(SensorFusion_GetLinearVelocity(),
                          heading, dt);
  }
  wanted_detected = vision->detected &&
    ((s_black_target && vision->object_type == BLACK_AREA_OBJECT_TYPE) ||
     (!s_black_target && vision->object_type != BLACK_AREA_OBJECT_TYPE));
  if (wanted_detected) s_missing_ms = 0U;
  else s_missing_ms += elapsed_ms;

  switch (s_state) {
  case MOTION_STATE_INIT: {
    FusionState_t fusion;
    Motion_StopOutput(out);
    SensorFusion_GetState(&fusion);
    if (!HomeTrajectory_IsOriginSet()) {
      if (!fusion.imu_alive && s_state_ms < HOME_ORIGIN_WAIT_MS) {
        break;
      }
      SensorFusion_ResetHeading();
      heading = SensorFusion_GetHeading();
      HomeTrajectory_SetOrigin(heading);
    }
    s_reverse_head = 0U;
    s_black_target = 0U;
    s_wall_black = 0U;
    s_collected = 0U;
    s_missing_ms = 0U;
    Enter(MOTION_STATE_ROTATE_SEARCH);
    break;
  }

  case MOTION_STATE_ROTATE_SEARCH:
    if (s_collected >= TOTAL_OBJECTS_TO_COLLECT) {
      Enter(MOTION_STATE_HOME_PREPARE);
      break;
    }
#if OBJECT_APPROACH_TEST_MODE
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else {
      /*
       * 与蓝牙RIGHT共用同一组紧凑右转参数：
       * 左侧外轮正转，右侧内轮反转。
       */
      Motion_Drive(TIGHT_TURN_LINEAR_SPEED_CM_S,
                   -DEG_TO_RAD(TIGHT_TURN_ANGULAR_SPEED_DEG_S), out);
    }
#else
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (s_missing_ms >= SEARCH_NO_TARGET_TIMEOUT_MS) {
      s_wall_black = 0U;
      s_scan_heading = heading;
      s_sample_ms = 0U;
      UltrasonicAvoid_ResetScan();
      Enter(MOTION_STATE_WALL_SCAN_FIRST);
    } else {
      Motion_Rotate(-DEG_TO_RAD(SEARCH_ROTATION_SPEED_DEG_S), out);
    }
#endif
    break;

  case MOTION_STATE_PRE_CENTERING:
    if (!wanted_detected || s_state_ms >= PRE_CENTERING_TIMEOUT_MS) {
      ResumeSearch();
    } else if (AbsF((float)GetAlignmentError(vision->x_offset_px)) <=
               TRACKING_DEADZONE_PX) {
      Enter(s_black_target ? MOTION_STATE_BLACK_AREA_TRACK
                           : MOTION_STATE_TARGET_TRACKING);
    } else {
      LogicalDrive(0.0f,
        BlockAlignment_GetAngularCorrection(
          GetAlignmentError(vision->x_offset_px)), out);
    }
    break;

  case MOTION_STATE_TARGET_TRACKING:
  case MOTION_STATE_BLACK_AREA_TRACK:
    if (!wanted_detected) {
      /*
       * 普通物块已经完成对齐并实际向前追踪后，目标消失通常意味着
       * 物块被车体/滚刷遮挡，小车此刻已停止；将该停止事件视为追踪完成。
       */
      if (!s_black_target &&
          s_tracking_forward_ms >= TRACKING_COMPLETE_MIN_FORWARD_MS &&
          s_missing_ms >= TRACKING_COMPLETE_LOSS_MS) {
        Enter(MOTION_STATE_FINAL_APPROACH);
      } else if (s_missing_ms >= TARGET_LOCK_WINDOW_MS) {
        ResumeSearch();
      }
    } else if (vision->distance_cm <=
               (s_black_target ? BLACK_AREA_ARRIVE_CM : COLLECT_DISTANCE_CM)) {
      if (s_black_target) {
        /* 已到黑区前：原地掉头把后斗对准卸货区，再执行卸货序列。 */
        s_turn_target = heading + PI_F;
        Enter(MOTION_STATE_HOME_TURN_AROUND);
      } else {
        Enter(MOTION_STATE_FINAL_APPROACH);
      }
    } else {
      s_tracking_forward_ms += elapsed_ms;
      TrackTargetWhileMoving(vision, out);
    }
    break;

  case MOTION_STATE_FINAL_APPROACH:
    /*
     * 视觉追踪已结束，保持最后对准方向，以追踪速度继续前进1.3秒，
     * 把物块送入滚刷范围。
     */
    if (s_state_ms >= FINAL_APPROACH_DURATION_MS) {
      Motion_StopOutput(out);
      Enter(MOTION_STATE_BRUSH_COLLECT);
    } else {
      LogicalDrive(TRACKING_LINEAR_SPEED_CM_S * TRACKING_SLOW_RATIO,
                   0.0f,
                   out);
    }
    break;

  case MOTION_STATE_BRUSH_COLLECT:
    Motion_StopOutput(out);
    if (s_action_step == 0U) {
      out->brush_cycle = 1U;
      s_action_step = 1U;
    } else if (s_action_step == 1U &&
               ActuatorServos_IsBusy(ACTUATOR_SERVO_BRUSH)) {
      s_action_step = 2U;
    } else if (s_action_step == 2U &&
               !ActuatorServos_IsBusy(ACTUATOR_SERVO_BRUSH)) {
      s_state_ms = 0U;
      s_action_step = 3U;
    } else if (s_action_step == 3U &&
               s_state_ms >= COLLECT_BUCKET_DELAY_MS) {
      out->bucket_cycle = 1U;
      s_action_step = 4U;
    } else if (s_action_step == 4U &&
               ActuatorServos_IsBusy(ACTUATOR_SERVO_BUCKET)) {
      s_action_step = 5U;
    } else if (s_action_step == 5U &&
               !ActuatorServos_IsBusy(ACTUATOR_SERVO_BUCKET)) {
      s_collected++;
      Enter(MOTION_STATE_SEARCH_CONTINUE);
    }
    if (s_state_ms >= BRUSH_TIMEOUT_MS + BUCKET_OUTBOUND_MS +
                      BUCKET_RETURN_MS + COLLECT_BUCKET_DELAY_MS) {
      Enter(MOTION_STATE_SEARCH_CONTINUE);
    }
    break;

  case MOTION_STATE_SEARCH_CONTINUE:
    s_missing_ms = 0U;
    Enter((s_collected >= TOTAL_OBJECTS_TO_COLLECT)
        ? MOTION_STATE_HOME_PREPARE
        : MOTION_STATE_ROTATE_SEARCH);
    break;

  case MOTION_STATE_HOME_PREPARE:
    Motion_StopOutput(out);
    /* 直线段只跑全程的 HOME_PARTIAL_RETURN_RATIO，剩余距离作为退出阈值。 */
    s_move_target_cm = HomeTrajectory_GetDistanceCm() *
                       (1.0f - HOME_PARTIAL_RETURN_RATIO);
    Enter(MOTION_STATE_HOME_FOLLOW);
    break;

  case MOTION_STATE_HOME_FOLLOW: {
    float return_linear;
    float return_angular;
    uint8_t arrived = HomeTrajectory_GetReturnCommand(heading,
                                                     &return_linear,
                                                     &return_angular);
    if (arrived ||
        HomeTrajectory_GetDistanceCm() <= s_move_target_cm ||
        s_state_ms >= HOME_FOLLOW_TIMEOUT_MS) {
      /* 里程计只负责把车带到卸货区附近，精确定位交给视觉。 */
      Motion_StopOutput(out);
      s_black_target = 1U;
      s_wall_black = 1U;
      s_missing_ms = 0U;
      Enter(MOTION_STATE_BLACK_AREA_SEARCH);
    } else {
      Motion_Drive(return_linear, return_angular, out);
    }
    break;
  }

  case MOTION_STATE_HOME_TURN_AROUND: {
    float turn_error = WrapPi(s_turn_target - heading);
    if (AbsF(turn_error) <= DEG_TO_RAD(HOME_TURN_AROUND_TOL_DEG) ||
        s_state_ms >= HOME_TURN_AROUND_TIMEOUT_MS) {
      Motion_StopOutput(out);
      Enter(MOTION_STATE_HOME_DONE);
    } else {
      Motion_Rotate((turn_error > 0.0f)
        ? DEG_TO_RAD(HOME_TURN_AROUND_SPEED_DEG_S)
        : -DEG_TO_RAD(HOME_TURN_AROUND_SPEED_DEG_S), out);
    }
    break;
  }

  case MOTION_STATE_HOME_DONE:
    Motion_StopOutput(out);
    /* 掉头完成：底盘保持停车，接入现有升斗→开门→关门→落斗序列。 */
    Enter(MOTION_STATE_BLACK_AREA_PROCESS);
    break;

  case MOTION_STATE_CAMERA_ROTATE:
    Motion_StopOutput(out);
    if (s_action_step == 0U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_CAMERA,
                                  CAMERA_ANGLE_BLACK_DEG)) {
        s_action_step = 1U;
        s_state_ms = 0U;
      }
    }
    if (s_action_step == 1U &&
        s_state_ms >= CAMERA_ROTATE_DURATION_MS) {
      s_reverse_head = 1U;
      s_black_target = 1U;
      s_wall_black = 1U;
      s_missing_ms = 0U;
      Enter(MOTION_STATE_BLACK_AREA_SEARCH);
    }
    break;

  case MOTION_STATE_BLACK_AREA_SEARCH:
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (s_missing_ms >= BLACK_SEARCH_TIMEOUT_MS) {
      s_wall_black = 1U;
      s_scan_heading = heading;
      s_sample_ms = 0U;
      UltrasonicAvoid_ResetScan();
      Enter(MOTION_STATE_WALL_SCAN_FIRST);
    } else {
      LogicalDrive(0.0f, -DEG_TO_RAD(SEARCH_ROTATION_SPEED_DEG_S), out);
    }
    break;

  case MOTION_STATE_WALL_SCAN_FIRST:
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
      break;
    }
    Motion_Rotate(-DEG_TO_RAD(WALL_SCAN_ROTATION_SPEED_DEG_S), out);
    s_sample_ms += elapsed_ms;
    if (s_sample_ms >= WALL_SAMPLE_INTERVAL_MS) {
      s_sample_ms %= WALL_SAMPLE_INTERVAL_MS;
      if (wall->valid) UltrasonicAvoid_RecordSample(wall->distance_cm, heading);
    }
    if (AbsF(heading - s_scan_heading) >=
          DEG_TO_RAD(WALL_SCAN_TOTAL_DEG - WALL_HEADING_TOL_DEG) ||
        s_state_ms >= WALL_SCAN_TIMEOUT_MS) {
      if (!UltrasonicAvoid_HasValidSample()) {
        ResumeSearch();
      } else {
        /*
         * 摄像头不旋转时（当前流程），超声与摄像头同朝车头，
         * 两个阶段都转到最近墙的反方向；仅在启用摄像头反转的
         * 旧流程里，实体车头才需要朝向最近墙本身。
         */
        s_turn_target = UltrasonicAvoid_GetMinDirection() +
                        (s_reverse_head ? 0.0f : PI_F);
        Enter(MOTION_STATE_WALL_SCAN_SECOND);
      }
    }
    break;

  case MOTION_STATE_WALL_SCAN_SECOND:
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (AbsF(WrapPi(s_turn_target - heading)) <=
        DEG_TO_RAD(WALL_HEADING_TOL_DEG)) {
      s_turn_target = heading +
        ((s_small_count & 1U) ? DEG_TO_RAD(WALL_SMALL_ROTATE_DEG)
                             : -DEG_TO_RAD(WALL_SMALL_ROTATE_DEG));
      Enter(MOTION_STATE_WALL_SMALL_ROTATE);
    } else if (s_state_ms >= WALL_SCAN_TIMEOUT_MS) {
      ResumeSearch();
    } else {
      Motion_Rotate(-DEG_TO_RAD(WALL_SCAN_ROTATION_SPEED_DEG_S), out);
    }
    break;

  case MOTION_STATE_WALL_SMALL_ROTATE:
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (AbsF(WrapPi(s_turn_target - heading)) <=
        DEG_TO_RAD(WALL_SMALL_ROTATE_TOL_DEG)) {
      s_small_count++;
      Enter(MOTION_STATE_WALL_MEASURE_MOVE);
    } else if (s_state_ms >= WALL_SCAN_TIMEOUT_MS) {
      ResumeSearch();
    } else {
      Motion_Rotate((s_small_count & 1U)
        ? DEG_TO_RAD(WALL_SCAN_ROTATION_SPEED_DEG_S)
        : -DEG_TO_RAD(WALL_SCAN_ROTATION_SPEED_DEG_S), out);
    }
    break;

  case MOTION_STATE_WALL_MEASURE_MOVE:
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (s_action_step == 0U) {
      if (wall->valid) {
        s_move_target_cm = (float)wall->distance_cm *
          (float)WALL_TRAVEL_NUMERATOR / (float)WALL_TRAVEL_DENOMINATOR;
        s_move_cm = 0.0f;
        s_action_step = 1U;
        s_state_ms = 0U;
      } else if (s_state_ms >= WALL_MEASURE_TIMEOUT_MS) {
        ResumeSearch();
      }
    } else {
      s_move_cm += AbsF(SensorFusion_GetLinearVelocity()) * dt;
      if (s_move_cm >= s_move_target_cm ||
          s_state_ms >= WALL_MOVE_TIMEOUT_MS) {
        ResumeSearch();
      } else {
        LogicalDrive(WALL_APPROACH_SPEED_CM_S, 0.0f, out);
      }
    }
    break;

  case MOTION_STATE_BLACK_AREA_PROCESS:
    Motion_StopOutput(out);
    if (s_action_step == 0U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_BUCKET, BUCKET_END_DEG)) {
        s_action_step = 1U;
        s_state_ms = 0U;
      }
    }
    if (s_action_step == 1U &&
        s_state_ms >= BUCKET_LIFT_DURATION_MS) {
      Enter(MOTION_STATE_UNLOADING);
    }
    break;

  case MOTION_STATE_UNLOADING:
    Motion_StopOutput(out);
    if (s_action_step == 0U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_DOOR, DOOR_END_DEG)) {
        s_action_step = 1U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 1U &&
               s_state_ms >= BUCKET_UNLOAD_DURATION_MS) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_DOOR, DOOR_START_DEG)) {
        s_action_step = 2U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 2U &&
               s_state_ms >= BUCKET_DOOR_CLOSE_MS) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_BUCKET, BUCKET_START_DEG)) {
        s_action_step = 3U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 3U &&
               s_state_ms >= BUCKET_LOWER_DURATION_MS) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_CAMERA,
                                  CAMERA_ANGLE_BLOCK_DEG)) {
        s_reverse_head = 0U;
        s_black_target = 0U;
        /* 卸货点继续作为下一轮任务的新(0,0)，清除上一轮累计位姿。 */
        HomeTrajectory_Init();
        Enter(MOTION_STATE_INIT);
      }
    }
    break;

  default:
    Enter(MOTION_STATE_INIT);
    break;
  }

  for (i = 0U; i < WHEEL_COUNT; i++) {
    float a = (out->target_speed_cm_s[i] - s_prev_target[i]) / dt;
    if (a > MAX_WHEEL_ACCEL_CM_S2) a = MAX_WHEEL_ACCEL_CM_S2;
    if (a < -MAX_WHEEL_ACCEL_CM_S2) a = -MAX_WHEEL_ACCEL_CM_S2;
    out->target_accel_cm_s2[i] = a;
    s_prev_target[i] = out->target_speed_cm_s[i];
  }
  out->detect_black_area = s_reverse_head;
}

MotionState_t MotionStrategy_GetState(void) { return s_state; }
const char *MotionStrategy_GetStateName(void)
{
  switch (s_state) {
  case MOTION_STATE_INIT:                return "INIT";
  case MOTION_STATE_ROTATE_SEARCH:       return "ROTATE_SEARCH";
  case MOTION_STATE_PRE_CENTERING:       return "PRE_CENTERING";
  case MOTION_STATE_TARGET_TRACKING:     return "TARGET_TRACKING";
  case MOTION_STATE_FINAL_APPROACH:      return "FINAL_APPROACH";
  case MOTION_STATE_BRUSH_COLLECT:       return "BRUSH_COLLECT";
  case MOTION_STATE_SEARCH_CONTINUE:     return "SEARCH_CONTINUE";
  case MOTION_STATE_HOME_PREPARE:        return "HOME_PREPARE";
  case MOTION_STATE_HOME_FOLLOW:         return "HOME_FOLLOW";
  case MOTION_STATE_HOME_TURN_AROUND:    return "HOME_TURN_AROUND";
  case MOTION_STATE_HOME_DONE:           return "HOME_DONE";
  case MOTION_STATE_WALL_SCAN_FIRST:     return "WALL_SCAN_FIRST";
  case MOTION_STATE_WALL_SCAN_SECOND:    return "WALL_SCAN_SECOND";
  case MOTION_STATE_WALL_TURN_OPPOSITE:  return "WALL_TURN_OPPOSITE";
  case MOTION_STATE_WALL_SMALL_ROTATE:   return "WALL_SMALL_ROTATE";
  case MOTION_STATE_WALL_MEASURE_MOVE:   return "WALL_MEASURE_MOVE";
  case MOTION_STATE_CAMERA_ROTATE:       return "CAMERA_ROTATE";
  case MOTION_STATE_BLACK_AREA_SEARCH:   return "BLACK_AREA_SEARCH";
  case MOTION_STATE_BLACK_AREA_TRACK:    return "BLACK_AREA_TRACK";
  case MOTION_STATE_BLACK_AREA_PROCESS:  return "BLACK_AREA_PROCESS";
  case MOTION_STATE_UNLOADING:           return "UNLOADING";
  default:                               return "UNKNOWN";
  }
}
uint8_t MotionStrategy_GetCollectedCount(void) { return s_collected; }
uint8_t MotionStrategy_IsReverseHead(void) { return s_reverse_head; }
