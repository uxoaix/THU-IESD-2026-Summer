#include "motion_strategy.h"
#include "actuator_servos.h"
#include "block_alignment.h"
#include "sensor_fusion.h"
#include "ultrasonic_avoid.h"

#define PI_F 3.14159265358979f

static MotionState_t s_state;
static uint32_t s_state_ms;
static uint32_t s_missing_ms;
static uint32_t s_sample_ms;
static uint8_t s_collected;
static uint8_t s_reverse_head;
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

static void Enter(MotionState_t state)
{
  s_state = state;
  s_state_ms = 0U;
  s_action_step = 0U;
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
  s_collected = s_reverse_head = s_wall_black = 0U;
  s_action_step = s_small_count = 0U;
  s_scan_heading = s_turn_target = 0.0f;
  s_move_cm = s_move_target_cm = 0.0f;
  for (i = 0U; i < WHEEL_COUNT; i++) s_prev_target[i] = 0.0f;
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
  out->detect_black_area = s_reverse_head;

  SensorFusion_Update(wheels, dt);
  heading = SensorFusion_GetHeading();
  s_state_ms += elapsed_ms;
  wanted_detected = vision->detected &&
    ((s_reverse_head && vision->object_type == BLACK_AREA_OBJECT_TYPE) ||
     (!s_reverse_head && vision->object_type != BLACK_AREA_OBJECT_TYPE));
  if (wanted_detected) s_missing_ms = 0U;
  else s_missing_ms += elapsed_ms;

  switch (s_state) {
  case MOTION_STATE_INIT:
    Motion_StopOutput(out);
    s_reverse_head = 0U;
    s_wall_black = 0U;
    s_collected = 0U;
    s_missing_ms = 0U;
    SensorFusion_ResetHeading();
    Enter(MOTION_STATE_ROTATE_SEARCH);
    break;

  case MOTION_STATE_ROTATE_SEARCH:
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
    if (s_collected >= TOTAL_OBJECTS_TO_COLLECT) {
      Enter(MOTION_STATE_CAMERA_ROTATE);
    } else if (wanted_detected) {
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
    } else if (AbsF((float)vision->x_offset_px) <= TRACKING_DEADZONE_PX) {
      Enter(s_reverse_head ? MOTION_STATE_BLACK_AREA_TRACK
                          : MOTION_STATE_TARGET_TRACKING);
    } else {
      LogicalDrive(0.0f,
        BlockAlignment_GetAngularCorrection(vision->x_offset_px), out);
    }
    break;

  case MOTION_STATE_TARGET_TRACKING:
  case MOTION_STATE_BLACK_AREA_TRACK:
    if (!wanted_detected) {
      if (s_missing_ms >= TARGET_LOCK_WINDOW_MS) ResumeSearch();
    } else if (vision->distance_cm <=
               (s_reverse_head ? BLACK_AREA_ARRIVE_CM : COLLECT_DISTANCE_CM)) {
      Enter(s_reverse_head ? MOTION_STATE_BLACK_AREA_PROCESS
                          : MOTION_STATE_BRUSH_COLLECT);
    } else {
      LogicalDrive(TRACKING_LINEAR_SPEED_CM_S * TRACKING_SLOW_RATIO,
        BlockAlignment_GetAngularCorrection(vision->x_offset_px), out);
    }
    break;

  case MOTION_STATE_BRUSH_COLLECT:
    Motion_StopOutput(out);
#if OBJECT_APPROACH_TEST_MODE
    if (s_action_step == 0U) {
      out->brush_cycle = 1U;
      s_action_step = 1U;
    } else if (s_action_step == 1U &&
               ActuatorServos_IsBusy(ACTUATOR_SERVO_BRUSH)) {
      s_action_step = 2U;
    } else if (s_action_step == 2U &&
               !ActuatorServos_IsBusy(ACTUATOR_SERVO_BRUSH)) {
      s_collected++;
      Enter(MOTION_STATE_SEARCH_CONTINUE);
    }
    if (s_state_ms >= BRUSH_TIMEOUT_MS) {
      Enter(MOTION_STATE_SEARCH_CONTINUE);
    }
#else
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
#endif
    break;

  case MOTION_STATE_SEARCH_CONTINUE:
    s_missing_ms = 0U;
    Enter(MOTION_STATE_ROTATE_SEARCH);
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
         * 物块阶段：实体车头就是逻辑车头，实体车头转到最近墙反方向。
         * 黑区阶段：摄像头所在车尾成为逻辑车头，中央超声仍朝实体车头；
         * 为让逻辑车头背离最近墙且超声能够测到身后墙，实体车头应朝向
         * 最近墙本身。
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
uint8_t MotionStrategy_GetCollectedCount(void) { return s_collected; }
uint8_t MotionStrategy_IsReverseHead(void) { return s_reverse_head; }
