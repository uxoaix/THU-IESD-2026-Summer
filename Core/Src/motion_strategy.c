#include "motion_strategy.h"
#include "actuator_servos.h"
#include "block_alignment.h"
#include "imu_odometry.h"
#include "sensor_fusion.h"

#define PI_F 3.14159265358979f

static MotionState_t s_state;
static uint32_t s_state_ms;
static uint32_t s_missing_ms;
static uint32_t s_tracking_forward_ms;
static uint8_t s_collected;
/* 1=视觉目标切换成黑色卸货区 (返航后), 0=找红/黄物块。
 * 摄像头始终固定朝车头，车头恒为前方。 */
static uint8_t s_black_target;
static uint8_t s_wall_black;
static uint8_t s_action_step;
static float s_scan_heading;
static float s_move_cm;
static float s_prev_target[WHEEL_COUNT];
static float s_rotate_start_heading;
static float s_rotate_target_deg;
static uint8_t s_task_complete;
/* 返航直线段的退出阈值: 距原点还剩这么多就收手, 见 HOME_PARTIAL_RETURN_RATIO。 */
static float s_move_target_cm;
/* 返航开始认黑区的阈值: 距原点还剩这么多才允许视觉交接, 见
 * HOME_VISION_ENABLE_RATIO。比 s_move_target_cm 大, 所以先到这个门再到终点。 */
static float s_home_vision_cm;
/* 返航视觉交接已解锁 (过了上面那道门)。只升不降, 见 HOME_FOLLOW 里的说明。 */
static uint8_t s_home_vision_on;
/* 蓝墙退避: 记住被打断的状态，退开后原样恢复，不影响任务进度。 */
static MotionState_t s_backoff_return_state;
/* 蓝墙退避: 连续无墙时长，用于退出判据的去抖。 */
static uint32_t s_wall_clear_ms;
/* 黑区到位: 连续报到位的时长，A 帧本身无迟滞，靠这个去抖。 */
static uint32_t s_arrive_confirm_ms;
/* 已完成的升降卸货轮数，UNLOADING 内部 s_action_step 在 2~5 之间循环。 */
static uint8_t s_unload_cycles;
/* 贴墙纠缠累计计时，见 motion_config.h §12.2 WALL_STRUGGLE_TIMEOUT_MS。 */
static uint32_t s_wall_struggle_ms;
static uint8_t s_wall_struggle_active;
/* 找物块阶段的累计耗时，见 motion_config.h §7 SEARCH_GIVE_UP_MS。 */
static uint32_t s_search_ms;
/* 返航途中连续看见黑区的时长，达到 HOME_VISION_CONFIRM_MS 就交给视觉。 */
static uint32_t s_home_seen_ms;
/* 扫视方向表：每格存该朝向的蓝墙占比，见 motion_config.h §12.1。 */
static uint8_t s_scan_pct[SCAN_SECTOR_COUNT];
static float s_scan_ref_heading;  /* 0 号格对应的绝对航向 */
static uint8_t s_scan_table_valid;
/* 本次换位选定的目标航向。进 SEARCH_RELOCATE 时算一次，途中被退避打断
 * 重新对准时沿用同一个目标，不会因为车身转过了就改主意。 */
static float s_relocate_heading;
static uint8_t s_relocate_valid;

static float AbsF(float x) { return x < 0.0f ? -x : x; }

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

/*
 * 行进中的二次视觉对准：
 * 每个运动周期重新计算水平偏差。偏差越大，前进速度越低、转向修正越强；
 * 回到图像中心后恢复正常直行速度，避免高速前进时修正量不足。
 */
static int16_t GetAlignmentError(int16_t x_offset_px)
{
  return (int16_t)(x_offset_px - TRACKING_CENTER_OFFSET_PX);
}

/*
 * 原地对准专用的静摩擦下限，见 motion_config.h §9 的说明。
 * 只抬幅值不改方向；边走边修的 TrackTargetWhileMoving 不能用它——车已经在滚动，
 * 静摩擦早被打破，小修正本来就有效，再抬速只会让车沿路左右画龙。
 */
static float ApplyTurnFloor(float w)
{
  float floor_w = DEG_TO_RAD(PRE_CENTERING_MIN_TURN_DEG_S);
  if (w > 0.0f && w < floor_w) return floor_w;
  if (w < 0.0f && w > -floor_w) return -floor_w;
  return w;
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

  Motion_Drive(TRACKING_LINEAR_SPEED_CM_S *
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
  if (state == MOTION_STATE_BLACK_AREA_TRACK) {
    s_arrive_confirm_ms = 0U;
  }
  /*
   * 纠缠计时只在"退避 <-> 被打断的状态"之间往返时保留；真正换了阶段就说明
   * 状态机已经绕开了这面墙，重新给下一阶段一个完整的 10s。
   */
  if (state != MOTION_STATE_WALL_BACKOFF && state != s_backoff_return_state) {
    s_wall_struggle_ms = 0U;
    s_wall_struggle_active = 0U;
  }
}

static void ResumeSearch(void)
{
  s_missing_ms = 0U;
  Enter(s_wall_black ? MOTION_STATE_BLACK_AREA_SEARCH
                     : MOTION_STATE_ROTATE_SEARCH);
}

/*
 * 顺时针定角旋转：蓝牙 ROTATE180/ROTATE360 与返航掉头共用一套判据。
 *
 * 融合航向是单调累加量、不做 ±180°归一化，因此直接用"起始航向 - 当前航向"
 * 累计已转过的角度。相比"归一化到 ±180°的目标误差"这种写法，这样在边界上不会
 * 因为噪声让误差在 +π/-π 之间翻转、导致车来回反向摆动，360°以上的整圈旋转
 * 也能沿用同一判据。
 */
static void BeginRotateCw(float heading, float target_deg)
{
  s_rotate_start_heading = heading;
  s_rotate_target_deg = target_deg;
}

static uint8_t RotateCwStep(float heading, float speed_deg_s, float tol_deg,
                            uint32_t timeout_ms, MotionCommand_t *out)
{
  float turned_deg = (s_rotate_start_heading - heading) * 180.0f / PI_F;
  float remain_deg = s_rotate_target_deg - turned_deg;

  if (remain_deg <= tol_deg || s_state_ms >= timeout_ms) {
    Motion_StopOutput(out);
    return 1U;
  }
  /* 接近目标时减速，压低惯性过冲。 */
  if (remain_deg <= BT_ROTATE_SLOW_WINDOW_DEG &&
      speed_deg_s > BT_ROTATE_SLOW_SPEED_DEG_S) {
    speed_deg_s = BT_ROTATE_SLOW_SPEED_DEG_S;
  }
  Motion_Rotate(-DEG_TO_RAD(speed_deg_s), out);
  return 0U;
}

/*
 * 蓝墙退避判定，车上唯一的障碍处理入口。
 *
 * 返回 1 表示已停车并转入 WALL_BACKOFF，调用方应立即结束本周期的动作输出；
 * 退避结束后自动回到被打断的状态，任务进度不丢。
 *
 * 只在"朝着未知方向前进"的状态里调用；物块和黑区的最后接近段必须绕过它，
 * 否则目标本身会被当成障碍物，永远收不到货也卸不了货。
 */
static uint8_t BackoffIfWallSeen(const VisionWallData_t *vwall,
                                 MotionCommand_t *out)
{
  if (!vwall->valid || !vwall->blocked) return 0U;

  Motion_StopOutput(out);
  s_backoff_return_state = s_state;
  s_wall_clear_ms = 0U;
  s_wall_struggle_active = 1U;  /* 第一次遇墙起计时，后续退避不重置。 */
  Enter(MOTION_STATE_WALL_BACKOFF);
  return 1U;
}

/* 贴墙纠缠已超时：宿主状态该放弃当前打法，走自己的兜底跳转。 */
static uint8_t WallStruggleTimeout(void)
{
  return (s_wall_struggle_active &&
          s_wall_struggle_ms >= WALL_STRUGGLE_TIMEOUT_MS) ? 1U : 0U;
}

/*
 * 距上次收集成功已超预算，放弃本轮返航；每一轮都生效 (包括第一轮)。
 * 代价是第一轮若因视觉链路故障收不到物块，车会带着空斗返航跑完剩下的流程；
 * 换来的是任何一轮都不会卡在搜索或反复对准里耗完全场时间。
 */
static uint8_t SearchGiveUp(void)
{
  return (s_search_ms >= SEARCH_GIVE_UP_MS) ? 1U : 0U;
}

/*
 * 归一化到 (-pi, pi]。融合航向是单调累加量、不做归一化，要对准一个记录下来的
 * 绝对航向就必须先折回最短转向，否则会顺着累计圈数的方向一直转下去。
 */
static float WrapPi(float a)
{
  while (a > PI_F) a -= 2.0f * PI_F;
  while (a < -PI_F) a += 2.0f * PI_F;
  return a;
}

/* 开始新一轮扫视记录：以当前航向作为 0 号格的基准，清空全表。 */
static void ScanTableReset(float heading)
{
  uint8_t i;
  for (i = 0U; i < SCAN_SECTOR_COUNT; i++) s_scan_pct[i] = SCAN_SECTOR_EMPTY;
  s_scan_ref_heading = heading;
  s_scan_table_valid = 0U;
}

/* 把当前朝向看到的蓝墙占比记进对应格，同格取最小值以压掉单帧误检偏大。 */
static void ScanTableRecord(const VisionWallData_t *vwall, float heading)
{
  float rel_deg;
  uint8_t idx;
  uint8_t pct;

  if (!vwall->valid) return;

  /* 航向是单调累加量，先折回 [0,360) 才能落到格子里。 */
  rel_deg = (heading - s_scan_ref_heading) * 180.0f / PI_F;
  while (rel_deg < 0.0f) rel_deg += 360.0f;
  while (rel_deg >= 360.0f) rel_deg -= 360.0f;

  idx = (uint8_t)(rel_deg / SCAN_SECTOR_DEG);
  if (idx >= SCAN_SECTOR_COUNT) idx = SCAN_SECTOR_COUNT - 1U;

  pct = (vwall->fill_pct > 100U) ? 100U : vwall->fill_pct;
  if (s_scan_pct[idx] == SCAN_SECTOR_EMPTY || pct < s_scan_pct[idx]) {
    s_scan_pct[idx] = pct;
  }
  s_scan_table_valid = 1U;
}

/*
 * 从扫视表里挑换位航向：先求全表最小占比，再取占比最接近"最小值+offset_pct"
 * 的格，返回该格中心对应的绝对航向。offset_pct=0 就是挑最空旷方向。
 * 返回 0 表示表里没有有效数据，调用方应退化成沿当前朝向直行。
 */
static uint8_t ScanTablePick(uint8_t offset_pct, float *heading_out)
{
  uint8_t i;
  uint8_t min_pct = 100U;
  uint16_t target;
  uint8_t best_idx = 0U;
  uint16_t best_err = 0xFFFFU;

  if (!s_scan_table_valid) return 0U;

  for (i = 0U; i < SCAN_SECTOR_COUNT; i++) {
    if (s_scan_pct[i] != SCAN_SECTOR_EMPTY && s_scan_pct[i] < min_pct) {
      min_pct = s_scan_pct[i];
    }
  }
  target = (uint16_t)min_pct + (uint16_t)offset_pct;
  if (target > 100U) target = 100U;

  for (i = 0U; i < SCAN_SECTOR_COUNT; i++) {
    uint16_t err;
    if (s_scan_pct[i] == SCAN_SECTOR_EMPTY) continue;
    err = (s_scan_pct[i] > target) ? (uint16_t)(s_scan_pct[i] - target)
                                   : (uint16_t)(target - s_scan_pct[i]);
    if (err < best_err) {
      best_err = err;
      best_idx = i;
    }
  }
  if (best_err == 0xFFFFU) return 0U;

  /*
   * 取格子中心。这里加出来的绝对值可能和当时的原始航向差整数圈，但消费端
   * 用 WrapPi 取最短转向，同一方向的不同圈数表示是等价的。
   */
  *heading_out = s_scan_ref_heading +
                 DEG_TO_RAD(((float)best_idx + 0.5f) * SCAN_SECTOR_DEG);
  return 1U;
}

void MotionStrategy_Init(void)
{
  uint8_t i;
  s_state = MOTION_STATE_INIT;
  s_state_ms = s_missing_ms = 0U;
  s_tracking_forward_ms = 0U;
  s_collected = s_wall_black = 0U;
  s_black_target = 0U;
  s_action_step = 0U;
  s_scan_heading = 0.0f;
  s_move_cm = 0.0f;
  s_rotate_start_heading = 0.0f;
  s_rotate_target_deg = 0.0f;
  s_task_complete = 0U;
  s_move_target_cm = 0.0f;
  s_home_vision_cm = 0.0f;
  s_home_vision_on = 0U;
  s_backoff_return_state = MOTION_STATE_ROTATE_SEARCH;
  s_wall_clear_ms = 0U;
  s_arrive_confirm_ms = 0U;
  s_unload_cycles = 0U;
  s_wall_struggle_ms = 0U;
  s_wall_struggle_active = 0U;
  s_search_ms = 0U;
  s_home_seen_ms = 0U;
  ScanTableReset(0.0f);
  s_relocate_heading = 0.0f;
  s_relocate_valid = 0U;
  for (i = 0U; i < WHEEL_COUNT; i++) s_prev_target[i] = 0.0f;
  ImuOdometry_Init();
}

void MotionStrategy_Stop(void)
{
  s_state = MOTION_STATE_STANDBY;
  s_state_ms = 0U;
  s_action_step = 0U;
  s_missing_ms = 0U;
  s_wall_struggle_ms = 0U;
  s_wall_struggle_active = 0U;
  s_search_ms = 0U;
  s_home_seen_ms = 0U;
  s_relocate_valid = 0U;
  s_scan_table_valid = 0U;
}

void MotionStrategy_RequestRotateCw(float target_deg)
{
  BeginRotateCw(ImuOdometry_GetHeadingRad(), target_deg);
  s_task_complete = 0U;
  Enter(MOTION_STATE_BT_ROTATE);
}

uint8_t MotionStrategy_TakeTaskComplete(void)
{
  if (!s_task_complete) return 0U;
  s_task_complete = 0U;
  return 1U;
}

void MotionStrategy_Update(const VisionData_t *vision,
                           const WheelFeedback_t *wheels,
                           const VisionWallData_t *vwall,
                           const VisionArrivalData_t *arrival,
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

  ImuOdometry_Update(wheels, dt);
  /*
   * 全车只有这一个航向来源: 融合航向 (编码器 + IMU, 见 sensor_fusion)。
   * 曾经给定角机动 (蓝牙定角旋转、返航对准/直行/掉头) 单独走过一条纯 IMU
   * 航向, 想绕开原地旋转时的四轮打滑; 实测纯 IMU 更差 —— 模组 yaw 多报约
   * 20%, 且该比例随转速/负载变化, 一轮全场下来累积误差比打滑还大, 故撤回。
   * 纯 IMU 航向现在只留在遥测里 (ImuOdometry_GetImuYawDeg) 用于对比诊断。
   */
  heading = ImuOdometry_GetHeadingRad();
  s_state_ms += elapsed_ms;
  if (s_wall_struggle_active) s_wall_struggle_ms += elapsed_ms;
  wanted_detected = vision->detected &&
    ((s_black_target && vision->object_type == BLACK_AREA_OBJECT_TYPE) ||
     (!s_black_target && vision->object_type != BLACK_AREA_OBJECT_TYPE));
  if (wanted_detected) s_missing_ms = 0U;
  else s_missing_ms += elapsed_ms;

  /*
   * 距上次滚刷收集成功的时长，见 motion_config.h §7 SEARCH_GIVE_UP_MS。
   * 只在 BRUSH_COLLECT 走完整套动作、s_collected 真的加一时才清零——衡量的
   * 是"有没有真的收到物块"，不是"有没有看见物块"。看见就清零的话，车反复
   * 盯着一个够不到的物块、或者每次追踪都在最后丢失，计时会被无限推迟。
   * 收集阶段全程累加，所以卡在 PRE_CENTERING/TARGET_TRACKING 的往复里也会
   * 到点，不像只在搜索态计时那样漏掉这类卡死。
   * 三个例外不计时: 黑区/返航阶段(s_black_target)找的是卸货区不是物块;
   * STANDBY 与 BT_ROTATE 是手动/蓝牙态, 在那停多久都不该算进任务预算;
   * INIT 只过一个周期, 顺带排除。
   */
  if (s_black_target) {
    s_search_ms = 0U;
  } else if (s_state != MOTION_STATE_INIT &&
             s_state != MOTION_STATE_STANDBY &&
             s_state != MOTION_STATE_BT_ROTATE) {
    s_search_ms += elapsed_ms;
  }

  /*
   * 返航途中的视觉交接确认。放在 switch 之前而不是各状态里面，是为了让
   * "被蓝墙退避打断"自动等价于"连续性中断"：退避期间 s_state 是
   * WALL_BACKOFF，走 else 分支清零，不必再往 Enter() 里加特例。
   * s_home_vision_on 是"已走过 HOME_VISION_ENABLE_RATIO"的门：解锁前纯里程计
   * 返航，看见黑区也不累加，免得认成对方的卸货区。
   * 除这道门之外没有别的距离门限，误检全靠 OpenMV 的 BLACK_PIXELS_THRESHOLD
   * 把关，见 motion_config.h 里 HOME_VISION_CONFIRM_MS 的说明。
   */
  if (s_state == MOTION_STATE_HOME_FOLLOW &&
      s_home_vision_on && s_black_target && wanted_detected) {
    s_home_seen_ms += elapsed_ms;
  } else {
    s_home_seen_ms = 0U;
  }

  switch (s_state) {
  case MOTION_STATE_STANDBY:
    Motion_StopOutput(out);
    break;

  case MOTION_STATE_BT_ROTATE:
    if (RotateCwStep(heading, BT_ROTATE_SPEED_DEG_S, BT_ROTATE_TOL_DEG,
                     BT_ROTATE_TIMEOUT_MS, out)) {
      s_task_complete = 1U;
      Enter(MOTION_STATE_STANDBY);
    }
    break;

  case MOTION_STATE_WALL_BACKOFF:
    /*
     * 蓝墙退避：后退 + 右转。摄像头固定朝车头，所以"远离墙"就是物理后退。
     * 返航段和其他状态用两套退出判据，见下面各自的注释。
     */
    if (s_backoff_return_state == MOTION_STATE_HOME_FOLLOW) {
      /*
       * 返航专用的定量退避：后退固定一段并右偏 HOME_BACKOFF_TURN_DEG，不等墙
       * 从视野里消失——返航要的是绕过去继续走，不是原地磨到墙不见。
       * 退完必须把锁定的返航方位角一起右旋同样角度，否则回到 HOME_FOLLOW
       * 第一个周期就按原方位角左转拧回来，又撞上同一面墙。
       * 墙还在就再触发一次，自然形成一档一档的绕行；累计绕不出去时由
       * HOME_FOLLOW 里的 WallStruggleTimeout() 放弃返航转去找黑区。
       */
      if (s_state_ms >= HOME_BACKOFF_DURATION_MS) {
        Motion_StopOutput(out);
        ImuOdometry_NudgeReturnBearing(-DEG_TO_RAD(HOME_BACKOFF_TURN_DEG));
        Enter(MOTION_STATE_HOME_FOLLOW);
      } else {
        Motion_Drive(-WALL_BACKOFF_SPEED_CM_S,
                     -DEG_TO_RAD(HOME_BACKOFF_TURN_DEG_S), out);
      }
      break;
    }

    /*
     * 通用退避：一直后退右转，直到 OpenMV 连续 WALL_BACKOFF_CLEAR_MS 不再报
     * 有墙。搜索类状态没有既定航线，转到哪算哪，所以以"墙不见"为准。
     * 链路断了 (valid=0) 时 blocked 被清 0，会走正常退出，不会一直傻退。
     */
    if (vwall->blocked) s_wall_clear_ms = 0U;
    else s_wall_clear_ms += elapsed_ms;

    if (s_wall_clear_ms >= WALL_BACKOFF_CLEAR_MS ||
        s_state_ms >= WALL_BACKOFF_TIMEOUT_MS) {
      Motion_StopOutput(out);
      if (s_backoff_return_state == MOTION_STATE_SEARCH_RELOCATE) {
        /*
         * 换位途中撞墙：不回去重走这段距离。换位的目的本来就是"换个视角再
         * 搜"，既然这个方向被墙拦住，退避后回原地扫视重新挑方向，比沿同一
         * 方向再撞一次有意义；此刻车也已经离开原扫视点了，重新扫一圈能看到
         * 新画面。ResumeSearch() 按 s_wall_black 决定回物块扫视还是黑区扫视。
         */
        s_relocate_valid = 0U;
        ResumeSearch();
      } else {
        Enter(s_backoff_return_state);
        /*
         * 退避只是插进来的一段，回到扫视时不能把进度清零：贴着墙时会反复
         * 触发退避，每次都重锁起始航向的话，转满一圈的判据永远达不到，车就卡
         * 在原地转+退避的循环里出不去；扫视方向表也会被反复清空，选不出方向。
         */
        if (s_backoff_return_state == MOTION_STATE_ROTATE_SEARCH ||
            s_backoff_return_state == MOTION_STATE_BLACK_AREA_SEARCH) {
          s_action_step = 1U;
        }
      }
    } else {
      Motion_Drive(-WALL_BACKOFF_SPEED_CM_S,
                   -DEG_TO_RAD(WALL_BACKOFF_TURN_DEG_S), out);
    }
    break;

  case MOTION_STATE_INIT: {
    FusionState_t fusion;
    Motion_StopOutput(out);
    ImuOdometry_GetFusionState(&fusion);
    if (!ImuOdometry_IsOriginSet()) {
      if (!fusion.imu_alive && s_state_ms < HOME_ORIGIN_WAIT_MS) {
        break;
      }
      ImuOdometry_SetHome();
      /* SetHome 会重置航向, 要重取, 别把复位前的值带进本周期。 */
      heading = ImuOdometry_GetHeadingRad();
    }
    s_black_target = 0U;
    s_wall_black = 0U;
    s_collected = 0U;
    s_missing_ms = 0U;
    Enter(MOTION_STATE_ROTATE_SEARCH);
    break;
  }

  case MOTION_STATE_ROTATE_SEARCH:
    /* 收满，或距上次收集成功已超预算(收不动了)，都转返航。 */
    if (s_collected >= TOTAL_OBJECTS_TO_COLLECT || SearchGiveUp()) {
      Enter(MOTION_STATE_HOME_PREPARE);
      break;
    }
    if (s_action_step == 0U) {
      /* 进本状态时锁定起始航向，用于判断是否已转满一圈。 */
      s_scan_heading = heading;
      /* 新一轮扫视重建方向表；退避返回时 step 已是 1，表不会被清。 */
      ScanTableReset(heading);
      s_action_step = 1U;
    }
    ScanTableRecord(vwall, heading);
#if OBJECT_APPROACH_TEST_MODE
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (BackoffIfWallSeen(vwall, out)) {
      /* 已转入退避，本周期不再输出其他动作。 */
    } else if (AbsF(heading - s_scan_heading) >=
                 DEG_TO_RAD(SEARCH_FULL_CIRCLE_DEG) ||
               s_state_ms >= SEARCH_NO_TARGET_TIMEOUT_MS ||
               WallStruggleTimeout()) {
      /*
       * 转满一圈仍未发现目标：原地再转下去只会看到同样的画面，
       * 必须换个位置。航向判据为主，时间判据只是 IMU 异常时的兜底。
       */
      s_wall_black = 0U;
      s_move_cm = 0.0f;
      /* 找物块：朝最空旷方向换位，离墙最远。 */
      s_relocate_valid = ScanTablePick(0U, &s_relocate_heading);
      Enter(MOTION_STATE_SEARCH_RELOCATE);
    } else {
      /* 小半径扫视：转速压低以留足单帧识别时间，见 §12 注释。 */
      Motion_Drive(SEARCH_SWEEP_LINEAR_CM_S,
                   -DEG_TO_RAD(SEARCH_SWEEP_ANGULAR_DEG_S), out);
    }
#else
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (BackoffIfWallSeen(vwall, out)) {
      /* 已转入退避。 */
    } else if (s_missing_ms >= SEARCH_NO_TARGET_TIMEOUT_MS) {
      s_wall_black = 0U;
      s_move_cm = 0.0f;
      s_relocate_valid = ScanTablePick(0U, &s_relocate_heading);
      Enter(MOTION_STATE_SEARCH_RELOCATE);
    } else {
      Motion_Rotate(-DEG_TO_RAD(SEARCH_ROTATION_SPEED_DEG_S), out);
    }
#endif
    break;

  case MOTION_STATE_SEARCH_RELOCATE:
    /*
     * 换位：step0 原地对准进本状态时选好的航向，step1 沿它开环直行一段。
     * 车上没有测距传感器，蓝墙占比是唯一能判断"哪边空"的依据。航向由调用方
     * 从扫视方向表里挑好（找物块挑最空旷，找黑区挑稍微有墙的），见
     * motion_config.h §12.1。表无效时 aim_err 恒为 0，第一个周期就进 step1，
     * 退化成沿当前朝向直行。
     */
    if (wanted_detected) {
      Enter(MOTION_STATE_PRE_CENTERING);
    } else if (SearchGiveUp()) {
      /* 换位途中到点也立刻返航，不必等走完 SEARCH_RELOCATE_DIST_CM。 */
      Motion_StopOutput(out);
      Enter(MOTION_STATE_HOME_PREPARE);
    } else if (BackoffIfWallSeen(vwall, out)) {
      /* 已转入退避。 */
    } else if (WallStruggleTimeout()) {
      /* 一直贴着墙，对不准也走不出去：放弃这次换位，回搜索。 */
      Motion_StopOutput(out);
      s_relocate_valid = 0U;
      ResumeSearch();
    } else if (s_action_step == 0U) {
      float aim_err =
        s_relocate_valid ? WrapPi(s_relocate_heading - heading) : 0.0f;
      if (AbsF(aim_err) <= DEG_TO_RAD(RELOCATE_AIM_TOL_DEG) ||
          s_state_ms >= RELOCATE_AIM_TIMEOUT_MS) {
        Motion_StopOutput(out);
        /* 对准过程可能被退避打断重来，直行里程和超时都从这里重新起算。 */
        s_move_cm = 0.0f;
        s_state_ms = 0U;
        s_action_step = 1U;
      } else {
        Motion_Rotate(aim_err > 0.0f ? DEG_TO_RAD(RELOCATE_AIM_SPEED_DEG_S)
                                     : -DEG_TO_RAD(RELOCATE_AIM_SPEED_DEG_S),
                      out);
      }
    } else {
      s_move_cm += AbsF(SensorFusion_GetLinearVelocity()) * dt;
      if (s_move_cm >= SEARCH_RELOCATE_DIST_CM ||
          s_state_ms >= SEARCH_RELOCATE_TIMEOUT_MS) {
        Motion_StopOutput(out);
        s_relocate_valid = 0U;
        ResumeSearch();
      } else {
        Motion_Line(SEARCH_RELOCATE_SPEED_CM_S, out);
      }
    }
    break;

  case MOTION_STATE_PRE_CENTERING:
    if (!wanted_detected || s_state_ms >= PRE_CENTERING_TIMEOUT_MS) {
      ResumeSearch();
    } else if (AbsF((float)GetAlignmentError(vision->x_offset_px)) <=
               PRE_CENTERING_EXIT_PX) {
      Enter(s_black_target ? MOTION_STATE_BLACK_AREA_TRACK
                           : MOTION_STATE_TARGET_TRACKING);
    } else if (s_black_target) {
      /* 黑区面积大，边对准边前进，避免纯原地转圈超时回搜索。 */
      TrackTargetWhileMoving(vision, out);
    } else {
      Motion_Rotate(ApplyTurnFloor(
        BlockAlignment_GetAngularCorrection(
          GetAlignmentError(vision->x_offset_px))), out);
    }
    break;

  case MOTION_STATE_TARGET_TRACKING:
    if (!wanted_detected) {
      /*
       * 普通物块已经完成对齐并实际向前追踪后，目标消失通常意味着
       * 物块被车体/滚刷遮挡，小车此刻已停止；将该停止事件视为追踪完成。
       */
      if (s_tracking_forward_ms >= TRACKING_COMPLETE_MIN_FORWARD_MS &&
          s_missing_ms >= TRACKING_COMPLETE_LOSS_MS) {
        Enter(MOTION_STATE_FINAL_APPROACH);
      } else if (s_missing_ms >= TARGET_LOCK_WINDOW_MS) {
        ResumeSearch();
      }
    } else if (vision->distance_cm <= COLLECT_DISTANCE_CM) {
      Enter(MOTION_STATE_FINAL_APPROACH);
    } else {
      s_tracking_forward_ms += elapsed_ms;
      TrackTargetWhileMoving(vision, out);
    }
    break;

  case MOTION_STATE_BLACK_AREA_TRACK:
    /*
     * 到位判据来自 OpenMV 的 A 帧（黑区外框占画面的面积比），不再看距离：
     * OpenMV 报的"距离"是黑区近边缘到画面底边的像素间隙，远处黑区被画面
     * 上边缘裁切时同样读出接近 0 的值，会让车一看到黑区就地掉头；面积比
     * 远则小近则大，没有这个歧义。
     * A 帧本身没有迟滞，所以要求连续 BLACK_ARRIVE_CONFIRM_MS 都报到位。
     * 链路断了时 valid=0 且 arrived=0，走目标丢失分支回搜索，不会误卸货。
     * 本状态不做蓝墙退避——黑区就贴着墙，退避会让它永远卸不了货。
     */
    if (!wanted_detected) {
      if (s_missing_ms >= TARGET_LOCK_WINDOW_MS) ResumeSearch();
    } else {
      if (arrival->valid && arrival->arrived) {
        s_arrive_confirm_ms += elapsed_ms;
      } else {
        s_arrive_confirm_ms = 0U;
      }

      if (s_arrive_confirm_ms >= BLACK_ARRIVE_CONFIRM_MS) {
        BeginRotateCw(heading, HOME_TURN_AROUND_TARGET_DEG);
        Enter(MOTION_STATE_HOME_TURN_AROUND);
      } else {
        TrackTargetWhileMoving(vision, out);
      }
    }
    break;

  case MOTION_STATE_FINAL_APPROACH:
    /*
     * 视觉追踪已结束，保持最后对准方向，以追踪速度继续前进1秒，
     * 把物块送入滚刷范围。
     */
    if (s_state_ms >= FINAL_APPROACH_DURATION_MS) {
      Motion_StopOutput(out);
      Enter(MOTION_STATE_BRUSH_COLLECT);
    } else {
      Motion_Line(TRACKING_LINEAR_SPEED_CM_S * TRACKING_SLOW_RATIO, out);
    }
    break;

  case MOTION_STATE_BRUSH_COLLECT:
    /*
     * 只等滚刷走完就走，后斗升降与下一轮搜索并行。
     *
     * 能这么做是因为舵机模块是自治的：TriggerCycle 只设一次目标角并起计时，
     * 之后 AT_END → RETURNING → IDLE 由主循环里的 ActuatorServos_Run() 自己
     * 推进，与运动状态机无关。所以发完指令就能转下一轮，升降会在搜索/对准
     * 的过程中自行完成，省下原来干等后斗的约 2s。
     * cycle 标志是一次性脉冲（Update 开头统一清零），本周期设上就会被紧随
     * 其后的 ActuatorServos_Apply 消费，同周期 Enter() 不影响它生效。
     */
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
      /*
       * 发后斗指令并立即转下一轮。滚刷已经走完、物块已被扫进来，所以这里
       * 就是"收到一个"的真实节点，计数和搜索计时都在这一刻更新。
       */
      out->bucket_cycle = 1U;
      s_collected++;
      /* 唯一的清零点：滚刷成功走完才算一次真实进度。下面的超时分支不清零，
       * 那条路没收到东西。 */
      s_search_ms = 0U;
      Enter(MOTION_STATE_SEARCH_CONTINUE);
    }
    /* 兜底只覆盖滚刷段：后斗已不占用本状态的时间。 */
    if (s_state_ms >= BRUSH_TIMEOUT_MS + COLLECT_BUCKET_DELAY_MS) {
      Enter(MOTION_STATE_SEARCH_CONTINUE);
    }
    break;

  case MOTION_STATE_SEARCH_CONTINUE:
    s_missing_ms = 0U;
    Enter((s_collected >= TOTAL_OBJECTS_TO_COLLECT)
        ? MOTION_STATE_HOME_PREPARE
        : MOTION_STATE_ROTATE_SEARCH);
    break;

  case MOTION_STATE_HOME_PREPARE: {
    /*
     * 返航起步：锁定此刻朝向原点的方位角，并按当前距离算出两道门。
     * 只走全程的 HOME_PARTIAL_RETURN_RATIO，把"剩下多少距离"存进
     * s_move_target_cm 当退出阈值；余下的路交给视觉找黑区，用它消掉里程计
     * 的累计误差。
     */
    float dist = ImuOdometry_GetDistanceCm();
    Motion_StopOutput(out);
    ImuOdometry_BeginReturn();
    s_move_target_cm = dist * (1.0f - HOME_PARTIAL_RETURN_RATIO);
    s_home_vision_cm = dist * (1.0f - HOME_VISION_ENABLE_RATIO);
    /*
     * OpenMV 全程留在黑区模式，但起步一段不"认"——交接由 s_home_vision_on 单独
     * 把关，过了 1/3 才解锁。这样这一段是纯里程计返航：朝锁定的方位角开，
     * 看见黑区也不理，从而不会扎向对方的卸货区（场上两个黑区长得一样，
     * OpenMV 分不出是谁的，而起步时车在远端，最容易先看见对方那个）。
     *
     * 为什么不干脆切回物块模式：那样等于让摄像头在返航途中去找物块，没有意义;
     * 而且解锁点再切回黑区要等 OpenMV 重新稳定，白丢几帧，黑区到位帧 (A) 也
     * 只在黑区模式下发。留在黑区模式、只拦"认不认"最省事。
     * s_wall_black 照旧置 1，它决定各种兜底最终落到黑区搜索这条线上。
     */
    s_black_target = 1U;
    s_wall_black = 1U;
    s_home_vision_on = 0U;
    Enter(MOTION_STATE_HOME_FOLLOW);
    break;
  }

  case MOTION_STATE_HOME_FOLLOW: {
    /*
     * 朝锁定的方位角直行，偏差大时先原地转（迟滞判据都在
     * HomeTrajectory_GetReturnCommand 里）。出路：
     *   确认看见黑区                → 交给 BLACK_AREA_TRACK（仅 1/3 之后）；
     *   走完 3/4 或已进到 12cm 内   → BLACK_AREA_SEARCH 原地扫视；
     *   撞墙                        → 退一小段 + 方位角右旋 8°，回本状态继续
     *                                 往前拱；墙还在就再来一档，如此反复；
     *   贴墙累计到点 / 兜底超时     → BLACK_AREA_SEARCH。
     */
    float linear = 0.0f;
    float angular = 0.0f;
    uint8_t arrived = ImuOdometry_GetReturnCommand(heading, &linear, &angular);

    /*
     * 走过 HOME_VISION_ENABLE_RATIO 之后才解锁视觉交接；在此之前纯靠里程计
     * 朝家开，看见黑区也不理。
     * 只升不降：遇墙退避会把车往后推、剩余距离回涨，若跟着回锁就会在门附近
     * 反复上锁解锁，交接确认计时永远攒不满。
     */
    if (!s_home_vision_on &&
        ImuOdometry_GetDistanceCm() <= s_home_vision_cm) {
      s_home_vision_on = 1U;
    }

    if (s_home_seen_ms >= HOME_VISION_CONFIRM_MS) {
      /* 计时能攒满就说明这一刻 wanted_detected 为真，s_missing_ms 已是 0。 */
      Motion_StopOutput(out);
      Enter(MOTION_STATE_BLACK_AREA_TRACK);
    } else if (arrived ||
               ImuOdometry_GetDistanceCm() <= s_move_target_cm ||
               s_state_ms >= HOME_FOLLOW_TIMEOUT_MS ||
               (s_wall_struggle_active &&
                s_wall_struggle_ms >= HOME_WALL_GIVEUP_MS)) {
      /*
       * 直线段到此为止。贴墙计时用返航自己的 HOME_WALL_GIVEUP_MS 而不是
       * WallStruggleTimeout()：后者的 10s 是配大角度退避定的，8° 一档绕不完
       * 就会被判成卡死。到点表示一直贴着墙绕不出去，剩下的返航距离不要了，
       * 就地找黑区；s_wall_black 在 HOME_PREPARE 已置 1，BLACK_AREA_SEARCH
       * 后续的换位/重扫都会留在黑区这条线上。
       */
      Motion_StopOutput(out);
      Enter(MOTION_STATE_BLACK_AREA_SEARCH);
    } else if (BackoffIfWallSeen(vwall, out)) {
      /* 已转入退避，本周期不再输出其他动作。 */
    } else {
      Motion_Drive(linear, angular, out);
    }
    break;
  }

  case MOTION_STATE_HOME_TURN_AROUND:
    /* 融合航向闭环, 不是定时开环: 轮子打滑/空转时按时间转会差得更远。
     * 停角精度由 FUSION_YAW_SCALE 和 HOME_TURN_AROUND_TOL_DEG 共同决定。 */
    if (RotateCwStep(heading, HOME_TURN_AROUND_SPEED_DEG_S,
                     HOME_TURN_AROUND_TOL_DEG,
                     HOME_TURN_AROUND_TIMEOUT_MS, out)) {
      Enter(MOTION_STATE_HOME_DONE);
    }
    break;

  case MOTION_STATE_HOME_DONE:
    /*
     * 掉头完成后先倒一段，再进 UNLOADING 卸货序列。
     * 掉头 180°后车尾朝着黑区，此时到位判据是按车头看到的黑区面积算的，
     * 车尾很可能还压在黑区边缘上，直接卸货物块容易滚出区外。
     */
    if (s_state_ms >= HOME_BACKUP_DURATION_MS) {
      Motion_StopOutput(out);
      Enter(MOTION_STATE_UNLOADING);
    } else {
      Motion_Line(-HOME_BACKUP_SPEED_CM_S, out);
    }
    break;

  case MOTION_STATE_BLACK_AREA_SEARCH:
    if (s_action_step == 0U) {
      /* 本轮黑区扫视重建方向表；退避返回时 step 已是 1，表不会被清。 */
      ScanTableReset(heading);
      s_action_step = 1U;
    }
    ScanTableRecord(vwall, heading);
    if (wanted_detected) {
      /* 黑区搜索：检测到即开始追踪，跳过纯原地对准。 */
      Enter(MOTION_STATE_BLACK_AREA_TRACK);
    } else if (BackoffIfWallSeen(vwall, out)) {
      /* 已转入退避。 */
    } else if (s_missing_ms >= BLACK_SEARCH_TIMEOUT_MS ||
               WallStruggleTimeout()) {
      /* 原地找不到黑区：换个位置再找，s_wall_black 保证回到黑区搜索。 */
      s_wall_black = 1U;
      s_move_cm = 0.0f;
      /*
       * 找黑区不能朝最空旷方向走：卸货区贴着墙，最空旷的方向指向场地中央，
       * 朝那边走只会离黑区更远。挑占比"最小值+10"的格，即稍微有点墙的方向。
       */
      s_relocate_valid = ScanTablePick(SCAN_BLACK_OFFSET_PCT,
                                       &s_relocate_heading);
      Enter(MOTION_STATE_SEARCH_RELOCATE);
    } else {
      Motion_Rotate(-DEG_TO_RAD(SEARCH_ROTATION_SPEED_DEG_S), out);
    }
    break;

  case MOTION_STATE_UNLOADING:
    /*
     * 开门 → 升降 → 前进 0.5s → 升降 → 左前方弧线开出黑区 → 关门。
     * 门只开关一次，从开门一直开到驶出黑区之后：物块常卡在斗底或门边一次滑
     * 不出去，落斗再升起的冲击能把它抖松，而抖动只有在门开着时才有意义；
     * 开出黑区那一段的颠簸同理，所以关门放在最后而不是开出之前。
     * 单轮升降之内不留停留时间，斗到位即反向；两轮之间挪一小段，避免两次都
     * 卸在同一点、物块堆起来互相挡住出口。
     */
    Motion_StopOutput(out);
    if (s_action_step == 0U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_DOOR, DOOR_END_DEG)) {
        s_unload_cycles = 0U;
        s_action_step = 1U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 1U && s_state_ms >= UNLOAD_DOOR_OPEN_MS) {
      s_action_step = 2U;
    } else if (s_action_step == 2U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_BUCKET, BUCKET_END_DEG)) {
        s_action_step = 3U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 3U && s_state_ms >= BUCKET_LIFT_DURATION_MS) {
      s_action_step = 4U;
    } else if (s_action_step == 4U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_BUCKET, BUCKET_START_DEG)) {
        s_action_step = 5U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 5U && s_state_ms >= BUCKET_LOWER_DURATION_MS) {
      s_unload_cycles++;
      /* 还有下一轮就先挪一小段(step6)，卸完了就直接关门(step7)。 */
      s_action_step = (s_unload_cycles < UNLOAD_REPEAT_COUNT) ? 6U : 7U;
      s_state_ms = 0U;
    } else if (s_action_step == 6U) {
      /* 两轮升降之间挪位，让第二轮卸在稍微错开的位置。 */
      if (s_state_ms >= UNLOAD_MID_FORWARD_MS) {
        Motion_StopOutput(out);
        if (s_unload_cycles == 1U) {
          /*
           * 第二次卸货的落脚点定为下一轮的(0,0)：这里离真实卸货点最近。
           * 用 MoveOriginHere() 而不是 SetHome()：后者会把融合航向清零，
           * 而返航靠开机锁存的绝对航向定方向，中途清一次航向基准就失效了。
           * 位置清零本身只影响 home_x/home_y 遥测，不参与返航决策。
           */
          ImuOdometry_MoveOriginHere();
        }
        s_action_step = 2U;
      } else {
        Motion_Line(UNLOAD_EXIT_SPEED_CM_S, out);
      }
    } else if (s_action_step == 7U) {
      /*
       * 沿弧线朝左前方开出黑区：车尾还压在卸货区上，原地起转会把刚倒出来的
       * 物块扫散。带 45°左偏是为了让下一轮搜索的起始朝向与上一轮错开。
       * 这一段门仍然开着：卡在斗底或门边的物块能借开出时的颠簸继续掉出来，
       * 出了黑区再关，免得把没抖出来的物块一路带走。
       */
      if (s_state_ms >= UNLOAD_EXIT_FORWARD_MS) {
        Motion_StopOutput(out);
        s_action_step = 8U;
        s_state_ms = 0U;
      } else {
        Motion_Drive(UNLOAD_EXIT_SPEED_CM_S,
                     DEG_TO_RAD(UNLOAD_EXIT_TURN_DEG_S), out);
      }
    } else if (s_action_step == 8U) {
      if (ActuatorServos_SetAngle(ACTUATOR_SERVO_DOOR, DOOR_START_DEG)) {
        s_action_step = 9U;
        s_state_ms = 0U;
      }
    } else if (s_action_step == 9U && s_state_ms >= BUCKET_DOOR_CLOSE_MS) {
      s_black_target = 0U;
      /* 新一轮的久搜计时从这里重新起算。 */
      s_search_ms = 0U;
      Enter(MOTION_STATE_INIT);
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
}

MotionState_t MotionStrategy_GetState(void) { return s_state; }
const char *MotionStrategy_GetStateName(void)
{
  switch (s_state) {
  case MOTION_STATE_INIT:                return "INIT";
  case MOTION_STATE_STANDBY:             return "STANDBY";
  case MOTION_STATE_BT_ROTATE:           return "BT_ROTATE";
  case MOTION_STATE_WALL_BACKOFF:        return "WALL_BACKOFF";
  case MOTION_STATE_ROTATE_SEARCH:       return "ROTATE_SEARCH";
  case MOTION_STATE_SEARCH_RELOCATE:     return "SEARCH_RELOCATE";
  case MOTION_STATE_PRE_CENTERING:       return "PRE_CENTERING";
  case MOTION_STATE_TARGET_TRACKING:     return "TARGET_TRACKING";
  case MOTION_STATE_FINAL_APPROACH:      return "FINAL_APPROACH";
  case MOTION_STATE_BRUSH_COLLECT:       return "BRUSH_COLLECT";
  case MOTION_STATE_SEARCH_CONTINUE:     return "SEARCH_CONTINUE";
  case MOTION_STATE_HOME_PREPARE:        return "HOME_PREPARE";
  case MOTION_STATE_HOME_FOLLOW:         return "HOME_FOLLOW";
  case MOTION_STATE_HOME_TURN_AROUND:    return "HOME_TURN_AROUND";
  case MOTION_STATE_HOME_DONE:           return "HOME_DONE";
  case MOTION_STATE_BLACK_AREA_SEARCH:   return "BLACK_AREA_SEARCH";
  case MOTION_STATE_BLACK_AREA_TRACK:    return "BLACK_AREA_TRACK";
  case MOTION_STATE_UNLOADING:           return "UNLOADING";
  default:                               return "UNKNOWN";
  }
}
uint8_t MotionStrategy_GetCollectedCount(void) { return s_collected; }
