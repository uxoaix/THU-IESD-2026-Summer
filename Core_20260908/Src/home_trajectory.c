#include "home_trajectory.h"
#include "motion_config.h"
#include <math.h>

#define PI_F 3.14159265358979f

static float s_x_cm;
static float s_y_cm;
static float s_home_heading;
static float s_prev_relative_heading;
static uint8_t s_origin_set;

/*
 * 返航方位角: 进 HOME_FOLLOW 时锁一次, 之后只由 NudgeReturnBearing 改。
 *
 * 为什么锁定而不每周期重算 atan2(-y,-x): 里程计的 (x,y) 本身在漂, 每周期重算
 * 会让目标方向跟着漂动量一起抖; 更要紧的是遇墙退避时车被推离原航线, 重算出
 * 的新方位角会把车直接拧回墙那边, 绕不出去。锁定后退避只需把这个角右旋一档,
 * 车就走上一条确定的新航线。
 *
 * 这个角只表达几何, 活在里程计坐标系里 (+x = 建系时车头方向), 供遥测 bear_deg
 * 用; 角度闭环不直接吃它, 见下面 s_return_imu_target。
 */
static float s_return_bearing_rad;
/*
 * 角度闭环真正追的目标: 一个绝对的纯 IMU 航向读数。
 *
 * 锁定那一刻把几何方位角折算成"还差多少角度没转到", 加在当时的 IMU 读数上,
 * 之后每周期只做 WrapPi(target - imu_now) —— 闭环从此只用 IMU 的增量。
 *
 * 这样拆的理由是两条航向的零点对不上: s_home_heading 记的是建系时的融合航向,
 * 而 IMU 航向的零点在上次 ResetHeading (开机/SET_HOME)。第一次卸货后走的是
 * MoveOriginHere(), 它换原点却不重置航向, 两条累加器又各过各的标度系数
 * (FUSION_YAW_SCALE 0.85 / IMU_YAW_SCALE 1.0), 全场累计转角下来能差出几十度。
 * 只在锁定这一个瞬间做一次跨系折算, 之后全走增量, 分歧量就无从进入误差。
 *
 * 不做 WrapPi: IMU 航向是单调累加量, 目标跟着它走, 归一化交给求差那一步。
 */
static float s_return_imu_target;
static uint8_t s_return_locked;
/* 原地转迟滞: 1=正在原地转 (不带前进分量)。见 HOME_ROTATE_IN_PLACE_DEG。 */
static uint8_t s_rotate_in_place;

static float AbsF(float value) { return value < 0.0f ? -value : value; }

static float WrapPi(float angle)
{
  while (angle >  PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

static float ClampAngular(float rate)
{
  if (rate >  HOME_RETURN_MAX_ANGULAR_RAD_S) return  HOME_RETURN_MAX_ANGULAR_RAD_S;
  if (rate < -HOME_RETURN_MAX_ANGULAR_RAD_S) return -HOME_RETURN_MAX_ANGULAR_RAD_S;
  return rate;
}

static int16_t RoundClampI16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static float RelativeHeading(float heading_rad)
{
  return heading_rad - s_home_heading +
         DEG_TO_RAD(HOME_HEADING_OFFSET_DEG);
}

void HomeTrajectory_Init(void)
{
  s_x_cm = 0.0f;
  s_y_cm = 0.0f;
  s_home_heading = 0.0f;
  s_prev_relative_heading = 0.0f;
  s_origin_set = 0U;
  s_return_bearing_rad = 0.0f;
  s_return_imu_target = 0.0f;
  s_return_locked = 0U;
  s_rotate_in_place = 0U;
}

/*
 * 建立坐标系。right_offset_cm = 原点相对车当前位置的右向偏移:
 * 传 0 就是"车脚下即原点"; 传正值表示原点在车右侧那么远, 于是车的坐标记成
 * (0, +offset) —— 因为 +y 是设系航向的左侧, 原点在右就等于车在原点的左边。
 */
void HomeTrajectory_SetOrigin(float heading_rad, float right_offset_cm)
{
  s_x_cm = 0.0f;
  s_y_cm = right_offset_cm;
  s_home_heading = heading_rad;
  s_prev_relative_heading = 0.0f;
  s_origin_set = 1U;
  /* 换了原点, 上一趟锁定的方位角作废。 */
  s_return_locked = 0U;
  s_rotate_in_place = 0U;
}

/*
 * 进 HOME_FOLLOW 时调用: 锁定此刻朝向原点的方位角, 作为整段直行的目标。
 *
 * 跨系折算只在这里做一次: WrapPi(方位角 - 当前相对航向) 是"从现在的车头方向
 * 还要转多少才对着家", 这个差值两边同在融合航向系里, 相减后系统性偏差抵消;
 * 把它加到当时的 IMU 读数上, 就得到一个可以纯靠 IMU 增量去追的绝对目标。
 */
void HomeTrajectory_BeginReturn(float heading_rad, float imu_heading_rad)
{
  s_return_bearing_rad = atan2f(-s_y_cm, -s_x_cm);
  s_return_imu_target = imu_heading_rad +
    WrapPi(s_return_bearing_rad - RelativeHeading(heading_rad));
  s_return_locked = 1U;
  s_rotate_in_place = 0U;
}

/* 把锁定的方位角旋转 delta_rad (负=右旋)。遇墙退避后调用, 见 §12.2。 */
void HomeTrajectory_NudgeReturnBearing(float delta_rad)
{
  if (!s_return_locked) return;
  /* 几何角 (遥测用) 和闭环目标 (IMU 系) 要同步旋同一个量, 否则 bear_deg 和车
   * 实际在追的方向会越退避差得越远。 */
  s_return_bearing_rad = WrapPi(s_return_bearing_rad + delta_rad);
  s_return_imu_target += delta_rad;
  /* 目标刚跳了一档, 重新判定要不要先原地转。 */
  s_rotate_in_place = 0U;
}

uint8_t HomeTrajectory_IsOriginSet(void)
{
  return s_origin_set;
}

void HomeTrajectory_Update(float linear_cm_s, float heading_rad, float dt_s)
{
  float relative_heading;
  float avg_heading;

  if (!s_origin_set || dt_s <= 0.0f) return;

  relative_heading = RelativeHeading(heading_rad);
  avg_heading = (relative_heading + s_prev_relative_heading) * 0.5f;
  s_x_cm += linear_cm_s * cosf(avg_heading) * dt_s;
  s_y_cm += linear_cm_s * sinf(avg_heading) * dt_s;
  s_prev_relative_heading = relative_heading;
}

int16_t HomeTrajectory_GetXcm(void) { return RoundClampI16(s_x_cm); }
int16_t HomeTrajectory_GetYcm(void) { return RoundClampI16(s_y_cm); }

float HomeTrajectory_GetDistanceCm(void)
{
  return sqrtf(s_x_cm * s_x_cm + s_y_cm * s_y_cm);
}

/*
 * 直线返航段的每周期指令。
 *
 * @return 1 = 已进到 HOME_ARRIVAL_RADIUS_CM 内, 直线段可以结束 (此时不输出
 *             动作, 由调用方停车); 0 = 继续走, linear/angular 已填好。
 *
 * 航向偏差大就原地转 (linear=0), 偏差进到一半才边走边修 —— 单阈值会在阈值
 * 附近反复"停下原地转 / 起步前进", 所以用 s_rotate_in_place 做迟滞。
 */
uint8_t HomeTrajectory_GetReturnCommand(float heading_rad,
                                        float imu_heading_rad,
                                        float *linear_cm_s,
                                        float *angular_rad_s)
{
  float err;
  const float arrival_sq = HOME_ARRIVAL_RADIUS_CM * HOME_ARRIVAL_RADIUS_CM;

  if (!linear_cm_s || !angular_rad_s) return 0U;
  *linear_cm_s = 0.0f;
  *angular_rad_s = 0.0f;
  if (!s_origin_set) return 0U;

  if ((s_x_cm * s_x_cm + s_y_cm * s_y_cm) <= arrival_sq) return 1U;

  /* 没锁过就地锁一次, 防止调用方漏调 BeginReturn 时方位角是 0 (指向 +x)。 */
  if (!s_return_locked) HomeTrajectory_BeginReturn(heading_rad, imu_heading_rad);

  /* 闭环只看 IMU 增量: 目标和当前读数同一个累加器, 零点自然对齐。 */
  err = WrapPi(s_return_imu_target - imu_heading_rad);

  if (s_rotate_in_place) {
    /* 已进到阈值一半才退出原地转, 留出迟滞带。 */
    if (AbsF(err) <= DEG_TO_RAD(HOME_ROTATE_IN_PLACE_DEG) * 0.5f) {
      s_rotate_in_place = 0U;
    }
  } else if (AbsF(err) > DEG_TO_RAD(HOME_ROTATE_IN_PLACE_DEG)) {
    s_rotate_in_place = 1U;
  }

  *angular_rad_s = ClampAngular(HOME_RETURN_HEADING_KP * err);
  *linear_cm_s = s_rotate_in_place ? 0.0f : HOME_RETURN_SPEED_CM_S;
  return 0U;
}

/*
 * 遥测 bear_deg 的来源: 返航直线段该朝的方向, 折算回融合航向坐标系。
 *
 * 锁定后报锁定值 (含退避累计的右旋量), 那才是车正在追的目标; 没锁定时报实时
 * 的"当前位置指向原点"。
 *
 * 为什么要加回 s_home_heading: 内部方位角是相对原点航向存的, 而每轮卸货会
 * 用 MoveOriginHere 换原点, 原点航向就跟着变。不折算的话 bear_deg 和 hdg_deg
 * 从第二轮起零点不同, 两者相减得不到真正的航向偏差。折算之后调用方只要再减
 * 同一个归零点, 就能直接把两个数相减看车对没对准家。
 */
float HomeTrajectory_GetReturnBearingRad(void)
{
  float rel = s_return_locked ? s_return_bearing_rad
                              : atan2f(-s_y_cm, -s_x_cm);
  return rel + s_home_heading - DEG_TO_RAD(HOME_HEADING_OFFSET_DEG);
}
