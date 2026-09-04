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
 */
static float s_return_bearing_rad;
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
  s_return_locked = 0U;
  s_rotate_in_place = 0U;
}

void HomeTrajectory_SetOrigin(float heading_rad)
{
  s_x_cm = 0.0f;
  s_y_cm = 0.0f;
  s_home_heading = heading_rad;
  s_prev_relative_heading = 0.0f;
  s_origin_set = 1U;
  /* 换了原点, 上一趟锁定的方位角作废。 */
  s_return_locked = 0U;
  s_rotate_in_place = 0U;
}

/* 进 HOME_FOLLOW 时调用: 锁定此刻朝向原点的方位角, 作为整段直行的目标。 */
void HomeTrajectory_BeginReturn(void)
{
  s_return_bearing_rad = atan2f(-s_y_cm, -s_x_cm);
  s_return_locked = 1U;
  s_rotate_in_place = 0U;
}

/* 把锁定的方位角旋转 delta_rad (负=右旋)。遇墙退避后调用, 见 §12.2。 */
void HomeTrajectory_NudgeReturnBearing(float delta_rad)
{
  if (!s_return_locked) return;
  s_return_bearing_rad = WrapPi(s_return_bearing_rad + delta_rad);
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
  if (!s_return_locked) HomeTrajectory_BeginReturn();

  err = WrapPi(s_return_bearing_rad - RelativeHeading(heading_rad));

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
