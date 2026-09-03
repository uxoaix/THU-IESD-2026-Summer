#include "home_trajectory.h"
#include "motion_config.h"
#include <math.h>

#define PI_F 3.14159265358979f

static float s_x_cm;
static float s_y_cm;
static float s_home_heading;
static float s_prev_relative_heading;
static float s_return_bearing_rad;
static uint8_t s_origin_set;
static uint8_t s_return_locked;
static uint8_t s_rotate_in_place;

static float AbsF(float value) { return value < 0.0f ? -value : value; }

static float WrapPi(float angle)
{
  while (angle > PI_F) angle -= 2.0f * PI_F;
  while (angle < -PI_F) angle += 2.0f * PI_F;
  return angle;
}

static int16_t RoundClampI16(float value)
{
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static float ClampAngular(float rate)
{
  if (rate > HOME_RETURN_MAX_ANGULAR_RAD_S) {
    return HOME_RETURN_MAX_ANGULAR_RAD_S;
  }
  if (rate < -HOME_RETURN_MAX_ANGULAR_RAD_S) {
    return -HOME_RETURN_MAX_ANGULAR_RAD_S;
  }
  return rate;
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
  s_return_bearing_rad = 0.0f;
  s_origin_set = 0U;
  s_return_locked = 0U;
  s_rotate_in_place = 0U;
}

void HomeTrajectory_SetOrigin(float heading_rad)
{
  s_x_cm = 0.0f;
  s_y_cm = 0.0f;
  s_home_heading = heading_rad;
  s_prev_relative_heading = 0.0f;
  s_return_bearing_rad = 0.0f;
  s_return_locked = 0U;
  s_rotate_in_place = 0U;
  s_origin_set = 1U;
}

void HomeTrajectory_BeginReturn(void)
{
  if (!s_origin_set) return;

  s_return_bearing_rad = atan2f(-s_y_cm, -s_x_cm);
  s_return_locked = 1U;
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

/*
 * 把锁定的返航方位角旋转 delta_rad (负 = 右旋)。
 * 供返航途中的绕墙用: 车右偏一个角度绕过墙之后, 方位角必须跟着偏同样的角度,
 * 否则 GetReturnCommand 下一周期就把车拧回原方位角、直接撞回同一面墙。
 * 代价是终点会偏离原点 (方位角不再指向真正的家), 但 AUTO 流程的直线段本来
 * 只走全程的 HOME_PARTIAL_RETURN_RATIO, 剩下那段交给视觉找黑区来兜。
 * 未锁定 (不在返航中) 时什么都不做: 那时方位角每周期都实时重算, 改它没意义。
 */
void HomeTrajectory_NudgeReturnBearing(float delta_rad)
{
  if (!s_return_locked) return;
  s_return_bearing_rad = WrapPi(s_return_bearing_rad + delta_rad);
}

uint8_t HomeTrajectory_GetReturnCommand(float heading_rad,
                                        float *linear_cm_s,
                                        float *angular_rad_s)
{
  float distance_sq;
  float desired_heading;
  float heading_error;
  float abs_error;
  float rate;
  const float arrival_radius_sq =
    HOME_ARRIVAL_RADIUS_CM * HOME_ARRIVAL_RADIUS_CM;

  if (linear_cm_s) *linear_cm_s = 0.0f;
  if (angular_rad_s) *angular_rad_s = 0.0f;
  if (!s_origin_set || !linear_cm_s || !angular_rad_s) return 1U;

  distance_sq = s_x_cm * s_x_cm + s_y_cm * s_y_cm;
  if (distance_sq <= arrival_radius_sq) {
    return 1U;
  }

  if (s_return_locked) {
    desired_heading = s_return_bearing_rad;
  } else {
    desired_heading = atan2f(-s_y_cm, -s_x_cm);
  }

  heading_error = WrapPi(desired_heading - RelativeHeading(heading_rad));
  abs_error = AbsF(heading_error);

  /* 迟滞: 进入原地转的门槛高于退出门槛，否则误差在单一阈值上下抖动时
   * 会在"原地转"和"边走边修"之间反复切换，走出画龙轨迹。 */
  if (s_rotate_in_place) {
    if (abs_error <= DEG_TO_RAD(HOME_ROTATE_EXIT_DEG)) s_rotate_in_place = 0U;
  } else if (abs_error > DEG_TO_RAD(HOME_ROTATE_IN_PLACE_DEG)) {
    s_rotate_in_place = 1U;
  }

  rate = ClampAngular(HOME_RETURN_HEADING_KP * heading_error);

  if (s_rotate_in_place) {
    /* 原地转也按比例限速：接近目标自动降速，避免定速冲过头再回摆。
     * 但保底一个最小角速度，否则小误差时克服不了静摩擦转不动。 */
    if (AbsF(rate) < HOME_ROTATE_MIN_ANGULAR_RAD_S) {
      rate = (heading_error > 0.0f) ? HOME_ROTATE_MIN_ANGULAR_RAD_S
                                    : -HOME_ROTATE_MIN_ANGULAR_RAD_S;
    }
    *angular_rad_s = rate;
  } else {
    *linear_cm_s = HOME_RETURN_SPEED_CM_S;
    *angular_rad_s = rate;
  }
  return 0U;
}

int16_t HomeTrajectory_GetXcm(void) { return RoundClampI16(s_x_cm); }
int16_t HomeTrajectory_GetYcm(void) { return RoundClampI16(s_y_cm); }

float HomeTrajectory_GetDistanceCm(void)
{
  return sqrtf(s_x_cm * s_x_cm + s_y_cm * s_y_cm);
}

int16_t HomeTrajectory_GetRelativeHeadingDeg(float heading_rad)
{
  return RoundClampI16(RelativeHeading(heading_rad) * 180.0f / PI_F);
}

int16_t HomeTrajectory_GetReturnBearingDeg(void)
{
  if (!s_return_locked) {
    return RoundClampI16(atan2f(-s_y_cm, -s_x_cm) * 180.0f / PI_F);
  }
  return RoundClampI16(s_return_bearing_rad * 180.0f / PI_F);
}
