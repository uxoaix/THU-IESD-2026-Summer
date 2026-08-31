#include "home_trajectory.h"
#include "motion_config.h"
#include <math.h>

#define PI_F 3.14159265358979f

static float s_x_cm;
static float s_y_cm;
static float s_home_heading;
static uint8_t s_origin_set;

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

void HomeTrajectory_Init(void)
{
  s_x_cm = 0.0f;
  s_y_cm = 0.0f;
  s_home_heading = 0.0f;
  s_origin_set = 0U;
}

void HomeTrajectory_SetOrigin(float heading_rad)
{
  s_x_cm = 0.0f;
  s_y_cm = 0.0f;
  s_home_heading = heading_rad;
  s_origin_set = 1U;
}

uint8_t HomeTrajectory_IsOriginSet(void)
{
  return s_origin_set;
}

void HomeTrajectory_Update(float linear_cm_s, float heading_rad, float dt_s)
{
  float relative_heading;

  if (!s_origin_set || dt_s <= 0.0f) return;

  relative_heading = heading_rad - s_home_heading;
  s_x_cm += linear_cm_s * cosf(relative_heading) * dt_s;
  s_y_cm += linear_cm_s * sinf(relative_heading) * dt_s;
}

uint8_t HomeTrajectory_GetReturnCommand(float heading_rad,
                                        float *linear_cm_s,
                                        float *angular_rad_s)
{
  float dx;
  float dy;
  float distance_sq;
  float desired_heading;
  float heading_error;
  const float arrival_radius_sq =
    HOME_ARRIVAL_RADIUS_CM * HOME_ARRIVAL_RADIUS_CM;

  if (linear_cm_s) *linear_cm_s = 0.0f;
  if (angular_rad_s) *angular_rad_s = 0.0f;
  if (!s_origin_set || !linear_cm_s || !angular_rad_s) return 1U;

  /* 当前融合位姿到原点(0,0)的直线向量。 */
  dx = -s_x_cm;
  dy = -s_y_cm;
  distance_sq = dx * dx + dy * dy;
  if (distance_sq <= arrival_radius_sq) {
    return 1U;
  }

  desired_heading = atan2f(dy, dx);
  heading_error = WrapPi(desired_heading -
                         (heading_rad - s_home_heading));

  if (AbsF(heading_error) > DEG_TO_RAD(HOME_ROTATE_IN_PLACE_DEG)) {
    *angular_rad_s = (heading_error > 0.0f)
                   ? HOME_RETURN_MAX_ANGULAR_RAD_S
                   : -HOME_RETURN_MAX_ANGULAR_RAD_S;
  } else {
    *linear_cm_s = HOME_RETURN_SPEED_CM_S;
    *angular_rad_s = HOME_RETURN_HEADING_KP * heading_error;
    if (*angular_rad_s > HOME_RETURN_MAX_ANGULAR_RAD_S) {
      *angular_rad_s = HOME_RETURN_MAX_ANGULAR_RAD_S;
    } else if (*angular_rad_s < -HOME_RETURN_MAX_ANGULAR_RAD_S) {
      *angular_rad_s = -HOME_RETURN_MAX_ANGULAR_RAD_S;
    }
  }
  return 0U;
}

int16_t HomeTrajectory_GetXcm(void) { return RoundClampI16(s_x_cm); }
int16_t HomeTrajectory_GetYcm(void) { return RoundClampI16(s_y_cm); }

float HomeTrajectory_GetDistanceCm(void)
{
  return sqrtf(s_x_cm * s_x_cm + s_y_cm * s_y_cm);
}
