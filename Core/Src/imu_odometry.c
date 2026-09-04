#include "imu_odometry.h"
#include "home_trajectory.h"
#include "imu_driver.h"
#include "motion_config.h"

void ImuOdometry_Init(void)
{
  SensorFusion_Init();
  HomeTrajectory_Init();
}

void ImuOdometry_Update(const WheelFeedback_t *wheels, float dt_s)
{
  if (!wheels) return;

  SensorFusion_Update(wheels, dt_s);
  if (HomeTrajectory_IsOriginSet()) {
    HomeTrajectory_Update(SensorFusion_GetLinearVelocity(),
                          SensorFusion_GetHeading(), dt_s);
  }
}

void ImuOdometry_SetHome(void)
{
  SensorFusion_ResetHeading();
  HomeTrajectory_SetOrigin(SensorFusion_GetHeading());
}

void ImuOdometry_BeginReturn(void)
{
  HomeTrajectory_BeginReturn();
}

void ImuOdometry_NudgeReturnBearing(float delta_rad)
{
  HomeTrajectory_NudgeReturnBearing(delta_rad);
}

uint8_t ImuOdometry_IsOriginSet(void)
{
  return HomeTrajectory_IsOriginSet();
}

uint8_t ImuOdometry_IsImuAlive(void)
{
  return IMU_IsAlive() && IMU_IsInitialized();
}

float ImuOdometry_GetHeadingRad(void)
{
  return SensorFusion_GetHeading();
}

float ImuOdometry_GetLinearVelocityCmS(void)
{
  return SensorFusion_GetLinearVelocity();
}

float ImuOdometry_GetYawRateRadS(void)
{
  return SensorFusion_GetYawRate();
}

int16_t ImuOdometry_GetXcm(void)
{
  return HomeTrajectory_GetXcm();
}

int16_t ImuOdometry_GetYcm(void)
{
  return HomeTrajectory_GetYcm();
}

float ImuOdometry_GetDistanceCm(void)
{
  return HomeTrajectory_GetDistanceCm();
}

int16_t ImuOdometry_GetHeadingDeg(void)
{
  return HomeTrajectory_GetRelativeHeadingDeg(SensorFusion_GetHeading());
}

int16_t ImuOdometry_GetBearingDeg(void)
{
  return HomeTrajectory_GetReturnBearingDeg();
}

static int16_t RadToDegI16(float rad)
{
  float deg = rad * (180.0f / 3.14159265358979f);
  if (deg > 32767.0f) return 32767;
  if (deg < -32768.0f) return -32768;
  return (int16_t)(deg >= 0.0f ? deg + 0.5f : deg - 0.5f);
}

int16_t ImuOdometry_GetImuYawDeg(void)
{
  FusionState_t st;
  SensorFusion_GetState(&st);
  return RadToDegI16(st.heading_imu_rad);
}

int16_t ImuOdometry_GetEncHeadingDeg(void)
{
  FusionState_t st;
  SensorFusion_GetState(&st);
  return RadToDegI16(st.heading_enc_rad);
}

uint8_t ImuOdometry_GetReturnCommand(float *linear_cm_s, float *angular_rad_s)
{
  return HomeTrajectory_GetReturnCommand(SensorFusion_GetHeading(),
                                         linear_cm_s, angular_rad_s);
}

void ImuOdometry_GetFusionState(FusionState_t *out)
{
  SensorFusion_GetState(out);
}
