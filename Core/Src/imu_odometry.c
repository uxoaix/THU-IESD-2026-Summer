#include "imu_odometry.h"
#include "home_trajectory.h"
#include "imu_driver.h"
#include "motion_config.h"
#include <math.h>

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

/*
 * 建立坐标系: 航向清零 + 把原点定在车右侧 HOME_START_OFFSET_RIGHT_CM 处。
 * 开机初始化和蓝牙 SET_HOME 用。原点不取车脚下, 是因为要返回的黑区并不在
 * 上电位置上, 见 motion_config.h 里 HOME_START_OFFSET_RIGHT_CM 的说明。
 */
void ImuOdometry_SetHome(void)
{
  SensorFusion_ResetHeading();
  HomeTrajectory_SetOrigin(SensorFusion_GetHeading(),
                           HOME_START_OFFSET_RIGHT_CM);
}

/*
 * 只把 (0,0) 挪到当前位置, 不动航向。卸货时用它标记下一轮的原点。
 * 不调 SensorFusion_ResetHeading() 的原因: 那会连同卡尔曼协方差一起重置, 车
 * 正在动的时候清一次会带来一段收敛瞬态; 而返航只关心相对原点的位置和方位角,
 * SetOrigin 已经把新的原点航向记下来了, 航向累加器本身没有必要跟着归零。
 */
void ImuOdometry_MoveOriginHere(void)
{
  /* 偏移传 0: 此刻车就在真实卸货点上, 不像开机那次需要猜黑区在哪。 */
  HomeTrajectory_SetOrigin(SensorFusion_GetHeading(), 0.0f);
}

/* 返航直线段: 转发给 home_trajectory, 见那边的说明。 */
void ImuOdometry_BeginReturn(void)
{
  HomeTrajectory_BeginReturn();
}

void ImuOdometry_NudgeReturnBearing(float delta_rad)
{
  HomeTrajectory_NudgeReturnBearing(delta_rad);
}

uint8_t ImuOdometry_GetReturnCommand(float heading_rad,
                                     float *linear_cm_s,
                                     float *angular_rad_s)
{
  return HomeTrajectory_GetReturnCommand(heading_rad, linear_cm_s,
                                         angular_rad_s);
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

/*
 * 把"相对归零点的转角"换成 0~359 的整数度, 供遥测显示。
 *
 * 零点取 SensorFusion_GetYawRef(): 上电即为 0, 之后转多少就加减多少, 左转
 * (CCW) 增大、右转 (CW) 减小, 在 0/360 边界回绕 (右转 5° 显示 355)。
 * 蓝牙 SET_HOME 与 AUTO 起步会重新归零, 那是 SET_HOME 的既定语义。
 *
 * 用 fmodf 而不是 while 逐圈减: 三个航向累加器都是单调量, 跑一场下来能到上万
 * 度, while 会退化成上百次迭代。
 */
static int16_t RadToDeg360(float rad_rel)
{
  float deg = fmodf(rad_rel * (180.0f / 3.14159265358979f), 360.0f);
  int16_t out;
  if (deg < 0.0f) deg += 360.0f;
  /* 四舍五入会把 359.7 抬成 360, 归回 0 才能保证区间是 [0,360)。 */
  out = (int16_t)(deg + 0.5f);
  return (out >= 360) ? 0 : out;
}

/* 融合航向 (0~359)。已过 FUSION_YAW_SCALE, 是全车控制实际用的那一路。
 * 注意不再叠 HOME_HEADING_OFFSET_DEG —— 那个补偿只服务于 x/y 积分。 */
int16_t ImuOdometry_GetHeadingDeg(void)
{
  return RadToDeg360(SensorFusion_GetHeading() - SensorFusion_GetYawRef());
}

/* 返航目标方位角 (0~359)。与 hdg_deg 同零点, 两者相减即当前航向偏差。 */
int16_t ImuOdometry_GetBearingDeg(void)
{
  return RadToDeg360(HomeTrajectory_GetReturnBearingRad() -
                     SensorFusion_GetYawRef());
}

/*
 * 纯 IMU 航向 (0~359), 未过任何标度校准。仅供遥测对照, 不参与控制。
 * 与 yaw_enc / hdg_deg 同零点, 三者对照即可定位标度误差归属: 原地转过已知
 * 角度后, 谁的增量偏离实际转角, 问题就在谁身上。
 */
int16_t ImuOdometry_GetImuYawDeg(void)
{
  return RadToDeg360(SensorFusion_GetImuHeading() - SensorFusion_GetYawRef());
}

/* 编码器推算航向 (0~359), 未过标度校准。四轮滑移转向用两轮差速公式算的,
 * 原地转时会系统性偏大, 见 FUSION_YAW_SCALE 注释。 */
int16_t ImuOdometry_GetEncHeadingDeg(void)
{
  FusionState_t st;
  SensorFusion_GetState(&st);
  return RadToDeg360(st.heading_enc_rad - SensorFusion_GetYawRef());
}

void ImuOdometry_GetFusionState(FusionState_t *out)
{
  SensorFusion_GetState(out);
}
