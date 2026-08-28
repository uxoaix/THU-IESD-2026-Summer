/**
 * @file    sensor_fusion.h
 * @brief   多源传感器融合: 卡尔曼滤波 + 互补滤波 (IMU + 编码器)
 *
 *   把 IMU 的角度/角速度 与 编码器推算的航向 融合, 输出更可靠的航向与速度。
 */
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "stm32f1xx_hal.h"
#include "motion_config.h"

/* 融合算法选择 (与 config.h FUSION_DEFAULT_MODE 数值对应) */
typedef enum {
    FUSION_MODE_OFF = 0,            /* 不融合: 仅编码器航向 (向后兼容) */
    FUSION_MODE_COMPLEMENTARY,      /* 互补滤波 (简单稳) */
    FUSION_MODE_KALMAN             /* 卡尔曼滤波 (最优估计, 默认) */
} FusionMode_t;

/**
 * @brief  融合后状态 (供 motion_strategy / 上位机观测)
 *
 *   heading_rad     : 融合航向 (rad, 正=CCW; 不归一化, 单调累加, 与 Odometry 同)
 *   yaw_rate_radps  : 融合偏航角速度 (rad/s)
 *   linear_vel_cms  : 融合直线速度 (cm/s, 正=前进) — 默认取编码器均值
 *   heading_imu_rad  : IMU yaw 转弧度 (调试用, 对比融合结果)
 *   heading_enc_rad  : 编码器航向 (调试用, 对比融合结果)
 *   imu_alive        : IMU 是否在线 (1=在线, 0=离线→仅编码器)
 *   fused            : 本周期是否做了融合 (1=融合, 0=仅编码器降级)
 *   mode             : 当前生效的融合算法
 */
typedef struct {
    float       heading_rad;
    float       yaw_rate_radps;
    float       linear_vel_cms;
    float       heading_imu_rad;
    float       heading_enc_rad;
    uint8_t     imu_alive;
    uint8_t     fused;
    FusionMode_t mode;
} FusionState_t;

/**
 * @brief  初始化融合模块 (清零航向 + 复位滤波器)
 *
 *   ① 编码器航向累加器清零
 *   ② 卡尔曼状态 x=[θ,ω]=0, 协方差 P 设为初值 (FUSION_KF_P0_*)
 *   ③ 互补滤波的"上周期航向"清零
 *   ④ 模式设为 FUSION_DEFAULT_MODE
 *
 * @note   需先调 IMU_Init + IMUProcessor_Init (若要用 IMU)。
 *         建议与 Odometry_ResetHeading 同时调用 (任务开始/卸货后)。
 */
void SensorFusion_Init(void);

/**
 * @brief  设置融合算法 (运行时可切换)
 * @param  mode  FUSION_MODE_OFF / COMPLEMENTARY / KALMAN
 * @note   切换时不复位航向 (平滑过渡); 想复位用 SensorFusion_ResetHeading。
 */
void SensorFusion_SetMode(FusionMode_t mode);

/** @brief 查询当前融合算法 */
FusionMode_t SensorFusion_GetMode(void);

/**
 * @brief  融合周期更新 (主循环每 FUSION_UPDATE_PERIOD_MS 调用一次)
 *
 *   ① 调 IMUProcessor_Update 取 IMU 物理量 (有新周期才换算)
 *   ② 用编码器速度推算编码器航向 (同 Odometry_Update 公式)
 *   ③ 查 IMU 是否在线:
 *        在线 → 按当前 mode 做卡尔曼/互补融合
 *        离线 → 降级为仅编码器 (θ=θ_enc, ω=编码器差分)
 *   ④ 计算直线速度 (编码器均值, 可选叠 IMU 加速度预测)
 *
 * @param  wheels  4 轮编码器反馈 (speed[], 单位 cm/s)
 * @param  dt      本周期时长 (秒), 通常 MOTION_PERIOD_MS/1000 = 0.05
 *
 * @note   非阻塞。dt 建议用实测值; 若传 0 内部按 FUSION_UPDATE_PERIOD_MS 兜底。
 */
void SensorFusion_Update(const WheelFeedback_t *wheels, float dt);

/**
 * @brief  取出最新融合状态 (拷贝到 out)
 * @param  out  [输出] 融合状态结构体
 */
void SensorFusion_GetState(FusionState_t *out);

/** @brief 取融合航向 (rad, 正=CCW, 单调累加) — 替代 Odometry_GetHeading */
float SensorFusion_GetHeading(void);

/** @brief 取融合偏航角速度 (rad/s) */
float SensorFusion_GetYawRate(void);

/** @brief 取融合直线速度 (cm/s) */
float SensorFusion_GetLinearVelocity(void);

/**
 * @brief  复位航向为 0 (每轮任务开始/卸货后调用, 与 Odometry_ResetHeading 同语义)
 * @note   只清航向状态, 不改融合算法/噪声参数。
 *         建议同时调 IMU 配置复位 yaw (若模组支持) 或重新对齐零点。
 */
void SensorFusion_ResetHeading(void);

#endif /* SENSOR_FUSION_H */
