/**
 * @file    sensor_fusion.c
 * @brief   多源传感器融合实现: 卡尔曼滤波 + 互补滤波 (IMU + 编码器)
 *
 *   SensorFusion_Update(wheels, dt):
 *     ① IMUProcessor_Update()  —— 取 IMU 物理量 (有新周期才换算)
 *     ② EncoderHeading_Update  —— 用编码器速度积分航向 (同 motion_strategy Odometry 公式)
 *     ③ IMU 在线?
 *          是 → 按当前 mode 跑卡尔曼/互补融合
 *          否 → 降级: θ=θ_enc, ω=ω_enc
 *     ④ 算直线速度 (编码器均值, 可选叠 IMU 加速度预测)
 *     ⑤ 刷新对外状态 s_state
 *
 *   预测 (匀速转弯模型):
 *     θ_pred = θ + ω·dt        ω_pred = ω
 *     P_pred = F·P·Fᵀ + Q,   F=[[1,dt],[0,1]]
 *     Q = σ_a²·[[dt⁴/4, dt³/2],[dt³/2, dt²]]   (白色偏航加速度噪声)
 *   测量更新 (标量, 顺序处理 3 路):
 *     对每路 (z, H, R):
 *       S  = H·P·Hᵀ + R
 *       K  = P·Hᵀ / S
 *       y  = z - H·x        (角度测量时把 y 归一化到 [-π,π])
 *       x += K·y
 *       P  = (I - K·H)·P
 *   3 路测量:
 *     ① θ_enc  H=[1,0] R=R_enc      (编码器航向, 含打滑)
 *     ② θ_imu  H=[1,0] R=R_imu_yaw  (IMU yaw, 短期精密)
 *     ③ ω_imu  H=[0,1] R=R_imu_gyro (IMU 角速度, 最精密)
 *
 *   测量更新时把"新息" y 归一化到 [-π,π], 无论 IMU yaw 在哪一侧都能按
 *   最短角度方向修正, 不会因 ±180° 跨越算出大误差。
 *
 *   预测: θ_pred = θ_prev + ω_imu·dt              (陀螺短期推算)
 *   绝对修正: θ_corr = θ_enc + γ·wrap(θ_imu - θ_enc)  (IMU 与编码器按 γ 混合)
 *   融合: θ = θ_pred + (1-α)·wrap(θ_corr - θ_pred)  (预测为主, 绝对源长期拉回)
 *
 *   对外出口做标度校准: 滤波器内部保持原始航向空间, 只把"相对归零点的增量"
 *   按系数缩放后发布。两个出口用两个系数 ——
 *     GetHeading    (融合航向) 过 FUSION_YAW_SCALE。定角旋转、掉头、里程计
 *                   (x,y)、返航锁定方位角时的几何折算与遥测都吃这一路。
 *     GetImuHeading (纯 IMU 航向) 过 IMU_YAW_SCALE。HOME_FOLLOW 锁定之后的
 *                   保向闭环吃这一路 (只追 IMU 增量, 绕开编码器打滑), 另供
 *                   遥测 imu_yaw 对照。
 */
#include "sensor_fusion.h"
#include "imu_processor.h"
#include "imu_driver.h"

/* 内部状态 */

/* 编码器航向累加器 (与 motion_strategy 的 heading_estimate 同公式, 独立维护) */
static float s_enc_heading = 0.0f;     /* rad, 正=CCW */
static float s_enc_heading_prev = 0.0f;/* 上周期编码器航向 (算 ω_enc 用) */

/* 卡尔曼状态 */
static float s_kf_theta = 0.0f;        /* 航向 rad */
static float s_kf_omega = 0.0f;        /* 偏航角速度 rad/s */
static float s_kf_P00 = FUSION_KF_P0_HEADING;
static float s_kf_P01 = 0.0f;
static float s_kf_P11 = FUSION_KF_P0_RATE;

/* 互补滤波状态 */
static float s_comp_theta = 0.0f;      /* 上周期融合航向 */

/* 标度校准 (见 FUSION_YAW_SCALE)
 *   对外航向 = s_yaw_ref + FUSION_YAW_SCALE · (原始融合航向 - s_yaw_ref)
 *   滤波器内部一律用原始量, 只在对外出口缩放, 避免污染 IMU 绝对参考。 */
static float s_yaw_ref   = 0.0f;       /* 上次归零时的原始航向 (缩放基准) */
static float s_theta_raw = 0.0f;       /* 未缩放的融合航向 */

/* 纯 IMU 航向 (HOME_FOLLOW 保向闭环 + 遥测, 见 SensorFusion_GetImuHeading)
 *   JY901S 的 yaw 寄存器在 ±180° 回绕, 这里按周期差分展开成单调累加量,
 *   好让"起始航向 - 当前航向"这种判据能跨过 ±180° 边界。
 *   不参与滤波, 不过 FUSION_YAW_SCALE (那个是补编码器打滑的), 只过自己的
 *   IMU_YAW_SCALE (0.83, 补模组多报约 20% 的转角)。 */
static float s_imu_heading = 0.0f;     /* 展开后的纯 IMU 航向 (rad) */
static float s_imu_src_prev = 0.0f;    /* 上周期的增量来源读数 */
static uint8_t s_imu_src_alive = 0U;   /* 上周期用的是 IMU 还是编码器 */

/* 直线速度 */
static float s_lin_vel_cms = 0.0f;     /* 当前融合速度 */
static float s_lin_vel_prev = 0.0f;    /* 上周期速度 (加速度预测用) */

/* 对外状态 */
static FusionState_t s_state;
static FusionMode_t  s_mode = (FusionMode_t)FUSION_DEFAULT_MODE;

/* 内部工具函数 */

/**
 * @brief  把角度归一化到 [-π, π] */
static float WrapToPi(float angle)
{
    while (angle >  3.14159265358979f) angle -= 6.28318530717958f;
    while (angle < -3.14159265358979f) angle += 6.28318530717958f;
    return angle;
}

/**
 * @brief  编码器航向积分 (同 motion_strategy Odometry_Update 公式)
 * @param  wheels  4 轮速度反馈
 * @param  dt      周期 (秒)
 *
 *   v_left  = (FL + RL) / 2
 *   v_right = (FR + RR) / 2
 *   静止死区: 左右都 < ODOMETRY_STATIC_DEADZONE_CM_S → 不积分 (防零漂)
 *   ω_enc = (v_right - v_left) / WHEEL_TRACK_WIDTH_CM
 *   heading += ω_enc · dt
 * @return 本周期编码器推算的瞬时偏航角速度 ω_enc (rad/s, 供降级/调试用) */
static float EncoderHeading_Update(const WheelFeedback_t *wheels, float dt)
{
    const float v_left  = (wheels->speed_cm_s[WHEEL_FRONT_LEFT] +
                           wheels->speed_cm_s[WHEEL_REAR_LEFT])  * 0.5f;
    const float v_right = (wheels->speed_cm_s[WHEEL_FRONT_RIGHT] +
                           wheels->speed_cm_s[WHEEL_REAR_RIGHT]) * 0.5f;

    /* 静止死区 (与 motion_strategy 一致, 防编码器零漂让航向缓慢漂移) */
    if ((v_left  > -ODOMETRY_STATIC_DEADZONE_CM_S && v_left  < ODOMETRY_STATIC_DEADZONE_CM_S) &&
        (v_right > -ODOMETRY_STATIC_DEADZONE_CM_S && v_right < ODOMETRY_STATIC_DEADZONE_CM_S)) {
        return 0.0f;
    }

    const float omega_enc = (v_right - v_left) / WHEEL_TRACK_WIDTH_CM;
    s_enc_heading_prev = s_enc_heading;
    s_enc_heading += omega_enc * dt;
    return omega_enc;
}

/* 卡尔曼滤波 */

/**
 * @brief  卡尔曼预测 (匀速转弯模型)
 *   θ_pred = θ + ω·dt
 *   P_pred = F·P·Fᵀ + Q
 *   Q 用白色偏航加速度模型, σ_a² = FUSION_KF_Q_ACCEL */
static void KF_Predict(float dt)
{
    /* 状态预测 */
    s_kf_theta += s_kf_omega * dt;
    /* ω 不变 */

    /* 协方差预测: P = F·P·Fᵀ + Q
     *   P00' = P00 + 2·dt·P01 + dt²·P11 + Q00
     *   P01' = P01 + dt·P11 + Q01
     *   P11' = P11 + Q11 */
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    const float q  = FUSION_KF_Q_ACCEL;
    const float Q00 = q * dt4 * 0.25f;
    const float Q01 = q * dt3 * 0.5f;
    const float Q11 = q * dt2;

    const float new_P00 = s_kf_P00 + 2.0f * dt * s_kf_P01 + dt2 * s_kf_P11 + Q00;
    const float new_P01 = s_kf_P01 + dt * s_kf_P11 + Q01;
    const float new_P11 = s_kf_P11 + Q11;

    s_kf_P00 = new_P00;
    s_kf_P01 = new_P01;
    s_kf_P11 = new_P11;
}

/**
 * @brief  卡尔曼角度测量更新 (H=[1,0], 新息归一化到 [-π,π])
 *
 * P_new = P - K·H·P, 其中 K·H·P = (H·P)ᵀ·(H·P)/S (外积形式)。
 * @param  z    角度测量 (rad)
 * @param  R    测量方差 (rad²) */
static void KF_UpdateAngle(float z, float R)
{
    /* H·P (用旧 P): H=[1,0] → [P00, P01] */
    const float PH0 = s_kf_P00;
    const float PH1 = s_kf_P01;
    const float S  = PH0 + R;            /* H·P·Hᵀ + R = P00 + R */
    if (S < 1e-12f) { return; }          /* 数值保护 */
    const float K0 = PH0 / S;
    const float K1 = PH1 / S;

    /* 新息 (归一化到最短角差, 正确处理 ±180° 跨越) */
    const float y = WrapToPi(z - s_kf_theta);

    s_kf_theta += K0 * y;
    s_kf_omega += K1 * y;

    /* P = P - (H·P)ᵀ·(H·P)/S, 全部用旧 P (PH 已捕获) */
    s_kf_P00 -= PH0 * PH0 / S;
    s_kf_P01 -= PH0 * PH1 / S;
    s_kf_P11 -= PH1 * PH1 / S;
}

/**
 * @brief  卡尔曼角速度测量更新 (H=[0,1], 新息不归一化)
 * @param  z    角速度测量 (rad/s)
 * @param  R    测量方差 (rad²/s²) */
static void KF_UpdateRate(float z, float R)
{
    /* H·P (用旧 P): H=[0,1] → [P01, P11] */
    const float PH0 = s_kf_P01;
    const float PH1 = s_kf_P11;
    const float S  = PH1 + R;            /* H·P·Hᵀ + R = P11 + R */
    if (S < 1e-12f) { return; }
    const float K0 = PH0 / S;
    const float K1 = PH1 / S;

    const float y = z - s_kf_omega;

    s_kf_theta += K0 * y;
    s_kf_omega += K1 * y;

    /* P = P - (H·P)ᵀ·(H·P)/S, 全部用旧 P */
    s_kf_P00 -= PH0 * PH0 / S;
    s_kf_P01 -= PH0 * PH1 / S;
    s_kf_P11 -= PH1 * PH1 / S;
}

/**
 * @brief  跑一周期卡尔曼融合
 * @param  theta_enc   编码器航向 (rad)
 * @param  theta_imu   IMU yaw (rad, 已乘符号)
 * @param  omega_imu   IMU 角速度 (rad/s, 已乘符号)
 * @param  dt          周期 (秒) */
static void Fusion_Kalman(float theta_enc, float theta_imu, float omega_imu,
                          float dt, uint8_t imu_new)
{
    KF_Predict(dt);
    /* 编码器每周期更新；IMU 每个新周期只能融合一次，禁止重复消费旧帧。 */
    KF_UpdateAngle(theta_enc,   FUSION_KF_R_ENC_RAD);
    if (imu_new) {
        KF_UpdateAngle(theta_imu, FUSION_KF_R_IMU_YAW_RAD);
        KF_UpdateRate(omega_imu,  FUSION_KF_R_IMU_GYRO_RAD);
    }
}

/**
 * @brief  跑一周期互补滤波
 *   预测:  θ_pred = θ_prev + ω_imu·dt
 *   绝对源混合: θ_corr = θ_enc + γ·wrap(θ_imu - θ_enc)
 *   融合:  θ = θ_pred + (1-α)·wrap(θ_corr - θ_pred)
 *   ω 取 IMU 角速度 (短期最稳) */
static void Fusion_Complementary(float theta_enc, float theta_imu,
                                 float omega_imu, float omega_enc,
                                 float dt, uint8_t imu_new)
{
    if (imu_new) {
        const float theta_pred = s_comp_theta + omega_imu * dt;
        const float theta_corr = theta_enc +
          FUSION_COMP_GAMMA * WrapToPi(theta_imu - theta_enc);
        s_comp_theta = theta_pred + (1.0f - FUSION_COMP_ALPHA) *
          WrapToPi(theta_corr - theta_pred);
    } else {
        /* 没有新 IMU 帧时只按编码器推进，不能反复融合旧姿态。 */
        const float theta_pred = s_comp_theta + omega_enc * dt;
        s_comp_theta = theta_pred + (1.0f - FUSION_COMP_ALPHA) *
          WrapToPi(theta_enc - theta_pred);
    }
}

/* 公共接口实现 */

void SensorFusion_Init(void)
{
    s_enc_heading      = 0.0f;
    s_enc_heading_prev = 0.0f;

    s_kf_theta = 0.0f;
    s_kf_omega = 0.0f;
    s_kf_P00   = FUSION_KF_P0_HEADING;
    s_kf_P01   = 0.0f;
    s_kf_P11   = FUSION_KF_P0_RATE;

    s_comp_theta  = 0.0f;
    s_yaw_ref     = 0.0f;
    s_theta_raw   = 0.0f;
    s_imu_heading   = 0.0f;
    s_imu_src_prev  = 0.0f;
    s_imu_src_alive = 0U;
    s_lin_vel_cms = 0.0f;
    s_lin_vel_prev = 0.0f;

    s_mode = (FusionMode_t)FUSION_DEFAULT_MODE;

    /* 对外状态清零 */
    s_state.heading_rad     = 0.0f;
    s_state.yaw_rate_radps = 0.0f;
    s_state.linear_vel_cms  = 0.0f;
    s_state.heading_imu_rad = 0.0f;
    s_state.heading_enc_rad = 0.0f;
    s_state.imu_alive        = 0;
    s_state.fused           = 0;
    s_state.mode            = s_mode;
}

void SensorFusion_SetMode(FusionMode_t mode)
{
    s_mode = mode;
    /* 切模式时把当前航向同步进目标滤波器状态, 避免跳变。
     * 必须用未缩放的 s_theta_raw: 滤波器内部工作在原始航向空间。 */
    if (mode == FUSION_MODE_KALMAN) {
        s_kf_theta = s_theta_raw;
    }
    if (mode == FUSION_MODE_COMPLEMENTARY) {
        s_comp_theta = s_theta_raw;
    }
}

FusionMode_t SensorFusion_GetMode(void)
{
    return s_mode;
}

void SensorFusion_Update(const WheelFeedback_t *wheels, float dt)
{
    if (wheels == 0) { return; }

    /* dt 兜底 (调用方传 0 时按默认周期) */
    if (dt <= 0.0f) {
        dt = (float)FUSION_UPDATE_PERIOD_MS / 1000.0f;
    }

    /* ① 取 IMU 物理量 (内部判断有无新周期, 无则保持上次值) */
    const uint8_t imu_new = IMUProcessor_Update();

    /* ② 编码器航向积分 */
    const float omega_enc = EncoderHeading_Update(wheels, dt);

    /* ③ 取 IMU 物理量缓存 + 在线判断 */
    IMU_Physical_t imu;
    IMUProcessor_GetPhysical(&imu);
    const uint8_t imu_alive = IMU_IsAlive() && IMU_IsInitialized();

    /* IMU yaw/角速度转弧度并乘符号 (deg→rad, dps→rad/s) */
    const float DEG2RAD = 3.14159265358979f / 180.0f;
    const float theta_imu = imu.angle_deg[2] * DEG2RAD * FUSION_IMU_YAW_SIGN;
    const float omega_imu = imu.gyro_dps[2] * DEG2RAD * FUSION_IMU_YAW_SIGN;

    /* ④ 融合 (或降级) */
    float theta_out;
    float omega_out;

    if (imu_alive && s_mode != FUSION_MODE_OFF) {
        /* IMU 在线 + 融合开启 */
        if (s_mode == FUSION_MODE_KALMAN) {
            Fusion_Kalman(s_enc_heading, theta_imu, omega_imu, dt, imu_new);
            theta_out = s_kf_theta;
            omega_out = s_kf_omega;
        } else { /* FUSION_MODE_COMPLEMENTARY */
            Fusion_Complementary(s_enc_heading, theta_imu, omega_imu,
                                 omega_enc, dt, imu_new);
            theta_out = s_comp_theta;
            omega_out = imu_new ? omega_imu : omega_enc;
        }
        s_state.fused = 1;
    } else {
        /* 降级: 仅编码器 (IMU 离线 或 mode=OFF)
         *   θ = θ_enc, ω = ω_enc (本周期编码器推算)
         *   同步到卡尔曼/互补状态, IMU 恢复后平滑接续 */
        theta_out = s_enc_heading;
        omega_out = omega_enc;
        s_kf_theta = s_enc_heading;
        s_kf_omega = omega_enc;
        s_comp_theta = s_enc_heading;
        s_state.fused = 0;
    }

    /* ⑤ 直线速度 (编码器均值, 可选叠 IMU 加速度预测)
     *   v_enc = (v_left + v_right)/2
     *   v_pred = v_prev + a_imu_x·dt  (a: g→cm/s², 1g≈980.665 cm/s²)
     *   v = (1-β)·v_enc + β·v_pred, β=FUSION_VEL_ACCEL_WEIGHT (默认 0) */
    const float v_left  = (wheels->speed_cm_s[WHEEL_FRONT_LEFT] +
                           wheels->speed_cm_s[WHEEL_REAR_LEFT])  * 0.5f;
    const float v_right = (wheels->speed_cm_s[WHEEL_FRONT_RIGHT] +
                           wheels->speed_cm_s[WHEEL_REAR_RIGHT]) * 0.5f;
    const float v_enc = (v_left + v_right) * 0.5f;
    float v_out = v_enc;
    if (imu_alive && FUSION_VEL_ACCEL_WEIGHT > 0.0f) {
        const float a_cms2 = imu.accel_g[0] * 980.665f;
        const float v_pred = s_lin_vel_prev + a_cms2 * dt;
        v_out = (1.0f - FUSION_VEL_ACCEL_WEIGHT) * v_enc +
                FUSION_VEL_ACCEL_WEIGHT * v_pred;
    }
    s_lin_vel_prev = v_out;
    s_lin_vel_cms  = v_out;

    /*
     * ⑥' 纯 IMU 航向展开。
     *   IMU 在线时取 yaw 寄存器的增量 (要 WrapToPi, 它在 ±180° 回绕);
     *   离线时退化成取编码器航向的增量 (本身单调, 不用 wrap), 这样断线时
     *   曲线是平滑接续而不是跳变 —— 两个源的绝对参考不同, 所以只能取增量。
     *   切换源的那一周期先对齐 prev, 否则会算出一个跨源的假增量。
     */
    {
        const float src = imu_alive ? theta_imu : s_enc_heading;
        if (imu_alive != s_imu_src_alive) {
            s_imu_src_prev  = src;
            s_imu_src_alive = imu_alive;
        }
        /* 两个源各用自己的标度系数: IMU 段用 IMU_YAW_SCALE, 降级到编码器的
         * 段沿用 FUSION_YAW_SCALE (编码器打滑那一项本来就是它标定的)。 */
        s_imu_heading += imu_alive
          ? WrapToPi(src - s_imu_src_prev) * IMU_YAW_SCALE
          : (src - s_imu_src_prev) * FUSION_YAW_SCALE;
        s_imu_src_prev = src;
    }

    /* ⑥ 刷新对外状态 (航向/角速度过标度校准, 见 FUSION_YAW_SCALE) */
    s_theta_raw             = theta_out;
    s_state.heading_rad     = s_yaw_ref +
                              FUSION_YAW_SCALE * (theta_out - s_yaw_ref);
    s_state.yaw_rate_radps  = omega_out * FUSION_YAW_SCALE;
    s_state.linear_vel_cms  = v_out;
    s_state.heading_imu_rad = theta_imu;
    s_state.heading_enc_rad = s_enc_heading;
    s_state.imu_alive        = imu_alive;
    s_state.mode            = s_mode;
}

void SensorFusion_GetState(FusionState_t *out)
{
    if (out) { *out = s_state; }
}

float SensorFusion_GetHeading(void)
{
    return s_state.heading_rad;
}

float SensorFusion_GetYawRate(void)
{
    return s_state.yaw_rate_radps;
}

float SensorFusion_GetLinearVelocity(void)
{
    return s_state.linear_vel_cms;
}

float SensorFusion_GetImuHeading(void)
{
    return s_imu_heading;
}

float SensorFusion_GetYawRef(void)
{
    return s_yaw_ref;
}

void SensorFusion_ResetHeading(void)
{
    /* 【帧对齐】 编码器航向从 0 起算, IMU yaw 是绝对角, 两者参考点不同。
     * 复位时若 IMU 在线, 把所有航向状态对齐到当前 IMU yaw, 使编码器与 IMU
     * 共享同一参考; 之后编码器从该点积分, IMU 从该点解算, 融合才有效。
     * IMU 离线则对齐到 0 (纯编码器, 无绝对参考)。 */
    float theta0 = 0.0f;
    if (IMU_IsAlive() && IMU_IsInitialized()) {
        IMU_Physical_t imu;
        IMUProcessor_GetPhysical(&imu);
        const float DEG2RAD = 3.14159265358979f / 180.0f;
        theta0 = imu.angle_deg[2] * DEG2RAD * FUSION_IMU_YAW_SIGN;
    }

    /* 标度基准跟着归零点走: 之后的缩放只作用于相对该点的增量。 */
    s_yaw_ref           = theta0;
    s_theta_raw         = theta0;

    /* 纯 IMU 航向也对齐到同一归零点, 两个航向出口从此刻起同源同值。 */
    s_imu_heading       = theta0;
    s_imu_src_prev      = theta0;
    s_imu_src_alive     = (IMU_IsAlive() && IMU_IsInitialized()) ? 1U : 0U;

    s_enc_heading       = theta0;
    s_enc_heading_prev  = theta0;
    s_kf_theta          = theta0;
    s_kf_omega          = 0.0f;
    s_kf_P00            = FUSION_KF_P0_HEADING;
    s_kf_P01            = 0.0f;
    s_kf_P11            = FUSION_KF_P0_RATE;
    s_comp_theta        = theta0;
    s_state.heading_rad = theta0;
    s_state.heading_imu_rad = theta0;
    s_state.heading_enc_rad = theta0;
    /* yaw_rate / linear_vel 不清 (它们是瞬时量, 与航向归零无关) */
}
