/**
 * @file    imu_processor.c
 * @brief   IMU 上层处理实现: 原始 int16 → 物理量 + 静止零偏估计 (ZUPT)
 *
 *   IMUProcessor_Init   : 清零物理量 + 复位零偏估计器
 *   IMUProcessor_Update (主循环每周期调用):
 *     ① IMU_IsDataReady? 否 → 返回 0 (本周期无新数据)
 *     ② IMU_GetRawData → 原始 int16
 *     ③ 换算成 g / °/s / ° / °C
 *     ④ 静止判别: 三轴角速度都 < IMU_GYRO_STATIC_DPS 视为静止
 *        - 静止: 把本周期陀螺值累加进零偏缓冲, 满样本数更新零偏
 *        - 运动: 用现有零偏扣除陀螺 (gyro_corrected = gyro - bias)
 *     ⑤ 存最新物理量, 置"物理量就绪"
 *
 *   不每周期都更新零偏 (会跟着真实运动跑偏), 而是只在"判定静止"时累积,
 *   凑满 IMU_GYRO_BIAS_SAMPLES (默认 32) 个静止样本, 求均值作为新零偏。
 *   且新零偏若超过 IMU_GYRO_BIAS_MAX_DPS (3°/s) 视为异常 (多半是动了),
 *   不采纳。这样既能在静止时校正漂移, 又不会被真实转动污染。
 */
#include "imu_processor.h"

/* 内部状态 */

static IMU_Physical_t s_phys;              /* 最新物理量 (换算后) */
static volatile uint8_t s_phys_ready = 0;  /* 1=有新物理量 */

/* 陀螺零偏估计 (ZUPT) */
static float  s_gyro_bias_dps[3]   = {0.0f, 0.0f, 0.0f};  /* 当前零偏 (°/s) */
static float  s_bias_accum_dps[3];        /* 静止样本累加器 */
static uint16_t s_bias_sample_cnt   = 0;  /* 已累积静止样本数 */

/* 内部工具函数 */

/**
 * @brief  检测车体是否处于"角速度静止"状态
 * @param  gyro_dps  三轴角速度 (°/s, 未扣零偏)
 * @return 1=静止 (三轴都低于阈值), 0=运动中
 */
static uint8_t IsGyroStatic(const float gyro_dps[3])
{
    for (int i = 0; i < 3; i++) {
        if (gyro_dps[i] < -IMU_GYRO_STATIC_DPS ||
            gyro_dps[i] >  IMU_GYRO_STATIC_DPS) {
            return 0;
        }
    }
    return 1;
}

/**
 * @brief  用一帧陀螺数据推进零偏估计 (仅静止时调用)
 *
 *   ① 把本周期陀螺值加进累加器, 样本数+1
 *   ② 凑满 IMU_GYRO_BIAS_SAMPLES 个样本:
 *      - 求均值
 *      - 若均值幅度 < IMU_GYRO_BIAS_MAX_DPS → 采纳为新零偏
 *        (超过上限视为"其实动了", 丢弃)
 *      - 清空累加器, 重新开始下一轮 */
static void UpdateGyroBias(const float gyro_dps[3])
{
    for (int i = 0; i < 3; i++) {
        s_bias_accum_dps[i] += gyro_dps[i];
    }
    s_bias_sample_cnt++;

    if (s_bias_sample_cnt >= IMU_GYRO_BIAS_SAMPLES) {
        float new_bias[3];
        uint8_t valid = 1;
        for (int i = 0; i < 3; i++) {
            new_bias[i] = s_bias_accum_dps[i] / (float)s_bias_sample_cnt;
            /* 幅度检查: 超过上限视为异常 (多半判静止时其实动了), 整批丢弃 */
            if (new_bias[i] < -IMU_GYRO_BIAS_MAX_DPS ||
                new_bias[i] >  IMU_GYRO_BIAS_MAX_DPS) {
                valid = 0;
            }
        }
        if (valid) {
            for (int i = 0; i < 3; i++) {
                s_gyro_bias_dps[i] = new_bias[i];
            }
        }
        /* 无论采纳与否, 清空累加器开始下一轮 */
        for (int i = 0; i < 3; i++) {
            s_bias_accum_dps[i] = 0.0f;
        }
        s_bias_sample_cnt = 0;
    }
}

/* 公共接口实现 */

void IMUProcessor_Init(void)
{
    for (int i = 0; i < 3; i++) {
        s_phys.accel_g[i]  = 0.0f;
        s_phys.gyro_dps[i] = 0.0f;
        s_phys.angle_deg[i] = 0.0f;
        s_gyro_bias_dps[i]  = 0.0f;
        s_bias_accum_dps[i] = 0.0f;
    }
    s_phys.temp_c    = 0.0f;
    s_phys_ready     = 0;
    s_bias_sample_cnt = 0;
}

uint8_t IMUProcessor_Update(void)
{
    /* ① 无新周期数据则跳过 (非阻塞) */
    if (!IMU_IsDataReady()) {
        return 0;
    }

    /* ② 取原始 int16 */
    IMU_RawData_t raw;
    IMU_GetRawData(&raw);

    /* ③ 换算成物理量 (raw / LSB_PER_*) */
    s_phys.accel_g[0]   = (float)raw.ax    / IMU_ACCEL_LSB_PER_G;
    s_phys.accel_g[1]   = (float)raw.ay    / IMU_ACCEL_LSB_PER_G;
    s_phys.accel_g[2]   = (float)raw.az    / IMU_ACCEL_LSB_PER_G;
    s_phys.gyro_dps[0]  = (float)raw.wx   / IMU_GYRO_LSB_PER_DPS;
    s_phys.gyro_dps[1]  = (float)raw.wy   / IMU_GYRO_LSB_PER_DPS;
    s_phys.gyro_dps[2]  = (float)raw.wz   / IMU_GYRO_LSB_PER_DPS;
    s_phys.angle_deg[0] = (float)raw.roll  / IMU_ANGLE_LSB_PER_DEG;
    s_phys.angle_deg[1] = (float)raw.pitch / IMU_ANGLE_LSB_PER_DEG;
    s_phys.angle_deg[2] = (float)raw.yaw   / IMU_ANGLE_LSB_PER_DEG;
    s_phys.temp_c       = (float)raw.temp / IMU_TEMP_LSB_PER_C;

    /* ④ 零偏估计: 静止时累积更新零偏; 否则用现有零偏扣除陀螺
     *    注意: 静止判别用未扣零偏的 gyro_dps; 扣除后写入 s_phys.gyro_dps。 */
    if (IsGyroStatic(s_phys.gyro_dps)) {
        UpdateGyroBias(s_phys.gyro_dps);
    }
    /* 扣除零偏 (无论静止与否都扣, 静止时扣完≈0, 运动时扣掉漂移分量) */
    for (int i = 0; i < 3; i++) {
        s_phys.gyro_dps[i] -= s_gyro_bias_dps[i];
    }

    /* ⑤ 标记有新物理量 */
    s_phys_ready = 1;
    return 1;
}

uint8_t IMUProcessor_IsPhysicalReady(void)
{
    return s_phys_ready;
}

void IMUProcessor_GetPhysical(IMU_Physical_t *out)
{
    if (out == 0) {
        return;
    }
    *out = s_phys;
}

void IMUProcessor_GetGyroBias(float bias_dps[3])
{
    if (bias_dps == 0) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        bias_dps[i] = s_gyro_bias_dps[i];
    }
}
