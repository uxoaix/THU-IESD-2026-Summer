/**
 * @file    imu_processor.h
 * @brief   IMU 上层处理: 原始 int16 → 物理量 + 静止零偏估计 (ZUPT)
 *
 *   加速度 (g)   = ax / 32768 × 16        → ax / IMU_ACCEL_LSB_PER_G
 *   角速度 (°/s) = wx / 32768 × 2000      → wx / IMU_GYRO_LSB_PER_DPS
 *   角度 (°)    = roll / 32768 × 180     → roll / IMU_ANGLE_LSB_PER_DEG
 *   温度 (°C)   = temp / 100              → temp / IMU_TEMP_LSB_PER_C
 *
 *   陀螺仪静止时本应输出 0°/s, 实际有 0.x°/s 的零偏。直接积分会让航向
 *   随时间线性漂移(几分钟偏几度)。本模块在车体静止(三轴角速度都小)时,
 *   把陀螺的均值当作零偏累计, 凑满 IMU_GYRO_BIAS_SAMPLES 个样本后更新零偏,
 *   之后每次换算都先减掉零偏。详见 IMUProcessor_Update 实现。
 */
#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "stm32f1xx_hal.h"
#include "motion_config.h"
#include "imu_driver.h"

/**
 * @brief  IMU 物理量读数 (换算后, 含零偏扣除)
 *
 *   accel_g[3]   : 加速度 (g), 下标 0=x,1=y,2=z
 *   gyro_dps[3]  : 角速度 (°/s), 已扣零偏
 *   angle_deg[3] : 角度 (°), 下标 0=roll,1=pitch,2=yaw
 *   temp_c       : 温度 (°C)
 */
typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float angle_deg[3];
    float temp_c;
} IMU_Physical_t;

/**
 * @brief  初始化 IMU 处理层 (清零物理量缓存 + 零偏估计状态)
 *
 *   ① 物理量缓存清零
 *   ② 零偏估计器复位 (零偏=0, 累积样本数=0)
 *   ③ 标记"无新物理量" (要等下一次 IMU 周期到来)
 *
 * @note   不调用 imu_driver 的初始化——那是 IMU_Init 的职责, 需先于本函数。
 */
void IMUProcessor_Init(void);

/**
 * @brief  周期更新: 读 IMU 原始 → 换算 → 扣零偏 → 存物理量
 *
 *   ① 查询 IMU_IsDataReady, 若无新周期数据则直接返回 0
 *   ② IMU_GetRawData 取原始 int16
 *   ③ 按公式换算成 g/°/s/°/°C
 *   ④ 若检测到静止, 累积陀螺均值用于更新零偏; 否则用已存零偏扣除陀螺
 *   ⑤ 标记"有新物理量"
 *
 * @return 1=本次取到并换算了新数据, 0=暂无新周期数据 (上层可跳过)
 *
 * @note   非阻塞。建议在主循环每周期调用一次 (与状态机同频)。
 *         即使返回 0, IMUProcessor_GetPhysical 仍可拿到上次旧值。
 */
uint8_t IMUProcessor_Update(void);

/**
 * @brief  查询是否有新的物理量就绪
 * @return 1=有新数据(IMUProcessor_Update 本次返回1), 0=无
 * @note   读取后不清除 (与 IMU_IsDataReady 语义不同, 供融合层判断)
 */
uint8_t IMUProcessor_IsPhysicalReady(void);

/**
 * @brief  取出最新物理量 (拷贝到 out)
 * @param  out  [输出] 物理量结构体 (调用方提供)
 * @note   即使无新数据, 也返回上一次的值。
 *         accel/gyro/angle 数组下标见 IMU_Physical_t 字段说明。
 */
void IMUProcessor_GetPhysical(IMU_Physical_t *out);

/**
 * @brief  查询当前估计的陀螺零偏 (供调试/自检)
 * @param  bias_dps  [输出] 三轴零偏 (°/s), 已扣到陀螺上
 * @note   零偏在静止累积满样本后才更新, 启动初期为 0。
 */
void IMUProcessor_GetGyroBias(float bias_dps[3]);

#endif /* IMU_PROCESSOR_H */
