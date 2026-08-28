/**
 * @file    ultrasonic_avoid.c
 * @brief   超声避障模块实现
 *
 *   - 用 static 变量维护扫描过程中的最小距离及其对应航向
 *   - 仅在有效距离范围内更新记录, 跳过盲区/超量程数据
 *   - ResetScan 在每次墙扫开始前调用, 确保无历史残留
 */
#include "ultrasonic_avoid.h"

static uint16_t min_distance_cm    = 0xFFFFu;  /* 最近墙距 (cm), 初始 0xFFFF=无效 */
static float    min_direction_rad  = 0.0f;      /* 最近墙点对应的航向 (rad) */
static uint8_t  has_valid_sample   = 0u;         /* 是否采到有效样本 (0=没, 1=有) */

void UltrasonicAvoid_ResetScan(void)
{
    /* 清空所有记录, 准备新一轮扫描 */
    min_distance_cm   = 0xFFFFu;
    min_direction_rad = 0.0f;
    has_valid_sample  = 0u;
}

void UltrasonicAvoid_RecordSample(uint16_t distance_cm, float heading_rad)
{
    /* 距离合理性检查: 超出有效范围视为噪声, 跳过
     * - < 5cm: 在盲区内, 数据不可靠
     * - > 300cm: 超出量程, 数据无效 */
    if (distance_cm < WALL_MIN_VALID_DIST_CM ||
        distance_cm > WALL_MAX_VALID_DIST_CM) {
        return;
    }
    /* 首次有效样本 或 比之前记录更近 → 更新最小记录 */
    if (!has_valid_sample || distance_cm < min_distance_cm) {
        min_distance_cm   = distance_cm;
        min_direction_rad = heading_rad;
        has_valid_sample  = 1u;
    }
}

uint16_t UltrasonicAvoid_GetMinDistance(void)
{
    return min_distance_cm;
}

float UltrasonicAvoid_GetMinDirection(void)
{
    return min_direction_rad;
}

uint8_t UltrasonicAvoid_HasValidSample(void)
{
    return has_valid_sample;
}
