/**
 * @file    ultrasonic_avoid.h
 * @brief   超声避障模块: 墙体距离采样 + 最近墙点方向记录
 *
 *   ResetScan:      开始新扫描前清空旧笔记
 *   RecordSample:   记一条新笔记 (距离 + 当时的朝向)
 *   GetMinDistance: 翻出最近墙距
 *   GetMinDirection:翻出最近墙点对应的朝向
 *   HasValidSample: 查是否记到过有效数据
 */
#ifndef ULTRASONIC_AVOID_H
#define ULTRASONIC_AVOID_H

#include "motion_config.h"

/**
 * @brief  复位扫描状态 (进入墙扫前调用)
 *
 * 【作用】
 *   清空历史记录, 准备开始新一轮 360° 扫描。
 */
void UltrasonicAvoid_ResetScan(void);

/**
 * @brief  记录一次墙体距离采样
 *
 * 【作用】
 *   把外部传来的 (距离, 航向) 存入扫描记录。若比之前记录的都近,
 *   更新"最近距离"和"对应航向"。
 *
 * @param  distance_cm  距离 (厘米)
 * @param  heading_rad  采样时车体朝向 (弧度, 由里程计提供)
 *
 * @note   仅记录有效距离 (5~300 cm), 超出范围视为噪声, 跳过不记。
 *         距离比当前最小值更小 → 更新记录。
 */
void UltrasonicAvoid_RecordSample(uint16_t distance_cm, float heading_rad);

/**
 * @brief  查询扫描中最小墙体距离
 * @return 最近距离 (厘米); 未采到有效点返回 0xFFFF (65535, 表示无效)
 */
uint16_t UltrasonicAvoid_GetMinDistance(void);

/**
 * @brief  查询最近墙点对应的车体航向
 * @return 航向 (弧度); 未采到有效点返回 0.0f
 * @note   这个朝向告诉 motion_strategy "第二圈应该转到哪里"
 */
float UltrasonicAvoid_GetMinDirection(void);

/**
 * @brief  查询是否已采到有效样本
 * @return 1=已有有效样本, 0=尚未采到
 * @note   墙扫结束时若返回 0, 说明全程没采到有效数据, 应直接恢复搜索
 */
uint8_t UltrasonicAvoid_HasValidSample(void);

#endif /* ULTRASONIC_AVOID_H */
