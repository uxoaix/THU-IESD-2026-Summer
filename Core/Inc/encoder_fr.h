#ifndef __ENCODER_FR_H__
#define __ENCODER_FR_H__

#include <stdint.h>

/* 右前轮编码器：TIM2 硬件正交解码，PA15 = A 相，PB3 = B 相。见 board_fr.h */

void EncoderFr_Init(void);

/* 每个控制周期调用一次，dt_s 为距上次调用的实际间隔（秒） */
void EncoderFr_Update(float dt_s);

/* 上一周期的增量计数，已带方向符号 */
int16_t EncoderFr_GetDelta(void);

/* 上电以来的累计计数，32 位不会像 CNT 那样 16 位回绕 */
int32_t EncoderFr_GetTotal(void);

/* 输出轴转速与轮面线速度 */
float EncoderFr_GetRpm(void);
float EncoderFr_GetSpeedMmS(void);

void EncoderFr_Reset(void);

#endif /* __ENCODER_FR_H__ */
