#ifndef __ENCODER_FL_H__
#define __ENCODER_FL_H__

#include <stdint.h>

/* 左前轮编码器：TIM4硬件正交解码，PB6=A相，PB7=B相 */
void EncoderFl_Init(void);
void EncoderFl_Update(float dt_s);
int16_t EncoderFl_GetDelta(void);
int32_t EncoderFl_GetTotal(void);
float EncoderFl_GetRpm(void);
float EncoderFl_GetSpeedMmS(void);
void EncoderFl_Reset(void);

#endif /* __ENCODER_FL_H__ */
