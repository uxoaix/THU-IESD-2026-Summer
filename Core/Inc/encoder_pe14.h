#ifndef __ENCODER_PE14_H__
#define __ENCODER_PE14_H__

void EncoderPe14_Init(void);
void EncoderPe14_Update(float dt_s);
float EncoderPe14_GetSpeedMmS(void);
void EncoderPe14_Reset(void);

#endif /* __ENCODER_PE14_H__ */
