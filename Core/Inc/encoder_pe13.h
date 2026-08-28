#ifndef __ENCODER_PE13_H__
#define __ENCODER_PE13_H__

void EncoderPe13_Init(void);
void EncoderPe13_Update(float dt_s);
float EncoderPe13_GetSpeedMmS(void);
void EncoderPe13_Reset(void);

#endif /* __ENCODER_PE13_H__ */
