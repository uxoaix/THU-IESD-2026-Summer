#ifndef __ENCODER_ALL_H__
#define __ENCODER_ALL_H__

#include "motor_all.h"
#include <stdint.h>

void EncoderAll_Init(const uint8_t modes[MOTOR_CHANNEL_COUNT],
                     const int8_t signs[MOTOR_CHANNEL_COUNT]);
void EncoderAll_Update(float dt_s);
int16_t EncoderAll_GetDelta(MotorId_t id);
int32_t EncoderAll_GetTotal(MotorId_t id);
float EncoderAll_GetRpm(MotorId_t id);
float EncoderAll_GetSpeedMmS(MotorId_t id);

#endif /* __ENCODER_ALL_H__ */
