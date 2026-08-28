#ifndef __OPENMV_UART_H__
#define __OPENMV_UART_H__

#include "motion_config.h"
#include "stm32f1xx_hal.h"

/*
 * ASCII帧：
 * V,color,cx,cy,distance_cm\n
 * color: 0=无目标, 1=红色, 2=黄色, 3=黑区。
 */
void OpenMvUart_Init(void);
void OpenMvUart_Process(void);
void OpenMvUart_GetLatest(VisionData_t *vision);
void OpenMvUart_SetDetectionMode(uint8_t black_area_mode);
uint32_t OpenMvUart_GetValidFrameCount(void);
uint32_t OpenMvUart_GetInvalidFrameCount(void);
void OpenMvUart_OnRxComplete(void);
void OpenMvUart_OnError(void);

#endif /* __OPENMV_UART_H__ */
