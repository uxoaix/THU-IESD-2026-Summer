#ifndef __OPENMV_UART_H__
#define __OPENMV_UART_H__

#include "motion_config.h"
#include "stm32f1xx_hal.h"

/*
 * ASCII帧：
 * V,color,cx,cy,distance_cm\n
 * color: 0=无目标, 1=红色, 2=黄色, 3=黑区。
 *
 * W,state,fill_pct\n  蓝色边界墙, 每帧都发。
 * state: 0=安全, 1=需要退避; fill_pct 是蓝色占 ROI 的百分比 (仅供标定)。
 *
 * A,state,fill_pct\n  黑区到位, 只在黑区模式 (M,1) 下发。
 * state: 1=可以掉头卸货; fill_pct 是黑区外框占整幅画面的百分比 (仅供标定)。
 */
void OpenMvUart_Init(void);
void OpenMvUart_Process(void);
void OpenMvUart_GetLatest(VisionData_t *vision);
void OpenMvUart_GetWall(VisionWallData_t *wall);
void OpenMvUart_GetArrival(VisionArrivalData_t *arrival);
void OpenMvUart_SetDetectionMode(uint8_t black_area_mode);
uint32_t OpenMvUart_GetValidFrameCount(void);
uint32_t OpenMvUart_GetInvalidFrameCount(void);
uint32_t OpenMvUart_GetWallFrameCount(void);
uint32_t OpenMvUart_GetArrivalFrameCount(void);
void OpenMvUart_OnRxComplete(void);
void OpenMvUart_OnError(void);

#endif /* __OPENMV_UART_H__ */
