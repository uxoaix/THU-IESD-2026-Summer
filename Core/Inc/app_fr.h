#ifndef __APP_FR_H__
#define __APP_FR_H__

/* PE9/PE11/PE13/PE14四电机开环助推+独立PID控制 */

void AppFr_Init(void);

/* 在 main 的 while(1) 里无条件调用，内部自己判周期 */
void AppFr_Run(void);

#endif /* __APP_FR_H__ */
