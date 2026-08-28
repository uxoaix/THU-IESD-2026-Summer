#ifndef __SERVO_ALL_H__
#define __SERVO_ALL_H__

#include <stdint.h>

/* PA6/PA7/PB0为小舵机，PB1为大舵机；四路同步0~180度往复。 */
void ServoAll_Init(void);
void ServoAll_Run(uint32_t now_ms);

#endif /* __SERVO_ALL_H__ */
