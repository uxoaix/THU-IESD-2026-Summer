#ifndef __HC05_UART_H__
#define __HC05_UART_H__

#include <stdint.h>

typedef enum {
  HC05_EVENT_NONE = 0,
  HC05_EVENT_AUTO,
  HC05_EVENT_MANUAL,
  HC05_EVENT_FORWARD,
  HC05_EVENT_BACKWARD,
  HC05_EVENT_LEFT,
  HC05_EVENT_RIGHT,
  HC05_EVENT_ROTATE_CW,
  HC05_EVENT_ROTATE_CCW,
  HC05_EVENT_STOP,
  HC05_EVENT_SERVO_DOOR,
  HC05_EVENT_SERVO_CAMERA,
  HC05_EVENT_SERVO_BUCKET,
  HC05_EVENT_SERVO_BRUSH,
  HC05_EVENT_SET_HOME,
  HC05_EVENT_STATUS
} Hc05Event_t;

void Hc05Uart_Init(void);
void Hc05Uart_Process(void);
Hc05Event_t Hc05Uart_TakeEvent(void);
void Hc05Uart_SendAck(const char *command);
void Hc05Uart_OnRxComplete(void);
void Hc05Uart_OnError(void);
uint32_t Hc05Uart_GetValidCount(void);
uint32_t Hc05Uart_GetInvalidCount(void);
/* 兼容硬件测试应用；统一 AppCore 不再使用停止锁存。 */
uint8_t Hc05Uart_IsStopped(void);

#endif
