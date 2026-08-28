#ifndef __DBG_UART_H__
#define __DBG_UART_H__

#include <stdint.h>

/*
 * 调试输出，UART4 / PC10-PC11 / 115200 8N1 (与 HC-05 蓝牙共用同一 UART)。
 * UART4 的 TX 方向同时给 HC-05 收电脑指令 + STM32 发调试/ACK 输出；
 * HC-05 RX 独占 UART4 RX 方向收电脑端指令。半双工蓝牙模块天然支持这种分流。
 * USART3 / PB10-PB11 已归还给 JY901S IMU 姿态模块，严禁 dbg_uart 再碰 USART3。
 *
 * 工程用 --specs=nano.specs 且未链接 _printf_float，%f 不会输出，
 * 所有浮点量都要先自己乘系数转成整数再打印。
 */

void DbgUart_Init(void);
void DbgUart_Print(const char *str);
void DbgUart_Printf(const char *fmt, ...);

/* 把浮点按 scale 放大后四舍五入成整数，便于用 %d 打印 */
int32_t DbgUart_Scaled(float value, float scale);

#endif /* __DBG_UART_H__ */
