/*
 * 【文件说明】 dbg_uart.c —— 调试串口输出 (与 HC-05 蓝牙共用 UART4)
 *
 * 【职责】给整个工程提供统一的调试打印接口 DbgUart_Print / DbgUart_Printf。
 *        底层走 UART4 (PC10/PC11), 与 HC-05 蓝牙模块共用 TX 方向,
 *        所以从 STM32 发出的调试信息 + HC-05 的 ACK 回显, 电脑端都能收到。
 *
 * 【类比理解】
 *   - DBG_UART = 调试通道的 "水管", 接到 UART4 这条 "主管道"
 *   - Print = 直接把整个字符串倒进去 (最快, 不做格式化)
 *   - Printf = 用 vsnprintf 先 "打字排版" 到缓冲区, 再倒出去 (方便但慢)
 *
 * 【参数裁剪】
 *   DBG_TX_TIMEOUT = 20ms —— HAL_UART_Transmit 的阻塞超时
 *   DBG_BUF_SIZE   = 240 字节 —— Printf 的缓冲区大小, 超长会被截断
 *
 * 【注意事项】
 *   - 用的是 HAL_UART_Transmit (阻塞式), 意味着打印期间 CPU 会等发送完,
 *     所以别在中断里调 Printf (会卡死或触发 HAL 错误)
 *   - DbgUart_Scaled 是小工具函数, 把 float × scale 四舍五入成 int32_t,
 *     方便调试时打印放大的浮点值
 */

#include "dbg_uart.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DBG_UART        (&huart4)            /* UART4 与 HC-05 共用 —— USART3 归 IMU */
#define DBG_TX_TIMEOUT  20U                  /* HAL_UART_Transmit 阻塞超时 20ms */
#define DBG_BUF_SIZE    240                  /* Printf 格式化缓冲区大小 */

/* vsnprintf 的工作缓冲区, 复用避免每打一条就 malloc */
static char s_buf[DBG_BUF_SIZE];

/* --------------------------------------------------------------------------
 * 【函数】DbgUart_Init —— 打印一条启动就绪提示
 * 【调用时机】AppCore_Init / AppHardwareTest_Init 最开头
 * ------------------------------------------------------------------------- */
void DbgUart_Init(void)
{
  DbgUart_Print("\r\n[APP] motion controller ready\r\n");
}

/* --------------------------------------------------------------------------
 * 【函数】DbgUart_Print
 * 【作用】原样发送一个 C 字符串 (不做格式化, 最快)
 * 【参数】str - 以 '\0' 结尾的字符串; 空串直接返回
 * ------------------------------------------------------------------------- */
void DbgUart_Print(const char *str)
{
  size_t len = strlen(str);

  if (len == 0U)
  {
    return;
  }
  HAL_UART_Transmit(DBG_UART, (uint8_t *)str, (uint16_t)len, DBG_TX_TIMEOUT);
}

/* --------------------------------------------------------------------------
 * 【函数】DbgUart_Printf
 * 【作用】格式化打印 (支持 %d/%u/%x/%s/%f 等 printf 格式)
 * 【参数】fmt - 格式化字符串, 后面跟可变参数 (...)
 * 【截断策略】vsnprintf 返回 n > 缓冲区大小时截断到 DBG_BUF_SIZE-1
 * ------------------------------------------------------------------------- */
void DbgUart_Printf(const char *fmt, ...)
{
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(s_buf, sizeof(s_buf), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return;
  }
  if (n > (int)sizeof(s_buf) - 1)
  {
    n = (int)sizeof(s_buf) - 1;
  }
  HAL_UART_Transmit(DBG_UART, (uint8_t *)s_buf, (uint16_t)n, DBG_TX_TIMEOUT);
}

/* --------------------------------------------------------------------------
 * 【函数】DbgUart_Scaled
 * 【作用】把 float × scale 四舍五入成 int32_t (调试时放大浮点用)
 * 【示例】DbgUart_Scaled(3.1415f, 1000.0f) → 3142
 * ------------------------------------------------------------------------- */
int32_t DbgUart_Scaled(float value, float scale)
{
  float v = value * scale;

  return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}
