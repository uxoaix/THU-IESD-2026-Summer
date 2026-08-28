/*
 * 【文件说明】 openmv_uart.c —— OpenMV 视觉模组串口接收 (USART2)
 *
 * 【职责】通过 USART2 (PA2/PA3) 从 OpenMV H7 接收视觉识别结果,
 *        用环形缓冲区 + 行解析的方式把 OpenMV 发来的一行行字符串
 *        (以 '\n' 结尾) 解析成 VisionData_t 结构体。
 *
 * 【数据流】
 *   OpenMV → USART2 RX 中断 → OnRxComplete → s_ring (环形缓冲区)
 *            → OpenMvUart_Process (主循环调) → ParseLine → s_latest
 *            → 上层 GetLatest 拿到 VisionData_t
 *
 * 【协议格式】
 *   新版 (推荐): V,detected,x,y,dist,type\n
 *     detected  (0/1), x/y 像素偏移 (-320~320, -240~240), distance cm
 *   旧版 (兼容): $cx,cy,w,h,dist\n
 *     中心坐标 + 宽高 + 距离, 没有类别 → 强制 type=1
 *
 * 【保护机制】
 *   - VISION_STALE_MS = 500ms —— 如果 500ms 没收到新帧, GetLatest 自动把 detected 清 0
 *   - 环形缓冲区满了就丢字节 (s_invalid_frames++), 不阻塞中断
 *   - 行超 63 字符就丢弃这行 (防止内存溢出)
 *   - GetLatest 用 PRIMASK 关中断拷贝 s_latest (防读到半更新值)
 *
 * 【硬件绑定】
 *   USART2 → PA2 (TX, STM32 发 OpenMV 控制指令) + PA3 (RX, OpenMV 发识别结果)
 */

#include "openmv_uart.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define RX_RING_SIZE          128U       /* 接收环形缓冲区容量 (字节) */
#define LINE_BUFFER_SIZE       64U       /* 单行最大长度 (63 + '\0') */
#define VISION_STALE_MS       500U       /* 超时判定 —— 500ms 没新帧就视为丢失目标 */

/* s_rx_byte: HAL 中断接收的当前字节 (1 字节环形), s_ring: 实际环形缓冲区
 * s_write_index/s_read_index: 读写索引; s_line: 当前正在拼的一行
 * s_latest: 最新有效识别结果; s_valid_frames/s_invalid_frames: 好帧/坏帧计数
 * s_detection_mode: 当前 OpenMV 处于哪种识别模式 (0=色块, 1=黑线) */
static uint8_t s_rx_byte;
static volatile uint8_t s_ring[RX_RING_SIZE];
static volatile uint16_t s_write_index;
static volatile uint16_t s_read_index;
static char s_line[LINE_BUFFER_SIZE];
static uint8_t s_line_length;
static VisionData_t s_latest;
static uint32_t s_valid_frames;
static uint32_t s_invalid_frames;
static uint8_t s_detection_mode = 0xFFU;

/* --------------------------------------------------------------------------
 * 【函数】ParseLine
 * 【作用】把一行字符串解析成 VisionData_t, 支持新版和旧版两种协议
 * 【参数】line - 以 '\0' 结尾的一行字符串 (不含 \r\n)
 * 【返回】无; 成功时 s_latest 更新 + s_valid_frames++, 失败 s_invalid_frames++
 * 【校验】所有字段都做范围检查 (比如 distance ≤ 1000cm), 超出就当坏帧
 * ------------------------------------------------------------------------- */
static void ParseLine(const char *line)
{
  unsigned int detected;
  int x_offset;
  int y_offset;
  unsigned int distance;
  unsigned int object_type;
  int center_x;
  int center_y;
  unsigned int width;
  unsigned int height;

  if (sscanf(line, "V,%u,%d,%d,%u,%u",
             &detected,
             &x_offset,
             &y_offset,
             &distance,
             &object_type) == 5)
  {
    if ((detected <= 1U) &&
        (x_offset >= -320) && (x_offset <= 320) &&
        (y_offset >= -240) && (y_offset <= 240) &&
        (distance <= 1000U) &&
        (object_type <= 255U))
    {
      s_latest.detected = (uint8_t)detected;
      s_latest.x_offset_px = (int16_t)x_offset;
      s_latest.y_offset_px = (int16_t)y_offset;
      s_latest.distance_cm = (uint16_t)distance;
      s_latest.object_type = (uint8_t)object_type;
      s_latest.timestamp_ms = HAL_GetTick();
      s_valid_frames++;
      return;
    }
  }

  /*
   * 兼容工程早期脚本：$cx,cy,w,h,dist
   * 旧协议没有颜色类别，只能按普通物块(type=1)处理。
   */
  if (sscanf(line, "$%d,%d,%u,%u,%u",
             &center_x,
             &center_y,
             &width,
             &height,
             &distance) == 5)
  {
    if ((center_x >= 0) && (center_x <= 320) &&
        (center_y >= 0) && (center_y <= 240) &&
        (width <= 320U) && (height <= 240U) &&
        (distance <= 1000U))
    {
      s_latest.detected =
        ((width > 0U) && (height > 0U)) ? 1U : 0U;
      s_latest.x_offset_px = (int16_t)(center_x - 160);
      s_latest.y_offset_px = (int16_t)(center_y - 120);
      s_latest.distance_cm = (uint16_t)distance;
      s_latest.object_type =
        (s_latest.detected != 0U) ? 1U : 0U;
      s_latest.timestamp_ms = HAL_GetTick();
      s_valid_frames++;
      return;
    }
  }

  s_invalid_frames++;
}

/* --------------------------------------------------------------------------
 * 【函数】OpenMvUart_Init —— 清零所有状态 + 启动 USART2 中断接收
 * 【额外动作】给 OpenMV 发 "M,0" 切换到色块识别模式 (默认)
 * ------------------------------------------------------------------------- */
void OpenMvUart_Init(void)
{
  memset(&s_latest, 0, sizeof(s_latest));
  s_write_index = 0U;
  s_read_index = 0U;
  s_line_length = 0U;
  s_valid_frames = 0U;
  s_invalid_frames = 0U;
  s_detection_mode = 0xFFU;
  HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
  OpenMvUart_SetDetectionMode(0U);
}

/* --------------------------------------------------------------------------
 * 【函数】OpenMvUart_Process
 * 【作用】主循环调用, 从环形缓冲区取字节拼行, 凑齐 '\n' 就 ParseLine
 * 【为什么不在中断里直接 Parse？】
 *   解析一行要跑 sscanf + 多次范围检查, 耗时太长, 会挡住后续字节的中断
 *   → 中断里只做 "存字节进环形缓冲区", 解析挪到主循环慢慢做
 * ------------------------------------------------------------------------- */
void OpenMvUart_Process(void)
{
  while (s_read_index != s_write_index)
  {
    char byte = (char)s_ring[s_read_index];
    s_read_index = (uint16_t)((s_read_index + 1U) %
                              RX_RING_SIZE);

    if (byte == '\n')
    {
      s_line[s_line_length] = '\0';
      if ((s_line_length > 0U) &&
          (s_line[s_line_length - 1U] == '\r'))
      {
        s_line[s_line_length - 1U] = '\0';
      }
      ParseLine(s_line);
      s_line_length = 0U;
    }
    else if (s_line_length < (LINE_BUFFER_SIZE - 1U))
    {
      s_line[s_line_length++] = byte;
    }
    else
    {
      s_line_length = 0U;
      s_invalid_frames++;
    }
  }
}

/* --------------------------------------------------------------------------
 * 【函数】OpenMvUart_GetLatest
 * 【作用】对外提供最新的视觉识别结果, 带中断安全 + 超时自动清零
 * 【中断安全】关中断拷贝 s_latest 到调用者的 vision (防读到半更新值)
 * 【超时保护】如果 s_latest.timestamp_ms 距离现在超过 500ms,
 *             直接把 vision->detected 清 0, 让上层知道 "摄像头暂时没看到东西"
 * ------------------------------------------------------------------------- */
void OpenMvUart_GetLatest(VisionData_t *vision)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  *vision = s_latest;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((uint32_t)(HAL_GetTick() - vision->timestamp_ms) >
      VISION_STALE_MS)
  {
    vision->detected = 0U;
    vision->x_offset_px = 0;
    vision->y_offset_px = 0;
    vision->distance_cm = 0U;
    vision->object_type = 0U;
  }
}

/* --------------------------------------------------------------------------
 * 【函数】OpenMvUart_SetDetectionMode
 * 【作用】给 OpenMV 发模式切换指令 —— "M,0"=色块识别, "M,1"=黑线识别
 * 【防抖】如果目标模式和当前一样, 直接 return 不重复发指令
 * 【调用方】运动策略根据任务需要自动切换 (比如循迹时切黑线模式)
 * ------------------------------------------------------------------------- */
void OpenMvUart_SetDetectionMode(uint8_t black_area_mode)
{
  static const uint8_t block_command[] = "M,0\n";
  static const uint8_t black_command[] = "M,1\n";
  uint8_t mode = (black_area_mode != 0U) ? 1U : 0U;

  if (mode == s_detection_mode)
  {
    return;
  }

  s_detection_mode = mode;
  if (mode != 0U)
  {
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)black_command,
                      sizeof(black_command) - 1U,
                      10U);
  }
  else
  {
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)block_command,
                      sizeof(block_command) - 1U,
                      10U);
  }
}

/* 只读有效帧计数 (诊断用 —— 看看 OpenMV 发的帧有多少能解析成功) */
uint32_t OpenMvUart_GetValidFrameCount(void)
{
  return s_valid_frames;
}

/* 只读无效帧计数 (诊断用 —— 环形缓冲区溢出 + 超行长 + 协议不匹配都会加 1) */
uint32_t OpenMvUart_GetInvalidFrameCount(void)
{
  return s_invalid_frames;
}

/* --------------------------------------------------------------------------
 * 【函数】OpenMvUart_OnRxComplete —— USART2 接收中断回调
 * 【触发】HAL_UART_RxCpltCallback 路由过来 (stm32f1xx_it.c)
 * 【做的事】把刚收到的 1 字节塞进 s_ring (环形缓冲区), 然后立刻
 *          重启下一次 1 字节中断接收 —— 这样就不会漏字节
 * 【丢包策略】s_ring 满了就丢, s_invalid_frames++ (不阻塞中断, 不丢后续)
 * ------------------------------------------------------------------------- */
void OpenMvUart_OnRxComplete(void)
{
  uint16_t next = (uint16_t)((s_write_index + 1U) %
                             RX_RING_SIZE);

  if (next != s_read_index)
  {
    s_ring[s_write_index] = s_rx_byte;
    s_write_index = next;
  }
  else
  {
    s_invalid_frames++;
  }

  HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

/* --------------------------------------------------------------------------
 * 【函数】OpenMvUart_OnError —— USART2 错误中断回调
 * 【做的事】只清 ORE (Overrun Error) 标志 + 重启接收
 * 【为什么只清 ORE？】ORE 是最常见的 (接收太慢导致上一字节被覆盖),
 *        其他错误 FE/PE/NE 通常不会在这个低速链路出现
 * ------------------------------------------------------------------------- */
void OpenMvUart_OnError(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}
