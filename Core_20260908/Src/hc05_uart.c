#include "hc05_uart.h"
#include "motion_config.h"
#include "usart.h"
#include "dbg_uart.h"
#include <string.h>

static uint8_t s_rx_byte;
static volatile uint8_t s_ring[HC05_RX_RING_SIZE];
static volatile uint16_t s_write_index;
static volatile uint16_t s_read_index;
static char s_line[HC05_LINE_BUFFER_SIZE];
static uint8_t s_line_length;
static volatile Hc05Event_t s_event;
static uint32_t s_valid;
static uint32_t s_invalid;

typedef struct { const char *name; Hc05Event_t event; } CommandMap_t;
static const CommandMap_t s_commands[] = {
  {"AUTO", HC05_EVENT_AUTO}, {"MANUAL", HC05_EVENT_MANUAL},
  {"FORWARD", HC05_EVENT_FORWARD}, {"BACKWARD", HC05_EVENT_BACKWARD},
  {"LEFT", HC05_EVENT_LEFT}, {"RIGHT", HC05_EVENT_RIGHT},
  {"ROTATE_CW", HC05_EVENT_ROTATE_CW},
  {"ROTATE_CCW", HC05_EVENT_ROTATE_CCW},
  {"SERVO_DOOR", HC05_EVENT_SERVO_DOOR},
  {"SERVO_CAMERA", HC05_EVENT_SERVO_CAMERA},
  {"SERVO_BUCKET", HC05_EVENT_SERVO_BUCKET},
  {"SERVO_BRUSH", HC05_EVENT_SERVO_BRUSH},
  {"SET_HOME", HC05_EVENT_SET_HOME},
  {"ROTATE180", HC05_EVENT_ROTATE_180},
  {"ROTATE360", HC05_EVENT_ROTATE_360},
  {"STATUS", HC05_EVENT_STATUS}
};

void Hc05Uart_SendAck(const char *command)
{
  DbgUart_Print("ACK,");
  DbgUart_Print(command);
  DbgUart_Print("\n");
}

static void ParseLine(const char *line)
{
  uint32_t i;
  /*
   * STOP 是最高优先级安全命令：即使已有普通事件尚未被主循环消费，
   * 也必须覆盖它，不能因单槽事件队列繁忙而丢失。
   */
  if (strcmp(line, "STOP") == 0) {
    s_event = HC05_EVENT_STOP;
    s_valid++;
    Hc05Uart_SendAck("STOP");
    return;
  }
  if (strcmp(line, "HELLO") == 0) {
    s_valid++;
    Hc05Uart_SendAck("HELLO");
    return;
  }
  for (i = 0U; i < (sizeof(s_commands) / sizeof(s_commands[0])); i++) {
    if (strcmp(line, s_commands[i].name) == 0) {
      /* 单槽事件队列：主循环尚未消费时拒绝覆盖，并明确返回 BUSY。 */
      if (s_event != HC05_EVENT_NONE) {
        DbgUart_Print("ACK,BUSY\n");
        return;
      }
      s_event = s_commands[i].event;
      s_valid++;
      Hc05Uart_SendAck(line);
      return;
    }
  }
  s_invalid++;
  DbgUart_Print("ACK,INVALID\n");
}

void Hc05Uart_Init(void)
{
  s_write_index = s_read_index = 0U;
  s_line_length = 0U;
  s_event = HC05_EVENT_NONE;
  s_valid = s_invalid = 0U;
  (void)HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
}

void Hc05Uart_Process(void)
{
  while (s_read_index != s_write_index) {
    char byte = (char)s_ring[s_read_index];
    s_read_index = (uint16_t)((s_read_index + 1U) % HC05_RX_RING_SIZE);
    if (byte == '\n') {
      if (s_line_length && s_line[s_line_length - 1U] == '\r') {
        s_line_length--;
      }
      s_line[s_line_length] = '\0';
      ParseLine(s_line);
      s_line_length = 0U;
    } else if (s_line_length < HC05_LINE_BUFFER_SIZE - 1U) {
      s_line[s_line_length++] = byte;
    } else {
      s_line_length = 0U;
      s_invalid++;
    }
  }
}

Hc05Event_t Hc05Uart_TakeEvent(void)
{
  Hc05Event_t event;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  event = s_event;
  s_event = HC05_EVENT_NONE;
  if (primask == 0U) { __enable_irq(); }
  return event;
}

void Hc05Uart_OnRxComplete(void)
{
  uint16_t next = (uint16_t)((s_write_index + 1U) % HC05_RX_RING_SIZE);
  if (next != s_read_index) {
    s_ring[s_write_index] = s_rx_byte;
    s_write_index = next;
  } else {
    s_invalid++;
  }
  (void)HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
}

void Hc05Uart_OnError(void)
{
  __HAL_UART_CLEAR_OREFLAG(&huart4);
  (void)HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
}

uint32_t Hc05Uart_GetValidCount(void) { return s_valid; }
uint32_t Hc05Uart_GetInvalidCount(void) { return s_invalid; }
uint8_t Hc05Uart_IsStopped(void) { return 0U; }
