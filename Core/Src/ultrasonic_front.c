#include "ultrasonic_front.h"
#include "main.h"
#include "tim.h"

#define TRIG_PORT GPIOD
#define TRIG_PIN GPIO_PIN_2
#define ECHO_PORT GPIOD
#define ECHO_PIN GPIO_PIN_12
#define TRIGGER_INTERVAL_MS 60U
#define ECHO_TIMEOUT_MS 30U
#define RESULT_STALE_MS 250U

typedef enum { ULTRA_IDLE = 0, ULTRA_WAIT_RISE, ULTRA_WAIT_FALL } UltraState_t;
static volatile UltraState_t s_state;
static volatile uint16_t s_echo_start_us;
static volatile WallSensorData_t s_latest;
static volatile uint32_t s_result_tick;
static uint32_t s_trigger_tick;

static void DelayUs(uint16_t us)
{
  uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
  while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) - start) < us) {}
}

void UltrasonicFront_Init(void)
{
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
  s_state = ULTRA_IDLE;
  s_latest.distance_cm = 0U;
  s_latest.valid = 0U;
  s_result_tick = 0U;
  s_trigger_tick = HAL_GetTick();
  __HAL_TIM_SET_COUNTER(&htim6, 0U);
  (void)HAL_TIM_Base_Start(&htim6);
}

void UltrasonicFront_Run(uint32_t now_ms)
{
  if (s_state != ULTRA_IDLE &&
      (uint32_t)(now_ms - s_trigger_tick) >= ECHO_TIMEOUT_MS) {
    s_latest.valid = 0U;
    s_latest.distance_cm = 0U;
    s_result_tick = now_ms;
    s_state = ULTRA_IDLE;
  }
  if (s_state == ULTRA_IDLE &&
      (uint32_t)(now_ms - s_trigger_tick) >= TRIGGER_INTERVAL_MS) {
    s_trigger_tick = now_ms;
    s_state = ULTRA_WAIT_RISE;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    DelayUs(10U);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
  }
}

void UltrasonicFront_OnEchoEdge(void)
{
  GPIO_PinState level = HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN);
  if (s_state == ULTRA_WAIT_RISE && level == GPIO_PIN_SET) {
    s_echo_start_us = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
    s_state = ULTRA_WAIT_FALL;
  } else if (s_state == ULTRA_WAIT_FALL && level == GPIO_PIN_RESET) {
    uint16_t end = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
    uint16_t duration_us = (uint16_t)(end - s_echo_start_us);
    uint32_t distance_cm = (uint32_t)duration_us * 343U / 20000U;
    if (distance_cm >= WALL_MIN_VALID_DIST_CM &&
        distance_cm <= WALL_MAX_VALID_DIST_CM) {
      s_latest.distance_cm = (uint16_t)distance_cm;
      s_latest.valid = 1U;
    } else {
      s_latest.distance_cm = 0U;
      s_latest.valid = 0U;
    }
    s_result_tick = HAL_GetTick();
    s_state = ULTRA_IDLE;
  }
}

void UltrasonicFront_GetLatest(WallSensorData_t *wall)
{
  uint32_t primask = __get_PRIMASK();
  uint32_t tick;
  __disable_irq();
  *wall = s_latest;
  tick = s_result_tick;
  if (primask == 0U) __enable_irq();
  if ((uint32_t)(HAL_GetTick() - tick) > RESULT_STALE_MS) {
    wall->valid = 0U;
    wall->distance_cm = 0U;
  }
}
