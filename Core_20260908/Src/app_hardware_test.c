/*
 * 【文件说明】 app_hardware_test.c —— 硬件联调测试模式
 *
 * 【职责】电机 PID + 四路舵机的单独联调测试。与 app_core.c 对称:
 *   - 电机: 可以设 MOTOR_TARGET_MM_S 跑固定转速闭环 (默认 0 = 不转)
 *   - 舵机: 上电后立刻触发四路各执行一次往复, 然后空闲时周期重触发
 *
 * 【什么时候用】
 *   APP_SELECTION = 0 时 main() 跑这个。硬件刚接上板,
 *   还没接 OpenMV / 运动策略 —— 用这个先验证电机 PID 和舵机电路是好的。
 *
 * 【时间节奏】
 *   CONTROL_PERIOD_MS = 10ms —— WheelSpeedControl_Run (电机 PID 闭环一直跑, 即使 target=0)
 *   TELEMETRY_PERIOD_MS = 200ms —— 打印 T,tick,四个速度,四个duty,四个fault
 *
 * 【和 AppCore_Run 的区别】
 *   - 没有 MotionStrategy / OpenMV (视觉不参与)
 *   - TriggerIdleServos 让四路舵机一直循环往复 —— 验证舵机电路
 *   - MOTOR_TARGET_MM_S 默认 0, 想测电机改这个宏
 *
 * 【蓝牙停止态】同样支持 STOP/RESUME —— STOP 时冻结舵机和遥测
 */

#include "app_hardware_test.h"
#include "actuator_servos.h"
#include "dbg_uart.h"
#include "wheel_speed_control.h"
#include "hc05_uart.h"
#include "stm32f1xx_hal.h"

#define MOTOR_TARGET_MM_S        0.0f        /* 固定目标转速, 0=不转 (测舵机时用) */
#define MOTOR_START_DELAY_MS      0U         /* 上电后多久开始跑电机 */
#define CONTROL_PERIOD_MS         10U        /* PID 闭环周期 10ms */
#define TELEMETRY_PERIOD_MS      200U        /* 遥测周期 200ms */

static uint32_t s_start_tick;
static uint32_t s_control_tick;
static uint32_t s_telemetry_tick;

/* --------------------------------------------------------------------------
 * 【函数】TriggerIdleServos —— 让空闲 (非 Busy) 的舵机循环往复
 * 【做的事】依次检查 DOOR / CAMERA / BUCKET / BRUSH,
 *          哪个舵机 IsBusy==0 就给它发 TriggerCycle —— 相当于 "自动循环模式"
 * 【什么时候调】主循环 Run 里, ActuatorServos_Run 之后
 * ------------------------------------------------------------------------- */
static void TriggerIdleServos(uint32_t now_ms)
{
  if (ActuatorServos_IsBusy(ACTUATOR_SERVO_DOOR) == 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_DOOR, now_ms);
  }
  if (ActuatorServos_IsBusy(ACTUATOR_SERVO_CAMERA) == 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_CAMERA, now_ms);
  }
  if (ActuatorServos_IsBusy(ACTUATOR_SERVO_BUCKET) == 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BUCKET, now_ms);
  }
  if (ActuatorServos_IsBusy(ACTUATOR_SERVO_BRUSH) == 0U)
  {
    ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BRUSH, now_ms);
  }
}

/* --------------------------------------------------------------------------
 * 【函数】SendTelemetry —— 硬件测试版遥测, 只看电机速度/duty/fault
 * 【协议】T,tick,spdFL,spdFR,spdRL,spdRR,dutyFL,dutyFR,dutyRL,dutyRR,
 *        faultFL,faultFR,faultRL,faultRR\r\n
 * ------------------------------------------------------------------------- */
static void SendTelemetry(uint32_t now_ms)
{
  DbgUart_Printf(
    "T,%lu,%d,%d,%d,%d,%d,%d,%d,%d,"
    "%u,%u,%u,%u\r\n",
    (unsigned long)now_ms,
    (int)WheelSpeedControl_GetSpeedMmS(WHEEL_FRONT_LEFT),
    (int)WheelSpeedControl_GetSpeedMmS(WHEEL_FRONT_RIGHT),
    (int)WheelSpeedControl_GetSpeedMmS(WHEEL_REAR_LEFT),
    (int)WheelSpeedControl_GetSpeedMmS(WHEEL_REAR_RIGHT),
    (int)WheelSpeedControl_GetDuty(WHEEL_FRONT_LEFT),
    (int)WheelSpeedControl_GetDuty(WHEEL_FRONT_RIGHT),
    (int)WheelSpeedControl_GetDuty(WHEEL_REAR_LEFT),
    (int)WheelSpeedControl_GetDuty(WHEEL_REAR_RIGHT),
    (unsigned int)WheelSpeedControl_GetFault(WHEEL_FRONT_LEFT),
    (unsigned int)WheelSpeedControl_GetFault(WHEEL_FRONT_RIGHT),
    (unsigned int)WheelSpeedControl_GetFault(WHEEL_REAR_LEFT),
    (unsigned int)WheelSpeedControl_GetFault(WHEEL_REAR_RIGHT));
}

/* --------------------------------------------------------------------------
 * 【函数】AppHardwareTest_Init —— 硬件测试版初始化
 * 【做的事】
 *   1. 开调试通道 + 初始化电机 PID + 初始化舵机 + 蓝牙
 *   2. 四路电机都设目标 = MOTOR_TARGET_MM_S (默认 0, 想测改这个)
 *   3. 立刻触发四路舵机各执行一次往复 —— 上电就能看到舵机动
 *   4. 打印 mode=FOUR_MOTOR_PID_AND_FOUR_SERVO_TEST 确认进入测试模式
 * ------------------------------------------------------------------------- */
void AppHardwareTest_Init(void)
{
  uint8_t i;

  DbgUart_Init();
  WheelSpeedControl_Init();
  ActuatorServos_Init();
  Hc05Uart_Init();   /* 启动 USART3 蓝牙中断接收 (HC-05 收电脑端 STOP/RESUME 指令) */

  for (i = 0U; i < WHEEL_COUNT; i++)
  {
    WheelSpeedControl_SetTargetMmS((WheelId_t)i,
                                  MOTOR_TARGET_MM_S);
  }

  s_start_tick = HAL_GetTick();
  s_control_tick = s_start_tick;
  s_telemetry_tick = s_start_tick;

  /* 明确下发四路舵机各执行一次固定往复周期。 */
  ActuatorServos_TriggerCycle(ACTUATOR_SERVO_DOOR, s_start_tick);
  ActuatorServos_TriggerCycle(ACTUATOR_SERVO_CAMERA, s_start_tick);
  ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BUCKET, s_start_tick);
  ActuatorServos_TriggerCycle(ACTUATOR_SERVO_BRUSH, s_start_tick);

  DbgUart_Printf(
    "mode=FOUR_MOTOR_PID_AND_FOUR_SERVO_TEST,"
    "target=%dmm/s\r\n",
    (int)MOTOR_TARGET_MM_S);
}

/* --------------------------------------------------------------------------
 * 【函数】AppHardwareTest_Run —— 硬件测试主循环 (while(1) 每轮调一次)
 * 【做的事】
 *   1. 蓝牙指令解析 (STOP/RESUME)
 *   2. STOP 态: 冻结舵机 + 遥测 (电机 PID 继续跑, target=0)
 *   3. ActuatorServos_Run + TriggerIdleServos (舵机节拍 + 循环触发)
 *   4. 10ms 周期: WheelSpeedControl_Run
 *   5. 200ms 周期: SendTelemetry
 * ------------------------------------------------------------------------- */
void AppHardwareTest_Run(void)
{
  uint32_t now = HAL_GetTick();

  Hc05Uart_Process();          /* 解析蓝牙指令 (STOP/RESUME) */

  /* 蓝牙停止态: 冻结舵机往复和遥测, 电机 duty=0 (StopAll 已停电机) */
  if (Hc05Uart_IsStopped() != 0U)
  {
    return;
  }

  ActuatorServos_Run(now);
  TriggerIdleServos(now);

  if ((uint32_t)(now - s_control_tick) >= CONTROL_PERIOD_MS)
  {
    float dt_s =
      (float)(uint32_t)(now - s_control_tick) / 1000.0f;
    uint8_t enabled =
      ((uint32_t)(now - s_start_tick) >= MOTOR_START_DELAY_MS)
      ? 1U : 0U;
    s_control_tick = now;
    WheelSpeedControl_Run(dt_s, enabled);
  }

  if ((uint32_t)(now - s_telemetry_tick) >=
      TELEMETRY_PERIOD_MS)
  {
    s_telemetry_tick = now;
    SendTelemetry(now);
  }
}
