#ifndef __BOARD_FR_H__
#define __BOARD_FR_H__

#include "tim.h"

/*
 * 四轮电气与机械参数。FR/FL是早期单轮调试遗留模块名；
 * 实车运动学映射统一为：PE9=右前、PE11=右后、PE13=左前、PE14=左后。
 *
 * 依据 1：《智能机电小车 STM32F103VET6 电气连接总表 Rev V7.0》
 * 依据 2：《实验材料说明》第 13/14/16 章（L298N、12V 280rpm 编码电机、12V 电池）
 *
 * 驱动 —— L298N #1 的 A 通道
 *   PE9   TIM1_CH1   ->  ENA   PWM 调速，TIM1 Full Remap，20 kHz，上电 0 占空比
 *   PE0   GPIO_Out   ->  IN1   方向控制 1
 *   PE1   GPIO_Out   ->  IN2   方向控制 2
 *   ENA 需移除跳帽并保留 10 kΩ 下拉，保证上电默认禁用
 *
 * 反馈 —— 12V/280rpm 减速电机自带增量编码器，TIM2 Encoder TI1+TI2（×4 解码）
 *   PA15  TIM2_CH1   <-  A 相     TIM2 Partial Remap 1，IC Filter = 6
 *   PB3   TIM2_CH2   <-  B 相
 *   红 = +5V_ENC，黑 = GND，A/B 必须统一转换到 3.3 V 后接入
 *   占用 PA15/PB3 要求 SYS = Serial Wire 并关闭完整 JTAG
 *
 * PE11通道（整车映射为左后）
 *   PE11  TIM1_CH2   ->  L298N #1 ENB
 *   PE2   GPIO_Out   ->  IN3
 *   PE3   GPIO_Out   ->  IN4
 *   PB6   TIM4_CH1   <-  编码器 A 相
 *   PB7   TIM4_CH2   <-  编码器 B 相
 *
 * OpenMV —— USART2，115200 8N1，PA2 = TX / PA3 = RX
 */

/* ---- 实车右前轮：TIM1_CH1 / PE9（遗留模块名FR） ---- */
#define FR_PWM_TIM              (&htim1)
#define FR_PWM_CHANNEL          TIM_CHANNEL_1
#define FR_PWM_PORT             GPIOE
#define FR_PWM_PIN              GPIO_PIN_9

/* MX_TIM1_Init 的 ARR。72 MHz / (3599 + 1) = 20 kHz，占空比分辨率即为该值 */
#define FR_PWM_MAX              3599

/* ---- 方向：PE0 = IN1，PE1 = IN2 ---- */
#define FR_IN1_PORT             GPIOE
#define FR_IN1_PIN              GPIO_PIN_0
#define FR_IN2_PORT             GPIOE
#define FR_IN2_PIN              GPIO_PIN_1
#define FR_DRIVE_SIGN           (-1)

/* ---- 实车右后轮：TIM1_CH2 / PE11（遗留模块名FL），PE2/PE3方向 ---- */
#define FL_PWM_TIM              (&htim1)
#define FL_PWM_CHANNEL          TIM_CHANNEL_2
#define FL_PWM_PORT             GPIOE
#define FL_PWM_PIN              GPIO_PIN_11

#define FL_IN1_PORT             GPIOE
#define FL_IN1_PIN              GPIO_PIN_2
#define FL_IN2_PORT             GPIOE
#define FL_IN2_PIN              GPIO_PIN_3
/* PE9/PE11 位于实车同一侧；按实测反转这一路的电机输出极性。 */
#define FL_DRIVE_SIGN           (+1)

/* ---- 实车左前轮：PE13/TIM1_CH3，PE4/PE5方向，TIM5编码器 ---- */
#define PE13_PWM_TIM            (&htim1)
#define PE13_PWM_CHANNEL        TIM_CHANNEL_3
#define PE13_PWM_PORT           GPIOE
#define PE13_PWM_PIN            GPIO_PIN_13
#define PE13_IN1_PORT           GPIOE
#define PE13_IN1_PIN            GPIO_PIN_4
#define PE13_IN2_PORT           GPIOE
#define PE13_IN2_PIN            GPIO_PIN_5
#define PE13_DRIVE_SIGN         (-1)
#define PE13_ENC_TIM            (&htim5)
#define PE13_ENC_SIGN           (-1)

/* ---- 实车左后轮：PE14/TIM1_CH4，PE6/PE7方向，TIM8编码器 ---- */
#define PE14_PWM_TIM            (&htim1)
#define PE14_PWM_CHANNEL        TIM_CHANNEL_4
#define PE14_PWM_PORT           GPIOE
#define PE14_PWM_PIN            GPIO_PIN_14
#define PE14_IN1_PORT           GPIOE
#define PE14_IN1_PIN            GPIO_PIN_6
#define PE14_IN2_PORT           GPIOE
#define PE14_IN2_PIN            GPIO_PIN_7
#define PE14_DRIVE_SIGN         (+1)
#define PE14_ENC_TIM            (&htim8)
#define PE14_ENC_SIGN           (+1)

/* PE9/PE11双通道调试时用于关闭PE13/PE14；四路初始化会重新启用它们 */
#define REAR_EN_PORT            GPIOE
#define REAR_EN_PINS            (GPIO_PIN_13 | GPIO_PIN_14)

/* 两个后轮方向引脚一并压低 */
#define REAR_IN_PORT            GPIOE
#define REAR_IN_PINS            (GPIO_PIN_4 | GPIO_PIN_5 | \
                                 GPIO_PIN_6 | GPIO_PIN_7)

/* ---- 编码器：TIM2 ---- */
#define FR_ENC_TIM              (&htim2)
#define FL_ENC_TIM              (&htim4)

/* 各轮A/B接反时单独改成-1，使车辆前进方向的计数为正 */
#define FR_ENC_SIGN             (+1)
#define FL_ENC_SIGN             (-1)

/*
 * 每输出轴一圈的计数值 = 每相线数 × 4（TI1+TI2 四倍频）× 减速比
 *   11 × 4 × 21.3 = 937.2
 * 电气表要求 PPR 实测后回填；若实测线数不是 11，只改 FR_ENC_LINES
 */
#define FR_ENC_LINES            11.0f
#define FR_GEAR_RATIO           21.3f
#define FR_ENC_COUNTS_PER_REV   (FR_ENC_LINES * 4.0f * FR_GEAR_RATIO)

/* ---- 机械 ---- */
#define FR_WHEEL_DIAMETER_MM    65.0f
#define FR_WHEEL_CIRC_MM        (3.14159265f * FR_WHEEL_DIAMETER_MM)

/*
 * 12 V 空载 280 rpm（输出轴）。L298N 自身约 2 V 压降，装车带载后
 * 实际可达转速会低于该值，这里只作为量程与限幅的参考上限。
 *   280 rpm ≈ 4.67 r/s ≈ 953 mm/s
 */
#define FR_RATED_RPM            280.0f
#define FR_RATED_MM_S           (FR_RATED_RPM / 60.0f * FR_WHEEL_CIRC_MM)

/* ---- 控制周期 ---- */
#define FR_LOOP_MS              10U
#define FR_LOOP_DT_S            0.01f

#endif /* __BOARD_FR_H__ */
