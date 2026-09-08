#ifndef __MOTOR_TEST_CONFIG_H__
#define __MOTOR_TEST_CONFIG_H__

/*
 * 四电机调试总开关
 *
 * 每个接口只改对应的 CONTROL_MODE：
 *   0 = 完全关闭，PWM和方向引脚保持低
 *   1 = 开环，使用 OPEN_DUTY_PERCENT
 *   2 = PID闭环，使用 TARGET_MM_S
 *
 * 首次接入新的电机/编码器时应先选1，确认方向和编码器符号后再选2。
 */
#define MOTOR_MODE_OFF          0U
#define MOTOR_MODE_OPEN_LOOP    1U
#define MOTOR_MODE_PID          2U

/* PE9 / TIM1_CH1，方向PE0/PE1，编码器TIM2 PA15/PB3 */
#define PE9_CONTROL_MODE        MOTOR_MODE_OPEN_LOOP
#define PE9_OPEN_DUTY_PERCENT   50
#define PE9_TARGET_MM_S         200.0f
#define PE9_DRIVE_SIGN          (-1)
#define PE9_ENCODER_SIGN        (-1)

/* PE11 / TIM1_CH2，方向PE2/PE3，编码器TIM4 PB6/PB7；当前接左后轮 */
#define PE11_CONTROL_MODE       MOTOR_MODE_OPEN_LOOP
#define PE11_OPEN_DUTY_PERCENT  50
#define PE11_TARGET_MM_S        200.0f
#define PE11_DRIVE_SIGN         (+1)
#define PE11_ENCODER_SIGN       (+1)

/* PE13 / TIM1_CH3，方向PE4/PE5，编码器TIM5 PA0/PA1 */
#define PE13_CONTROL_MODE       MOTOR_MODE_OFF
#define PE13_OPEN_DUTY_PERCENT  50
#define PE13_TARGET_MM_S        200.0f
#define PE13_DRIVE_SIGN         (+1)
#define PE13_ENCODER_SIGN       (+1)

/* PE14 / TIM1_CH4，方向PE6/PE7，编码器TIM8 PC6/PC7 */
#define PE14_CONTROL_MODE       MOTOR_MODE_OFF
#define PE14_OPEN_DUTY_PERCENT  50
#define PE14_TARGET_MM_S        200.0f
#define PE14_DRIVE_SIGN         (+1)
#define PE14_ENCODER_SIGN       (+1)

/* 通用控制参数 */
#define MOTOR_START_DELAY_MS    1000U
#define MOTOR_PID_BOOST_MS      800U
#define MOTOR_PID_BOOST_PERCENT 60
#define MOTOR_CONTROL_PERIOD_MS 10U
#define MOTOR_PRINT_PERIOD_MS   200U

#define MOTOR_PID_FF_DUTY       1200.0f
#define MOTOR_PID_KP            2.0f
#define MOTOR_PID_KI            8.0f
#define MOTOR_PID_KD            0.005f
#define MOTOR_SPEED_FILTER      0.25f
#define MOTOR_D_FILTER          0.20f
#define MOTOR_PID_I_MIN         (-1000.0f)
#define MOTOR_PID_I_MAX         1800.0f
#define MOTOR_PID_OUTPUT_MAX    3000.0f

/* 放宽后的安全保护 */
#define MOTOR_REVERSE_MM_S      50.0f
#define MOTOR_REVERSE_TICKS     50U
#define MOTOR_NO_FB_GRACE_TICKS 300U
#define MOTOR_NO_FB_DUTY        2700
#define MOTOR_NO_FB_MM_S        10.0f
#define MOTOR_NO_FB_TICKS       500U

#endif /* __MOTOR_TEST_CONFIG_H__ */
