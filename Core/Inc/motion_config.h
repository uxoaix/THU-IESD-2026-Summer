#ifndef __MOTION_CONFIG_H__
#define __MOTION_CONFIG_H__

#include <stdint.h>

/*
 * 【总览】系统统一参数配置中心
 *   本文件是 movemethod / openmv / 电机舵机三团队的唯一参数集中地。
 *   命名规范: 类型小写 _t 后缀; 枚举值大写+模块前缀; 字段带单位后缀
 *            (_cm_s / _cm_s2 / _cm / _px / _ms); 宏 U/f 后缀, 参数加括号。
 *
 *   §1-§4  原有运动策略参数 (已统一命名)
 *   §5-§10 movemethod 运动子模块参数 (从 my_code/movemethod/config.h 迁入)
 *   §11-§15 IMU/超声/传感器融合参数 (movemethod 子模块)
 *   §16     摄像头俯仰跟踪 (预留, 需 ActuatorServos 扩展 SetAngle API)
 *   §17     执行器舵机循环参数 (从 actuator_servos.c 抽取)
 *   §18     电机 PID 参数 (从 wheel_speed_control.c / app_fr.c 抽取)
 *   §19     OpenMV 协议常量 (STM32 端; OpenMV 侧 Python 参数见注释)
 *   §20     通用宏
 */

/* 1. 车轮 ID 枚举 */
typedef enum {
  WHEEL_FRONT_LEFT = 0,
  WHEEL_FRONT_RIGHT,
  WHEEL_REAR_LEFT,
  WHEEL_REAR_RIGHT,
  WHEEL_COUNT
} WheelId_t;

/* 2. 运动状态机定义 (16 态) */
typedef enum {
  MOTION_STATE_INIT = 0,
  MOTION_STATE_ROTATE_SEARCH,
  MOTION_STATE_PRE_CENTERING,
  MOTION_STATE_TARGET_TRACKING,
  MOTION_STATE_BRUSH_COLLECT,
  MOTION_STATE_SEARCH_CONTINUE,
  MOTION_STATE_WALL_SCAN_FIRST,
  MOTION_STATE_WALL_SCAN_SECOND,
  MOTION_STATE_WALL_TURN_OPPOSITE,
  MOTION_STATE_WALL_SMALL_ROTATE,
  MOTION_STATE_WALL_MEASURE_MOVE,
  MOTION_STATE_CAMERA_ROTATE,
  MOTION_STATE_BLACK_AREA_SEARCH,
  MOTION_STATE_BLACK_AREA_TRACK,
  MOTION_STATE_BLACK_AREA_PROCESS,
  MOTION_STATE_UNLOADING
} MotionState_t;

/* 上电控制模式: 0=手动，1=自动；运行中可用 MANUAL/AUTO 切换。 */
#ifndef DEFAULT_CONTROL_MODE
#define DEFAULT_CONTROL_MODE                1U
#endif

/* 3. 视觉输入数据结构 (OpenMV → STM32)
 *   字段单位在两端保持一致; timestamp_ms 由 STM32 接收时打戳。 */
typedef struct {
  uint8_t  detected;        /* 1=看到目标, 0=没看到 */
  int16_t  x_offset_px;     /* 目标水平偏移 (像素, 负=左, 正=右, 0=居中) */
  int16_t  y_offset_px;     /* 目标垂直偏移 (像素, 负=上, 正=下) */
  uint16_t distance_cm;     /* 目标距离 (cm) */
  uint8_t  object_type;      /* 1/2=红/黄物块, 3=黑色卸货区, 0=未知 */
  uint32_t timestamp_ms;     /* STM32 接收时间戳 (ms) */
} VisionData_t;

/* 4. 车轮反馈数据结构 (编码器 → 运动策略) */
typedef struct {
  float speed_cm_s[WHEEL_COUNT];   /* 实际轮速 (cm/s, 正=前进) */
  float accel_cm_s2[WHEEL_COUNT];  /* 实际轮加速度 (cm/s²) */
} WheelFeedback_t;

/* 5. 超声波传感器输入结构 */
typedef struct {
  uint16_t distance_cm;   /* 前方距离 (cm) */
  uint8_t  valid;          /* 1=有效, 0=超出量程/通信异常 */
} WallSensorData_t;

/* 6. 运动策略输出结构 (运动策略 → 电机驱动/舵机)
 *   target_accel_cm_s2 为前馈量, 当前 WheelSpeedControl 仅消费 target_speed,
 *   accel 字段保留待未来 PID 前馈启用 (见 §18)。 */
typedef struct {
  float    target_speed_cm_s[WHEEL_COUNT];  /* 目标轮速 (cm/s) */
  float    target_accel_cm_s2[WHEEL_COUNT]; /* 目标轮加速度 (cm/s², 预留前馈) */
  uint8_t  brush_cycle;     /* 滚刷触发一次 (1=触发) */
  uint8_t  bucket_cycle;    /* 后斗升降触发一次 */
  uint8_t  door_cycle;      /* 后门开闭触发一次 */
  uint8_t  camera_cycle;    /* 摄像头视野切换触发一次 */
  uint8_t  detect_black_area; /* 1=切到黑区检测模式, 0=物块模式 */
} MotionCommand_t;

/* 7. 状态机周期与超时 */
#define MOTION_PERIOD_MS                 50U
#define SEARCH_NO_TARGET_TIMEOUT_MS       10000U
#define BLACK_SEARCH_TIMEOUT_MS          10000U
#define TARGET_LOCK_WINDOW_MS            800U
#define PRE_CENTERING_TIMEOUT_MS         2000U

/* 8. 速度与运动控制参数 */
#define MAX_LINEAR_SPEED_CM_S            50.0f
#define MAX_ANGULAR_SPEED_RAD_S          1.0f      /* 最大角速度 (rad/s) */
#define MAX_WHEEL_ACCEL_CM_S2            100.0f
#define SEARCH_ROTATION_SPEED_DEG_S      12.0f
#define TRACKING_LINEAR_SPEED_CM_S       25.0f
#define COLLECT_LINEAR_SPEED_CM_S        8.0f
#define WALL_APPROACH_SPEED_CM_S         15.0f
#define MANUAL_LINEAR_SPEED_CM_S         20.0f
#define MANUAL_ROTATE_SPEED_DEG_S        20.0f
#define MANUAL_TURN_LINEAR_RATIO         0.55f

/* 9. 视觉追踪参数 */
#define TRACKING_DEADZONE_PX             15
#define TARGET_DISTANCE_MAX_CM           50U
#define COLLECT_DISTANCE_CM              10U
#define BLACK_AREA_ARRIVE_CM             10U
#define TRACKING_SLOW_RATIO              0.8f
#define BLACK_AREA_OBJECT_TYPE           3U
#define TOTAL_OBJECTS_TO_COLLECT         5U

/* 10. 模糊控制参数 (block_alignment 子模块, 物块对准)
 *
 *   输入: x_offset_px ([-160, +160], 负=左, 正=右)
 *   输出: 角速度修正 (rad/s, 正=逆时针/左转, 负=顺时针/右转)
 *   规则: x偏左→ω正(左转CCW), x偏右→ω负(右转CW), x=0→ω=0 */
#define FUZZY_X_OFFSET_MAX_PX            160
#define FUZZY_MF_HALF_WIDTH_PX           40
/* 隶属函数中心 (等距 40px 间隔) */
#define FUZZY_CENTER_NB                  (-120)
#define FUZZY_CENTER_NM                  (-80)
#define FUZZY_CENTER_NS                  (-40)
#define FUZZY_CENTER_ZE                  (0)
#define FUZZY_CENTER_PS                  (40)
#define FUZZY_CENTER_PM                  (80)
#define FUZZY_CENTER_PB                  (120)
/* 输出 singleton (rad/s) */
#define FUZZY_OUT_NB_RAD_S               (-0.5f)   /* 大幅右转 (CW) */
#define FUZZY_OUT_NM_RAD_S               (-0.3f)
#define FUZZY_OUT_NS_RAD_S               (-0.15f)
#define FUZZY_OUT_ZE_RAD_S               (0.0f)
#define FUZZY_OUT_PS_RAD_S               (0.15f)
#define FUZZY_OUT_PM_RAD_S               (0.3f)
#define FUZZY_OUT_PB_RAD_S               (0.5f)    /* 大幅左转 (CCW) */

/* 11. 车轮与编码器参数 */
#define WHEEL_DIAMETER_CM                6.5f
#define WHEEL_CIRCUMFERENCE_CM           (3.14159f * WHEEL_DIAMETER_CM)
#define WHEEL_TRACK_WIDTH_CM              22.4f      /* 左右轮中心距 (cm) */
#define WHEEL_BASE_CM                     9.7f      /* 前后轮中心距 (cm) */
#define ENCODER_PULSES_PER_REV            480        /* 每转脉冲数 */
#define ODOMETRY_STATIC_DEADZONE_CM_S     0.5f      /* 编码器静止零漂死区 (cm/s) */

/* 12. 墙体扫描参数 */
#define WALL_MIN_VALID_DIST_CM           5U
#define WALL_MAX_VALID_DIST_CM           300U
#define WALL_SAMPLE_INTERVAL_MS          100U
#define WALL_SCAN_TOTAL_DEG              360        /* 扫描总角度 (度) */
#define WALL_SCAN_ROTATION_SPEED_DEG_S   30.0f      /* 墙扫/找方向专用转速 */
#define WALL_SCAN_TIMEOUT_MS             18000U     /* 30°/s 完整一圈需12s */
#define WALL_MEASURE_TIMEOUT_MS          1500U
#define WALL_MOVE_TIMEOUT_MS             15000U
#define WALL_HEADING_TOL_DEG             8.0f
#define WALL_SMALL_ROTATE_DEG            15.0f
#define WALL_SMALL_ROTATE_TOL_DEG        1.0f
#define WALL_TRAVEL_NUMERATOR            2U
#define WALL_TRAVEL_DENOMINATOR           3U

/* 13. 滚刷控制参数 */
#define BRUSH_ROTATE_DURATION_MS         (BRUSH_OUTBOUND_MS + BRUSH_RETURN_MS)
#define BRUSH_TIMEOUT_MS                 3500U     /* 滚刷动作超时保护 */
#define COLLECT_BUCKET_DELAY_MS          150U      /* 滚刷完成后再启动后斗 */

/* 14. 后斗卸载参数 */
#define BUCKET_LIFT_DURATION_MS          2000U
#define BUCKET_UNLOAD_DURATION_MS        4000U
#define BUCKET_DOOR_CLOSE_MS             500U
#define BUCKET_LOWER_DURATION_MS         2000U

/* 15. 摄像头舵机参数 (180° 视野切换) */
#define CAMERA_ROTATE_DURATION_MS        1000U
#define CAMERA_SERVO_TIMEOUT_MS          2500U     /* 舵机超时保护 (ms) */
#define CAMERA_ANGLE_BLOCK_DEG           0U        /* 物块阶段角度 */
#define CAMERA_ANGLE_BLACK_DEG           180U      /* 黑色区域阶段角度 */

/* 16. HC-SR04 硬件驱动参考参数
 *   【注意】实际未使用: 集成版用 ultrasonic_front.c (PD2/PD12, EXTI 非阻塞)。
 *   以下保留 movemethod 原值仅作参考, 引脚 PB0/PB1 与舵机冲突。 */
#define HCSR04_TRIG_PIN                  GPIO_PIN_0
#define HCSR04_TRIG_PORT                 GPIOB
#define HCSR04_ECHO_PIN                  GPIO_PIN_1
#define HCSR04_ECHO_PORT                 GPIOB
#define HCSR04_CPU_FREQ_MHZ              72
#define HCSR04_TRIG_PULSE_US             10
#define HCSR04_SOUND_SPEED_CM_PER_US     0.0343f
#define HCSR04_DIST_FACTOR               (HCSR04_SOUND_SPEED_CM_PER_US / 2.0f)
#define HCSR04_ECHO_WAIT_TIMEOUT         2000000u
#define HCSR04_MEASURE_INTERVAL_MS       60
#define HCSR04_FILTER_SAMPLE_COUNT       3
#define HCSR04_FILTER_INTERVAL_MS        20
#define HCSR04_INVALID_DISTANCE         (-1.0f)

/* 17. 维特(Wit) IMU 硬件驱动参数 (UART 通信)
 *   【重要】IMU 使用 USART3 (PB10/PB11), 与 OpenMV (USART2 @115200) 物理隔离。
 *   usart.c 与 IMU_Init 均按实测 115200 配置。 */
#define IMU_UART_INSTANCE                USART3
#define IMU_UART_IRQn                    USART3_IRQn
#define IMU_UART_IRQ_PRIORITY            0
#define IMU_UART_BAUDRATE                115200
#define IMU_CFG_TX_TIMEOUT_MS            50
#define IMU_DATA_READY_TIMEOUT_MS        500

/* 维特协议帧格式 (固定 11 字节: [0x55][type][d0..d7][sum]) */
#define WIT_FRAME_LEN                    11
#define WIT_FRAME_HEADER                 0x55
#define WIT_TYPE_ACCEL                   0x51
#define WIT_TYPE_GYRO                    0x52
#define WIT_TYPE_ANGLE                    0x53

/* 维特配置命令帧 (5 字节: [0xFF][0xAA][reg][dataL][dataH]) */
#define WIT_CFG_CMD_LEN                  5
#define WIT_CFG_HEADER0                  0xFF
#define WIT_CFG_HEADER1                  0xAA
#define WIT_REG_SAVE                     0x00
#define WIT_REG_CALIB                    0x01
#define WIT_REG_OUTRATE                  0x03
#define WIT_REG_BAUD                     0x04

/* 18. IMU 物理量换算参数 (imu_processor 用) */
#define IMU_ACCEL_RANGE_G                16.0f
#define IMU_GYRO_RANGE_DPS               2000.0f
#define IMU_ANGLE_RANGE_DEG              180.0f
#define IMU_ACCEL_LSB_PER_G              (32768.0f / IMU_ACCEL_RANGE_G)
#define IMU_GYRO_LSB_PER_DPS             (32768.0f / IMU_GYRO_RANGE_DPS)
#define IMU_ANGLE_LSB_PER_DEG            (32768.0f / IMU_ANGLE_RANGE_DEG)
#define IMU_TEMP_LSB_PER_C               100.0f
#define IMU_GYRO_STATIC_DPS              5.0f
#define IMU_GYRO_BIAS_SAMPLES            32u
#define IMU_GYRO_BIAS_MAX_DPS            3.0f

/* 19. 传感器融合参数 (sensor_fusion 用)
 *
 *     ① 互补滤波 (COMPLEMENTARY): 陀螺短期预测 + 绝对源长期修正。
 *     ② 卡尔曼滤波 (KALMAN): 2 状态[航向, 角速度]最优估计, 3 路测量更新。
 *
 *     FUSION_INTEGRATION_ENABLED:
 *       0 = 不接入 motion_strategy (保持编码器里程计, 默认)
 *       1 = 接入: 在 motion_strategy.c 的 Odometry_Update 后调用
 *           SensorFusion_Update + SensorFusion_GetHeading 覆盖编码器航向。
 *           启用前需先 IMU_Init + IMUProcessor_Init + SensorFusion_Init。 */
#define FUSION_DEFAULT_MODE              2              /* 0=OFF,1=COMPLEMENTARY,2=KALMAN */
#define FUSION_INTEGRATION_ENABLED       1              /* 自动任务启用，断线自动退化到编码器 */
#define FUSION_UPDATE_PERIOD_MS          MOTION_PERIOD_MS
/* 互补滤波系数 */
#define FUSION_COMP_ALPHA                0.95f
#define FUSION_COMP_GAMMA                0.5f
/* IMU 航向符号 (校准用): 若融合航向往反方向跑, 改为 -1.0f */
#define FUSION_IMU_YAW_SIGN              1.0f
/* 卡尔曼噪声参数 (方差, 越大越不信任该源) */
#define FUSION_KF_Q_ACCEL                0.01f
#define FUSION_KF_R_ENC_RAD              0.02f
#define FUSION_KF_R_IMU_YAW_RAD          0.005f
#define FUSION_KF_R_IMU_GYRO_RAD         0.001f
#define FUSION_KF_P0_HEADING             0.1f
#define FUSION_KF_P0_RATE                0.1f
#define FUSION_VEL_ACCEL_WEIGHT          0.0f          /* IMU 加速度预测速度权重 (0=全信编码器) */

/* 兼容旧 block_alignment 接口；自动状态机不再调用摄像头俯仰跟踪。 */
#define CAMERA_TILT_DEADZONE_PX          15
#define CAMERA_TILT_KP_DEG_PER_PX        0.2f
#define CAMERA_TILT_MAX_STEP_DEG         2
#define CAMERA_TILT_CENTER_DEG           90
#define CAMERA_TILT_MIN_DEG              60
#define CAMERA_TILT_MAX_DEG              120

/* 21. 执行器舵机循环参数 (actuator_servos 用)
 *   四路舵机固定执行 start_deg → end_deg → start_deg 往复; 上层只触发一次。
 *   舵机引脚: DOOR=PA6/TIM3_CH1, CAMERA=PA7/TIM3_CH2,
 *             BUCKET=PB0/TIM3_CH3, BRUSH=PB1/TIM3_CH4。 */
#define SERVO_MIN_PULSE_US               500U
#define SERVO_MAX_PULSE_US               2500U
#define SERVO_ANGLE_LIMIT_DEG            180U

#define DOOR_START_DEG                   0U
#define DOOR_END_DEG                     180U
#define DOOR_OUTBOUND_MS                 1000U
#define DOOR_RETURN_MS                   1000U

#define CAMERA_START_DEG                 0U
#define CAMERA_END_DEG                   180U
#define CAMERA_OUTBOUND_MS               1000U
#define CAMERA_RETURN_MS                 1000U

#define BUCKET_START_DEG                 0U
#define BUCKET_END_DEG                   180U
#define BUCKET_OUTBOUND_MS               1000U
#define BUCKET_RETURN_MS                 1000U

#define BRUSH_START_DEG                  0U
#define BRUSH_END_DEG                    180U
#define BRUSH_OUTBOUND_MS                1000U
#define BRUSH_RETURN_MS                  1000U

/* 22. 电机 PID 参数 (wheel_speed_control 用, 从 app_fr.c 联调值迁入)
 *   轮速闭环: 目标速度(mm/s) → 占空比; START_BOOST_DUTY 依赖 FR_PWM_MAX (board_fr.h)。 */
#define WHEEL_PID_KP                     2.0f
#define WHEEL_PID_KI                     8.0f
#define WHEEL_PID_KD                     0.005f
#define WHEEL_PID_FEEDFORWARD_DUTY       1200.0f
#define WHEEL_SPEED_FILTER_ALPHA         0.25f
#define WHEEL_D_FILTER_ALPHA             0.20f
#define WHEEL_PID_I_MIN                  (-1000.0f)
#define WHEEL_PID_I_MAX                  1800.0f
#define WHEEL_PID_OUTPUT_MAX             3000.0f
#define WHEEL_START_BOOST_MS             800U
#define WHEEL_START_BOOST_DUTY_PCT       60         /* 占 FR_PWM_MAX 的百分比 */
/* 保护阈值 */
#define WHEEL_REVERSE_LIMIT_MM_S         50.0f
#define WHEEL_REVERSE_LIMIT_TICKS        50U
#define WHEEL_NO_FB_GRACE_TICKS          300U
#define WHEEL_NO_FB_DUTY                 2700
#define WHEEL_NO_FB_SPEED_MM_S           10.0f
#define WHEEL_NO_FB_LIMIT_TICKS         500U

/* 23. OpenMV 协议常量 (openmv_uart 用)
 *   OpenMV → STM32 帧: "V,detected,x_offset_px,y_offset_px,distance_cm,object_type\n"
 *   STM32 → OpenMV 命令: "M,0\n"=物块模式, "M,1\n"=黑色卸货区模式
 *   OpenMV 端 (Python, 运行在摄像头上, STM32 无法动态加载) 参数参考:
 *     FOCAL_LENGTH_PX = 480.0, IMAGE_WIDTH=320, IMAGE_HEIGHT=240
 *     BLOCK_THRESHOLD = (81,99,-28,-7,-33,93) [LAB, 需现场重新标定]
 *     BLACK_THRESHOLD  = (0,35,-20,20,-20,20)
 *     BLOCK_WIDTH_CM=8.0, BLACK_AREA_WIDTH_CM=20.0
 *   这些值请直接改 Core\my_code\openmv\uart_to_stm32.py。 */
#define OPENMV_FRAME_PREFIX              'V'
#define OPENMV_MODE_PREFIX               'M'
#define OPENMV_MODE_BLOCK                "M,0\n"
#define OPENMV_MODE_BLACK                "M,1\n"
#define OPENMV_OBJECT_TYPE_UNKNOWN       0U
#define OPENMV_OBJECT_TYPE_RED            1U
#define OPENMV_OBJECT_TYPE_YELLOW        2U
#define OPENMV_OBJECT_TYPE_BLACK_AREA    3U
#define OPENMV_UART_INSTANCE             USART2
#define OPENMV_UART_IRQn                 USART2_IRQn
#define OPENMV_BAUDRATE                  115200U
#define OPENMV_VISION_STALE_MS           500U       /* 超此时间视为丢失目标 */
#define OPENMV_RX_RING_SIZE              128U
#define OPENMV_LINE_BUFFER_SIZE          64U

/* 25. HC-05 蓝牙控制参数 (hc05_uart 用)
 *   电脑端 → HC-05 (UART4@115200 透传, PC10/PC11) → STM32
 *   指令: STOP=停车, GOFOR=直行, GOLEFT=左转, GORIGHT=右转,
 *         RESUME=退出手动控制并恢复自动运行
 *   每条指令均须以 '\n' 结尾，执行后经 dbg_uart 回显 ACK
 *   UART4 是完全空闲口, USART3 归还给 JY901S IMU (@115200)
 *   指令必须严格匹配(大写)+\n 才执行, 防误触发 */
#define HC05_UART_INSTANCE              UART4
#define HC05_UART_IRQn                  UART4_IRQn
#define HC05_BAUDRATE                   115200U
#define HC05_RX_RING_SIZE               128U
#define HC05_LINE_BUFFER_SIZE           32U
#define HC05_MANUAL_LINEAR_SPEED_CM_S   MANUAL_LINEAR_SPEED_CM_S
#define HC05_MANUAL_ANGULAR_SPEED_RAD_S DEG_TO_RAD(MANUAL_ROTATE_SPEED_DEG_S)

/* 24. 通用宏 */
#define DEG_TO_RAD(value) ((value) * 3.14159265358979f / 180.0f)

#endif /* __MOTION_CONFIG_H__ */
