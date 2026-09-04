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
 *   §11-§15 IMU/传感器融合参数 (movemethod 子模块)
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

/* 2. 运动状态机定义 (18 态)
 * 新增状态一律追加在末尾，避免改动已有状态的数值编号 (遥测 state= 字段)。
 * 拆除超声后，原墙扫五态 (WALL_SCAN_FIRST/SECOND、WALL_TURN_OPPOSITE、
 * WALL_SMALL_ROTATE、WALL_MEASURE_MOVE) 与超声避障 AVOID_TURN 一并删除，
 * 换位改由开环的 SEARCH_RELOCATE 承担；摄像头不再翻转，CAMERA_ROTATE 也删除。
 * 遥测 state= 的数值编号因此整体前移，看遥测请以 state_name 为准。 */
typedef enum {
  MOTION_STATE_INIT = 0,
  MOTION_STATE_ROTATE_SEARCH,
  MOTION_STATE_PRE_CENTERING,
  MOTION_STATE_TARGET_TRACKING,
  MOTION_STATE_FINAL_APPROACH,
  MOTION_STATE_BRUSH_COLLECT,
  MOTION_STATE_SEARCH_CONTINUE,
  MOTION_STATE_HOME_PREPARE,
  MOTION_STATE_HOME_FOLLOW,
  MOTION_STATE_HOME_TURN_AROUND,
  MOTION_STATE_HOME_DONE,
  MOTION_STATE_BLACK_AREA_SEARCH,
  MOTION_STATE_BLACK_AREA_TRACK,
  MOTION_STATE_UNLOADING,
  MOTION_STATE_STANDBY,
  MOTION_STATE_BT_ROTATE,
  MOTION_STATE_WALL_BACKOFF,
  MOTION_STATE_SEARCH_RELOCATE
} MotionState_t;

/* 上电控制模式: 0=手动，1=自动；运行中可用 MANUAL/AUTO 切换。 */
#ifndef DEFAULT_CONTROL_MODE
#define DEFAULT_CONTROL_MODE                0U  /* 视觉链路联调阶段默认手动停车 */
#endif

/*
 * 前阶段联调：1=循环执行搜索→对准→收集，满 TOTAL_OBJECTS_TO_COLLECT 次后
 * 直线返航并卸货。
 * 不进入摄像头反转流程。
 */
#define OBJECT_APPROACH_TEST_MODE         1U

/* 3. 视觉输入数据结构 (OpenMV → STM32)
 *   字段单位在两端保持一致; timestamp_ms 由 STM32 接收时打戳。 */
typedef struct {
  uint8_t  detected;        /* 1=看到目标, 0=没看到 */
  uint16_t center_x_px;     /* OpenMV 原始中心坐标, QVGA范围 0~319 */
  uint16_t center_y_px;     /* OpenMV 原始中心坐标, QVGA范围 0~239 */
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

/* 5. 视觉墙体输入结构 (OpenMV 蓝墙检测, 帧格式 W,state,fill_pct)
 *   车上已无超声, 这是唯一的障碍/边界感知来源。
 *   不估距离: 墙比画面还宽时任何按宽度反推的距离都会饱和, 而蓝色像素占比
 *   一直到贴脸都是单调的。blocked 由 OpenMV 带迟滞判定后给出;
 *   fill_pct 除了标定阈值时观察, 还被搜索扫视记进方向表用来挑换位方向。
 *   注意 OpenMV 每帧无条件发 W 帧, 视野里没有蓝色时 fill_pct=0。 */
typedef struct {
  uint8_t  blocked;       /* 1=蓝墙已近到需要退避, 0=安全 */
  uint8_t  fill_pct;      /* 蓝色占 ROI 的百分比 (0~100) */
  uint8_t  valid;          /* 1=帧新鲜, 0=超时无数据 */
  uint32_t timestamp_ms;   /* STM32 接收时间戳 (ms) */
} VisionWallData_t;

/* 5.1 黑区到位输入结构 (OpenMV 黑区面积判定, 帧格式 A,state,fill_pct)
 *   只在 OpenMV 处于黑区模式 (M,1) 时逐帧发送, 物块模式下收不到,
 *   此时靠 VISION_STALE_MS 超时把 arrived/valid 清 0。
 *   取代原先的 vision->distance_cm 判据: OpenMV 报的"距离"实际是黑区近边缘
 *   到画面底边的像素间隙, 远处黑区被画面上边缘裁切时同样读出接近 0 的值,
 *   会让车一看到黑区就地掉头。面积占比没有这个歧义 —— 远则小, 近则大。 */
typedef struct {
  uint8_t  arrived;        /* 1=黑区面积已超阈值, 可以掉头卸货 */
  uint8_t  fill_pct;       /* 黑区外框占整幅画面的百分比 (0~100) */
  uint8_t  valid;          /* 1=帧新鲜, 0=超时无数据 */
  uint32_t timestamp_ms;   /* STM32 接收时间戳 (ms) */
} VisionArrivalData_t;

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
/*
 * 黑区原地扫视的时限, 同时决定扫视方向表能覆盖多少方向: 以
 * SEARCH_ROTATION_SPEED_DEG_S 转 12.5s 约 375°, 转满一圈还留一点余量, 否则
 * 表里会有一段方向从未扫到、永远选不出来。改转速时这里要跟着改。
 */
#define BLACK_SEARCH_TIMEOUT_MS          12500U
/*
 * 找物块阶段的总兜底: 距上次"滚刷收集成功"超过这么久就放弃本轮, 带着已经
 * 收到的直接返航卸货 (收几个算几个)。
 *
 * 判据是收集成功而不是看见物块: 只有 BRUSH_COLLECT 走完整套滚刷+后斗动作、
 * s_collected 真的加一时才清零。按"看见"清零挡不住两种常见卡死 —— 车盯着
 * 一个够不到的物块反复对准, 或者每次追踪都在最后一刻丢失目标; 这两种情况下
 * 视觉一直有目标, 计时被无限推迟, 车能在原地耗完整场时间。
 *
 * 也因此这个计时在收集阶段全程累加 (含 PRE_CENTERING / TARGET_TRACKING /
 * FINAL_APPROACH / BRUSH_COLLECT), 不像只在搜索态计时那样漏掉上面那类卡死。
 * 判定点在 ROTATE_SEARCH 与 SEARCH_RELOCATE: 所有路径最迟经过 BRUSH_COLLECT
 * → SEARCH_CONTINUE 就会回到搜索态, 所以不必在每个状态里都查一遍。
 *
 * 与 SEARCH_NO_TARGET_TIMEOUT_MS 的区别: 那个是单轮扫视转满一圈的判据,
 * 只管一次原地扫视; 这个跨状态累加, 状态往返和蓝墙退避都不清零。
 *
 * 注意这是"每收一个物块"的预算, 不是整轮的: 收满 TOTAL_OBJECTS_TO_COLLECT
 * 需要连续多次都在预算内完成。若实测经常在还有物块时就提前返航, 加大这个值。
 */
#define SEARCH_GIVE_UP_MS                30000U  /* 40000 */
#define TARGET_LOCK_WINDOW_MS            800U
#define PRE_CENTERING_TIMEOUT_MS         2000U
#define TRACKING_COMPLETE_MIN_FORWARD_MS 200U /* 至少前进一段时间后才允许判定追踪完成 */
#define TRACKING_COMPLETE_LOSS_MS        100U /* 目标消失确认后触发滚刷 */

/* 8. 速度与运动控制参数
 *   下列线速度已整体上调 15% (原值见括号)。角速度一律未动: 扫视角速度关系到
 *   单帧识别时间, 定角旋转角速度关系到停角精度, 两者都不能跟着线速度涨。 */
#define MAX_LINEAR_SPEED_CM_S            50.0f     /* 限幅, 非阶段速度, 未上调 */
#define MAX_ANGULAR_SPEED_RAD_S          1.0f      /* 最大角速度 (rad/s) */
#define MAX_WHEEL_ACCEL_CM_S2            120.0f
#define SEARCH_ROTATION_SPEED_DEG_S      30.0f
#define TRACKING_LINEAR_SPEED_CM_S       30.0f     /* 20.0 */
#define COLLECT_LINEAR_SPEED_CM_S        8.0f      /* 无调用方, 未上调 */
#define MANUAL_LINEAR_SPEED_CM_S         23.0f     /* 20.0 */
#define MANUAL_ROTATE_SPEED_DEG_S        40.0f
#define TIGHT_TURN_LINEAR_SPEED_CM_S      9.2f     /* 8.0 */
#define TIGHT_TURN_ANGULAR_SPEED_DEG_S    65.0f

/* 9. 视觉追踪参数 */
#define VISION_IMAGE_WIDTH_PX            320U   /* OpenMV QVGA 画面宽 */
#define VISION_IMAGE_HEIGHT_PX           240U   /* OpenMV QVGA 画面高 */
#define VISION_CENTER_X_PX               (VISION_IMAGE_WIDTH_PX / 2U)
#define VISION_CENTER_Y_PX               (VISION_IMAGE_HEIGHT_PX / 2U)
#define CAMERA_X_AXIS_REVERSED            1U  /* 摄像头倒装，左右图像坐标取反 */
#define TRACKING_DEADZONE_PX             8
#define TRACKING_CENTER_OFFSET_PX       (-30) /* 摄像头安装偏移，随QVGA分辨率减半 */
/*
 * PRE_CENTERING 原地对准的两个专用参数, 解决"只差一点却转不动"。
 *
 * 模糊控制的输出在死区边缘趋近 0: 误差 9px 时算出来只有 0.09rad/s, 换算成
 * 轮速 (w × 22.4/2) 仅 1cm/s, 四轮原地转克服不了静摩擦, 于是车持续输出一个
 * 转不动的指令, 直到 PRE_CENTERING_TIMEOUT_MS 超时回搜索。
 *
 * MIN_TURN_DEG_S: 非零修正一律抬到能起转的量级 (取与 SEARCH_ROTATION_SPEED_
 *   DEG_S 相同的 30°/s, 那个转速已验证能从静止起转), 只改幅值不改方向。
 * EXIT_PX: 退出带放宽到远大于模糊死区(8px)。原地转是粗对准, 精对准由
 *   TARGET_TRACKING 边走边修完成; 带子留宽还能吸收 50ms 周期下抬速后的
 *   单周期转过量 (30°/s × 50ms = 1.5°, 折合约 5px) 加机械惯性, 避免在边界摆。
 */
#define PRE_CENTERING_MIN_TURN_DEG_S     30.0f
#define PRE_CENTERING_EXIT_PX            20
#define TARGET_DISTANCE_MAX_CM           50U
#define COLLECT_DISTANCE_CM              10U
#define FINAL_APPROACH_DURATION_MS       700U /* 追踪结束后保持追踪速度前进1秒 */
/* 黑区到位判据: 由 OpenMV 的 A 帧直接给出 (见 §5.1), STM32 不再自己算距离。
 * 阈值 BLACK_ARRIVAL_FILL_PCT 在 detect_distance.py 里, 用遥测 arv_pct 标定。
 * 这里只做一层去抖: A 帧没有迟滞, 阴影和黑区连成一片时单帧占比会突跳,
 * 所以要求连续这么久都报到位才掉头。约 4 帧, 不会明显滞后。 */
#define BLACK_ARRIVE_CONFIRM_MS          200U
#define TRACKING_SLOW_RATIO              1.0f
#define TRACKING_CORRECTION_GAIN         1.0f  /* 行进修正不过度放大，尽量保持两侧前进 */
#define TRACKING_MIN_LINEAR_RATIO        0.60f /* 大偏差时仍保留60%前进速度 */
#define BLACK_AREA_OBJECT_TYPE           3U
#define TOTAL_OBJECTS_TO_COLLECT         5U

/* 返航: 编码器距离 + 融合航向算出的二维位置, 直线驶向原点。
 *
 * 流程: HOME_PREPARE 记下此刻到原点的距离 d, 锁定朝向原点的方位角
 *       → HOME_FOLLOW 朝该方位角直行, 只走 d 的 HOME_PARTIAL_RETURN_RATIO
 *         (航向偏差大就先原地转, 见 HOME_ROTATE_IN_PLACE_DEG)
 *       → 走完 / 已进 HOME_ARRIVAL_RADIUS_CM / 兜底超时 → BLACK_AREA_SEARCH
 *       全程盯着黑区, 确认看见就直接交给 BLACK_AREA_TRACK。
 *
 * 为什么只走一部分而不是走到原点: 位置是编码器速度的二次积分, 打滑和标定
 * 误差随路程累积, 走完全程终点会明显偏离原点。所以直线段只负责"把车带到
 * 卸货区附近", 剩下的距离交给 OpenMV 认黑区, 用视觉消掉里程计的累计误差。
 *
 * 遇墙: 直线段撞墙时倒一小段并把锁定的方位角右旋 HOME_BACKOFF_TURN_DEG (8°),
 * 沿新方位角继续往前拱; 墙还在就再来一档, 8° 一档地蹭出去, 期间一直盯着黑区,
 * 看见就转 BLACK_AREA_TRACK 走后续流程 (见 §12.2 的 HOME_BACKOFF_*)。
 * 累计 HOME_WALL_GIVEUP_MS 还绕不出去就放弃剩下的返航距离, 就地转
 * BLACK_AREA_SEARCH 原地扫黑区。 */
#define HOME_ORIGIN_WAIT_MS              1500U
/* 直线段只走全程的这个比例, 余下距离作为退出阈值。
 * 调大 = 更依赖里程计精度, 调小 = 更早交给视觉但可能离黑区太远看不见。 */
#define HOME_PARTIAL_RETURN_RATIO        0.75f  /* 0.5 / 0.2 */
/* 直线段兜底超时: 里程计异常 (比如打滑导致距离一直不减) 时也能往下走。 */
#define HOME_FOLLOW_TIMEOUT_MS           20000U
/* 到原点的判定半径。因为有 PARTIAL_RETURN_RATIO, 通常先按比例退出, 这个值
 * 只是"出发时就已经离原点很近"的兜底判据。 */
#define HOME_ARRIVAL_RADIUS_CM           12.0f
/*
 * 返航途中的视觉交接: 返航全程 OpenMV 都处于黑区模式, 一旦连续
 * HOME_VISION_CONFIRM_MS 都看见黑区就直接进 BLACK_AREA_TRACK。
 * CONFIRM_MS 用来滤掉单帧闪跳; 被蓝墙退避打断时计时清零 (累加逻辑在 switch
 * 之前, 只在 s_state 为 HOME_FOLLOW 时累加)。
 *
 * 这里没有距离门限: 虽然返航有位置估计, 但里程计误差正是要靠视觉消掉的东西,
 * 用它去给视觉设门限就本末倒置了。代价是误检风险全靠 OpenMV 侧
 * 把关: BLACK_PIXELS_THRESHOLD 只有 300px (一帧的 0.4%), 阴影、黑胶带、深色
 * 底盘都够触发, 而误判代价不轻 (BLACK_AREA_TRACK 丢目标后会转去
 * BLACK_AREA_SEARCH, 返航航线就此中断)。若实测有远处误触发, 优先调高
 * OpenMV 的 BLACK_PIXELS_THRESHOLD, 而不是在这里加门限。
 */
#define HOME_VISION_CONFIRM_MS           300U
#define HOME_RETURN_SPEED_CM_S           23.0f  /* 20.0 */
#define HOME_RETURN_HEADING_KP           1.2f
#define HOME_RETURN_MAX_ANGULAR_RAD_S    0.8f
/*
 * 直线段的"先转再走"判据: 与锁定方位角的偏差超过这个角度就原地转, 不带前进
 * 分量; 进到一半 (本值的 0.5 倍) 才切回边走边修, 这个迟滞是必须的 ——
 * 单阈值会让车在阈值附近反复"停下原地转 / 起步前进", 走不动路。
 * 别调太大: 25° 已经够, 再大就变成动不动先原地转一圈才肯走。
 */
#define HOME_ROTATE_IN_PLACE_DEG         25.0f
/* 里程计航向的固定偏差补偿: 作用于 x/y 积分, 因而也间接影响由 (x,y) 算出的
 * 返航方位角。hdg_deg 遥测不经过它 (直接取融合航向相对归零点的转角), 所以调
 * 它不会改变 hdg_deg 读数, 但会让 home_x/home_y 与 bear_deg 一起转过去。
 * 只有在确认车实际走的方向与航向读数存在固定夹角时才动它。 */
#define HOME_HEADING_OFFSET_DEG          0.0f
/* 卸货前的原地掉头, 融合航向闭环 (不是按时间开环)。
 * 转不够 180° 车尾就没对准黑区, 物块会卸到区外, 所以停角精度要紧:
 * 由 FUSION_YAW_SCALE 与下面的 TOL 共同决定, 用蓝牙 ROTATE180 在地面验证。 */
#define HOME_TURN_AROUND_TARGET_DEG      180.0f
#define HOME_TURN_AROUND_SPEED_DEG_S     45.0f
#define HOME_TURN_AROUND_TOL_DEG         5.0f
#define HOME_TURN_AROUND_TIMEOUT_MS      10000U
/* 掉头后再倒一段: 车尾(后斗/门)此时朝着黑区, 多退一点能把卸货点从黑区
 * 边缘挪到中间, 免得物块滚出区外。车尾没有传感器, 只能用固定时长限量。
 * 3.0s × 17.25cm/s ≈ 52cm, 全程开环倒车, 车尾顶到墙也不会被察觉。 */
#define HOME_BACKUP_DURATION_MS          3000U
#define HOME_BACKUP_SPEED_CM_S           17.25f /* 15.0 */

/* 蓝牙 ROTATE180 / ROTATE360: 融合航向闭环的顺时针原地定角旋转。
 * 与 HOME_TURN_AROUND 同一套判据同一个航向源, 所以可以拿它当返航掉头的
 * 地面验证手段: ROTATE180 转得准, AUTO 里的卸货掉头就准。 */
#define BT_ROTATE180_TARGET_DEG          180.0f
#define BT_ROTATE360_TARGET_DEG          360.0f
#define BT_ROTATE_SPEED_DEG_S            45.0f
#define BT_ROTATE_SLOW_SPEED_DEG_S       18.0f  /* 接近目标时减速，提高停角精度 */
#define BT_ROTATE_SLOW_WINDOW_DEG        30.0f  /* 剩余角度小于此值开始减速 */
#define BT_ROTATE_TOL_DEG                3.0f
#define BT_ROTATE_TIMEOUT_MS             25000U

/* 10. 模糊控制参数 (block_alignment 子模块, 物块对准)-
 *
 *   输入: x_offset_px ([-160, +159], QVGA画面，负=左, 正=右)
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
#define FUZZY_OUT_NB_RAD_S               (-1.0f)   /* 大幅右转 (CW) */
#define FUZZY_OUT_NM_RAD_S               (-0.7f)
#define FUZZY_OUT_NS_RAD_S               (-0.4f)
#define FUZZY_OUT_ZE_RAD_S               (0.0f)
#define FUZZY_OUT_PS_RAD_S               (0.4f)
#define FUZZY_OUT_PM_RAD_S               (0.7f)
#define FUZZY_OUT_PB_RAD_S               (1.0f)    /* 大幅左转 (CCW) */

/* 11. 车轮与编码器参数 */
#define WHEEL_DIAMETER_CM                6.5f
#define WHEEL_CIRCUMFERENCE_CM           (3.14159f * WHEEL_DIAMETER_CM)
#define WHEEL_TRACK_WIDTH_CM              22.4f      /* 左右轮中心距 (cm) */
#define WHEEL_BASE_CM                     9.7f      /* 前后轮中心距 (cm) */
#define ENCODER_PULSES_PER_REV            480        /* 每转脉冲数 */
#define ODOMETRY_STATIC_DEADZONE_CM_S     0.5f      /* 编码器静止零漂死区 (cm/s) */

/* 12. 原地扫视搜索
 *   线速度与角速度之比决定回转半径 (4.6 / (35°/s) ≈ 7.5cm), 保持小半径以免
 *   搜索时跑离原地。角速度不宜太高: 目标停留在 80°视野里的时间只有
 *   视野角/角速度, 35°/s 约 2.3 秒; 再快 OpenMV 就容易因运动模糊和
 *   帧间跨度过大而漏检——这正是"一直转圈却不识别物块"的常见原因。 */
#define SEARCH_SWEEP_LINEAR_CM_S         4.6f   /* 4.0 */
#define SEARCH_SWEEP_ANGULAR_DEG_S       35.0f
/* 转满一圈的判据 (留余量, 不用整 360 以免边界差一点点判不到) */
#define SEARCH_FULL_CIRCLE_DEG           350.0f

/* 12.1 搜索换位 (转满一圈没发现目标 → 走一段换个视角重新搜)
 *   方向来自下面的扫视方向表 (不是盲走); 距离是开环的, 车上没有测距传感器,
 *   途中撞墙不回本状态重走: 退避完直接回原地扫视重新挑方向 (见 12.2),
 *   所以这段距离只在"一路无墙"时才会真的走满。 */
#define SEARCH_RELOCATE_DIST_CM          120.0f /* 60.0 */
#define SEARCH_RELOCATE_SPEED_CM_S       23.0f  /* 20.0 */
/*
 * 超时只是里程计失效时的兜底, 必须留足于"正常走完全程"所需的时间, 否则
 * 距离判据永远轮不到、每次换位都被超时截断: 120cm / 23cm/s ≈ 5.2s, 加上
 * 起步加速和对准阶段, 给到 12s。
 */
#define SEARCH_RELOCATE_TIMEOUT_MS       12000U
/*
 * 换位方向由"扫视方向表"决定, 不再盲走。
 *
 * 记录: ROTATE_SEARCH 与 BLACK_AREA_SEARCH 原地扫视时, 把一圈按
 *   SCAN_SECTOR_DEG 分格, 每格存该朝向看到的蓝墙 fill_pct (同格取最小,
 *   压掉单帧误检)。用占比而不是估距, 因为墙比画面还宽时任何按宽度反推的
 *   距离都会饱和, 而占比一直到贴脸都单调; 且 fill_pct 每帧都有, 0 代表
 *   视野里完全没有蓝色。没扫到的格留 SCAN_SECTOR_EMPTY, 选向时跳过
 *   (扫视不足一圈或被退避打断都会留空格)。
 *
 * 选向: 转完一圈后先求全表最小占比, 再挑占比最接近 "最小值 + 偏移" 的格。
 *   找物块换位偏移取 0, 即最空旷方向, 离墙最远最不容易再撞回同一面墙。
 *   找黑区换位偏移取 SCAN_BLACK_OFFSET_PCT: 卸货区通常贴着墙, 完全空旷的
 *   方向指向场地中央, 朝那边走反而更找不到, 得朝"稍微有点墙"的方向去。
 *
 * 表里没有有效数据 (收不到 W 帧) 时退化为沿当前朝向直行, 即改动前的行为。
 * 对准是原地转, 最坏要转 180°, 所以超时给到 6s。
 */
#define SCAN_SECTOR_DEG                  10.0f
#define SCAN_SECTOR_COUNT                36U
#define SCAN_SECTOR_EMPTY                0xFFU
#define SCAN_BLACK_OFFSET_PCT            5U
#define RELOCATE_AIM_SPEED_DEG_S         45.0f
#define RELOCATE_AIM_TOL_DEG             8.0f
#define RELOCATE_AIM_TIMEOUT_MS          6000U

/* 12.2 蓝墙退避 (OpenMV 视觉墙, 车上唯一的障碍感知)
 *   动作是"后退 + 右转", 退到 OpenMV 不再报有墙为止。
 *   在所有"朝未知方向前进"的状态生效: ROTATE_SEARCH / SEARCH_RELOCATE /
 *   HOME_FOLLOW / BLACK_AREA_SEARCH。
 *   退避结束后的去向分三种:
 *     SEARCH_RELOCATE  → 不回去重走剩下的距离, 那个方向既然被墙拦住, 直接回
 *                        原地扫视重新挑方向;
 *     HOME_FOLLOW      → 倒一小段并把锁定的返航方位角右旋 8°, 回 HOME_FOLLOW
 *                        沿新方位角继续往前拱, 墙还在就再来一档
 *                        (见下面的 HOME_BACKOFF_*);
 *     其余             → 回被打断的那个状态。
 *   物块与黑区的最后接近段必须禁用: 前者要撞上去推进滚刷
 *   (COLLECT_DISTANCE_CM 只有 10cm), 后者要压到黑区上方才卸货。
 *   CLEAR_MS 要求连续这么久无墙才退出, 否则蓝墙在视野边缘闪烁时会
 *   在退避与前进之间反复切换。 */
#define WALL_BACKOFF_SPEED_CM_S          17.25f /* 15.0 */
/* 转速大小; 方向在调用处取负 = 右转 (顺时针), 与搜索扫视同向。 */
#define WALL_BACKOFF_TURN_DEG_S          40.0f
#define WALL_BACKOFF_CLEAR_MS            400U
#define WALL_BACKOFF_TIMEOUT_MS          6000U
/*
 * 返航直线段 (HOME_FOLLOW) 专用的定量退避, 与上面的通用退避两点不同:
 *   1) 退出条件是"倒完 HOME_BACKOFF_DURATION_MS", 不等墙从视野里消失 ——
 *      返航要的是绕过去继续走, 不是原地磨到墙不见;
 *   2) 退完把锁定的返航方位角一起右旋 HOME_BACKOFF_TURN_DEG, 车才真的走上新
 *      航线。只转车不转方位角的话, 回到 HOME_FOLLOW 第一个周期就按原方位角
 *      左转拧回来, 又撞上同一面墙。
 * 一次只偏 8°, 偏完继续往前拱; 墙还在就再触发一次, 8° 一档地蹭出去。整个过程
 * 里 OpenMV 一直是黑区模式, 看见黑区随时转 BLACK_AREA_TRACK 走后续流程。
 * 17.25cm/s × 0.8s ≈ 后退 14cm。
 * 为什么偏这么小: 大角度一档就把车头甩离原航线, 走完 3/4 的落点偏差大; 8° 是
 * "尽量不偏离回家方向"和"能绕开墙"之间的折中, 代价是绕一面正面墙要十来档。
 * 累计兜底改用 HOME_WALL_GIVEUP_MS, 不是 WALL_STRUGGLE_TIMEOUT_MS, 原因见下。
 */
#define HOME_BACKOFF_DURATION_MS         800U
#define HOME_BACKOFF_TURN_DEG            8.0f   /* 30 / 45 / 10 */
/* 摊到整段退避上的转速; 改上面两个值时自动跟随。调用处取负 = 右转 (顺时针)。 */
#define HOME_BACKOFF_TURN_DEG_S \
  (HOME_BACKOFF_TURN_DEG / (HOME_BACKOFF_DURATION_MS / 1000.0f))
/*
 * 返航段专用的贴墙放弃预算, 顶替 WALL_STRUGGLE_TIMEOUT_MS。
 *
 * 为什么不能沿用那 10s: 那个数是配 30°/档定的, 三四档就绕开了。现在一档只偏
 * 8°, 正面撞墙要偏出去得十来档; 而计时把"退避 0.8s + 退完往前拱的那一下"整个
 * 算进去, 一档实际吃掉 1.5~2s, 10s 只够攒 40° 左右 —— 车还顶在墙上就被判成
 * 卡死, 8° 一档的绕行根本走不完。
 *
 * 这个兜底不能省: 一直 8° 转下去总会转到背离家的方向, 那时"走完 3/4"的正常
 * 出口永远等不到 (离原点的距离不减反增), 没有它车会顺着新方位角一路开走。
 * 30s 按上面的节奏够偏 120° 以上, 正面墙和场地角都够用。到点就放弃剩下的
 * 返航距离, 就地转 BLACK_AREA_SEARCH 原地扫黑区。
 */
#define HOME_WALL_GIVEUP_MS              30000U
/*
 * 贴墙纠缠兜底: 从第一次看见墙起累加, 跨越"退避 <-> 原状态"的往返不清零。
 * 各状态自己的超时用 s_state_ms, 而退避进出各调一次 Enter() 都会把它清零,
 * 所以车沿着墙走、每隔十几秒触发一次退避时, 那些超时永远攒不满。到点后由
 * 宿主状态执行自己的兜底跳转 (换位 / 转黑区搜索), 保证状态机往前走。
 * 只管搜索类状态 (ROTATE_SEARCH / SEARCH_RELOCATE / BLACK_AREA_SEARCH);
 * 返航段用的是 HOME_WALL_GIVEUP_MS, 那边一档只偏 8°, 需要宽裕得多的预算。
 * 计时器本身是同一个 (s_wall_struggle_ms), 只是判据阈值不同。
 */
#define WALL_STRUGGLE_TIMEOUT_MS         10000U

/* 13. 滚刷控制参数 */
#define BRUSH_ROTATE_DURATION_MS         (BRUSH_OUTBOUND_MS + BRUSH_RETURN_MS)
#define BRUSH_TIMEOUT_MS                 3500U     /* 滚刷动作超时保护 */
#define COLLECT_BUCKET_DELAY_MS          150U      /* 滚刷完成后再启动后斗 */

/* 14. 后斗卸载参数
 *   序列: 开门 → 升降 → 前进 0.5s → 升降 → 关门 → 左前方弧线开出黑区。
 *   两轮升降之间挪一小段, 让第二轮卸在稍微不同的位置, 免得所有物块堆在
 *   同一点上互相挡住出口。
 *   门只开关一次, 全程保持打开: 物块常卡在斗底或门边, 只有门开着时反复升降
 *   才能把它抖出去; 每轮都开关门既慢, 抖动也只在开门那一刻才有效。
 *   升降不留停留时间, 只等舵机走完行程就反向 —— 抖动靠的是升降本身的冲击,
 *   停在顶端并不会让卡住的物块自己滑出去。
 *   合计约 0.5 + 2 + 0.5 + 2 + 0.5 + 2.5 = 8.0s。 */
#define UNLOAD_DOOR_OPEN_MS              500U   /* 等门转到位 */
/* 只等舵机走完行程, 不额外停留; 两个值分别等于 BUCKET_OUTBOUND/RETURN_MS。
 * 舵机是直接给 PWM 目标角、按自身速度走, 所以缩短等待不改变升降速度,
 * 但若实测 1s 走不完 0°→120°, 这里必须跟着加大, 否则斗还没到位就反向。 */
#define BUCKET_LIFT_DURATION_MS          1000U
#define BUCKET_LOWER_DURATION_MS         1000U
#define BUCKET_DOOR_CLOSE_MS             500U   /* 等门关到位 */
#define UNLOAD_REPEAT_COUNT              2U
/*
 * 两轮升降之间的挪位, 同时也是下一轮 (0,0) 的取点: 挪完的落脚处就是第二次
 * 卸货的位置, 离真实卸货点最近, 作为下一轮返航目标最合适。
 * 0.5s × 17.25 ≈ 9cm, 只够错开一个车身宽度以内, 不会跑出卸货区。
 */
#define UNLOAD_MID_FORWARD_MS            500U
/*
 * 卸完货开出黑区再进下一轮: 车尾还压在卸货区上, 直接原地起转会把刚倒出来
 * 的物块扫散。走弧线朝左前方离开, 结束时车头已偏左 UNLOAD_EXIT_TURN_DEG,
 * 下一轮搜索的起始朝向与上一轮不同, 不会反复扫同一片区域。
 * 速度取与 HOME_DONE 倒车段相同; 2.5s 约 43cm 弧长, 回转半径约 82cm。
 * 同样是开环: 本状态不做蓝墙退避, 前方有墙不会被察觉。
 */
#define UNLOAD_EXIT_FORWARD_MS           2500U
#define UNLOAD_EXIT_SPEED_CM_S           HOME_BACKUP_SPEED_CM_S
#define UNLOAD_EXIT_TURN_DEG             30.0f  /* 整段累计左偏角 */
/* 摊到整段上的转速; 改上面两个值时自动跟随。调用处取正 = 左转 (逆时针)。 */
#define UNLOAD_EXIT_TURN_DEG_S \
  (UNLOAD_EXIT_TURN_DEG * 1000.0f / (float)UNLOAD_EXIT_FORWARD_MS)
/* 仅剩孤立模块 bucket_unload.c 在用 (状态机未调用它), 保留以免链接失败。 */
#define BUCKET_UNLOAD_DURATION_MS        4000U

/* 15. 摄像头舵机 180°视野切换已删除: 摄像头固定朝车头, 状态机不再翻转视野。
 *   舵机本身仍在 §21 配置, 可由蓝牙手动触发一次往复。 */

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
/* 航向标度校准: 融合航向相对"归零点"的增量整体乘以该系数。
 *
 *   由来: 下 ROTATE180 实际只转 150°, ROTATE360 实际只转 315° —— 估计出的
 *         转角比真实转角偏大, 车提前认为转够了就停。根因是原地旋转时 4 轮
 *         打滑, 编码器推算的转角虚高, 把融合航向拽了上去。
 *         标定式: 新值 = 当前值 × (实测转角 / 指令角度)。
 *
 *   >>> 这是全车唯一影响转角精度的系数 <<<
 *   曾经把定角旋转和返航链路挪去一条纯 IMU 航向 (见 IMU_YAW_SCALE), 想绕开
 *   编码器打滑; 实测更差, 已全部撤回。现在吃这个系数的有:
 *     · 蓝牙 ROTATE180 / ROTATE360
 *     · 返航链路: HOME_FOLLOW 朝方位角保向、HOME_TURN_AROUND 掉头
 *     · SEARCH_RELOCATE 的转向瞄准、扫视方向表
 *     · home_x / home_y 位置积分与 hdg_deg / bear_deg 遥测
 *
 *   标定注意: 0.85 是在 BT_ROTATE_TOL_DEG=3 时定的, 与当前值一致, 所以那次
 *   标定仍然有效。若改了容差再重标, 容差必须一起算进去:
 *     新值 = 当前值 × (实测/指令) × (指令-旧容差)/(指令-新容差)
 *
 *   注意: 只缩放"增量", 不动归零基准, 所以 SET_HOME 后的绝对参考不变。
 *   返航直线段追的方位角本身也在这个缩放过的航向系里, 与航向同源, 所以本
 *   系数对"能不能对准家"的影响会部分抵消; 它主要还是决定定角旋转的精度。 */
#define FUSION_YAW_SCALE                 0.85f
/* 纯 IMU 航向 (SensorFusion_GetImuHeading) 的标度校准。
 *
 *   >>> 现在不参与任何控制, 只作用于遥测的 imu_yaw 字段 <<<
 *   这一路是为了绕开原地旋转时的四轮打滑而单独拉出来的, 定角旋转和返航链路
 *   都曾走它, 实测不如融合航向, 已全部改回 FUSION_YAW_SCALE 那一路。
 *
 *   保持 1.0 (不校准) 是刻意的: 遥测里要看的就是模组的原始表现, 乘了系数就
 *   没法拿它和 yaw_enc / hdg_deg 对照定位问题了。
 *
 *   实测记录 (模组是 6 轴, yaw 为纯陀螺积分, 无磁力计):
 *     0.833 + tol 3 → 180 指令实测约 175°   → k = 175 / (177/0.833) = 0.824
 *     0.819 + tol 1 → 180 指令实测约 185°   → k = 185 / (179/0.819) = 0.846
 *     0.826 + tol 1 → 180 指令基本准        → k = 180 / (179/0.826) = 0.831
 *   k = 实测/模组原始估计, 三次落在 ±1.3% 内 (±2.4°, 在肉眼量角误差内), 即
 *   一个干净的比例误差: 模组 yaw 把转角多报约 20%。
 *   注: 早期那组"180 实测 150 / 360 实测 315"是融合航向通路的数据 (纯 IMU
 *   通路当时还不存在), 不能和上面三行混在一起拟合, 否则会拟出一个假的固定项。
 *
 *   为什么单独标定过还是不好用: 20% 这个量级远超陀螺器件指标, 基本只能是
 *   振动整流 (模组硬连底盘, 四轮搓地的振动被整流成直流偏置), 因此该比例随
 *   转速和负载变化 —— 旁证是"外部施加阻力时偏差变大"。而 0.826 是在
 *   BT_ROTATE_SPEED_DEG_S=45 标的, 全场累计转角最多的 ROTATE_SEARCH 跑的是
 *   SEARCH_ROTATION_SPEED_DEG_S=30, 用错档的系数, 加上搜索恒为顺时针, 同号
 *   残差在一轮里能叠到上百度, 返航对准初始方向就废了。
 *   要救这一路得先做机械隔振 (泡棉/橡胶浮装模组), 把 k 拉回 0.98 附近再谈
 *   标定; 光调系数追不上。 */
#define IMU_YAW_SCALE                    1.0f
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
/* 三个SG90：保持已经实测可用的参数不变。 */
#define SG90_MIN_PULSE_US                500U
#define SG90_MAX_PULSE_US                2500U
#define SG90_ANGLE_LIMIT_DEG             180U
/* 滚刷MG995：保留机械限位余量，避免上电在端点持续堵转。 */
#define MG995_MIN_PULSE_US               600U
#define MG995_MAX_PULSE_US               2400U
#define MG995_ANGLE_LIMIT_DEG             180U

/* 门舵机: 175°为上电/关门静止位 (留 5°余量, 避免长期顶死机械限位),
 * 卸货时开到 90°。 */
#define DOOR_START_DEG                   175U
#define DOOR_END_DEG                     90U
#define DOOR_OUTBOUND_MS                 1000U
#define DOOR_RETURN_MS                   1000U

#define CAMERA_START_DEG                 0U
#define CAMERA_END_DEG                   180U
#define CAMERA_OUTBOUND_MS               1000U
#define CAMERA_RETURN_MS                 1000U

#define BUCKET_START_DEG                 0U
#define BUCKET_END_DEG                   120U
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
#define WHEEL_PID_FEEDFORWARD_DUTY       2000.0f
#define WHEEL_SPEED_FILTER_ALPHA         0.25f
#define WHEEL_D_FILTER_ALPHA             0.20f
#define WHEEL_PID_I_MIN                  (-1000.0f)
#define WHEEL_PID_I_MAX                  1800.0f
#define WHEEL_PID_OUTPUT_MAX             3400.0f
#define WHEEL_START_BOOST_MS             120U
#define WHEEL_START_BOOST_DUTY_PCT       45         /* 低强度短助推，随后进入差速PID */
/* 保护阈值 */
#define WHEEL_REVERSE_LIMIT_MM_S         50.0f
#define WHEEL_REVERSE_LIMIT_TICKS        50U
#define WHEEL_NO_FB_GRACE_TICKS          300U
#define WHEEL_NO_FB_DUTY                 2700
#define WHEEL_NO_FB_SPEED_MM_S           10.0f
#define WHEEL_NO_FB_LIMIT_TICKS         500U

/* 23. OpenMV 协议常量 (openmv_uart 用)
 *   OpenMV → STM32 帧: "V,color,cx,cy,distance_cm\n"
 *   color: 0=未检测到, 1=红色物块, 2=黄色物块, 3=黑区。
 *   STM32接收后根据QVGA中心(160,120)生成x/y_offset供运动策略使用；
 *   x_offset会根据CAMERA_X_AXIS_REVERSED决定是否反向。
 *   STM32 → OpenMV 命令: "M,0\n"=物块模式, "M,1\n"=黑色卸货区模式
 *   OpenMV 端 (Python, 运行在摄像头上, STM32 无法动态加载) 参数参考:
 *     FOCAL_LENGTH_PX = 185.0, IMAGE_WIDTH=320, IMAGE_HEIGHT=240
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
#define OPENMV_MODE_REFRESH_MS           1000U      /* 模式指令周期性重发 */
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
