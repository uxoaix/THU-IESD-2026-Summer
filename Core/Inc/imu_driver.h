/**
 * @file    imu_driver.h
 * @brief   维特(Wit) IMU 低层硬件驱动接口 (UART 通信层)
 *
 *     ① 初始化串口(USART3@115200) + 启动中断接收
 *     ② 中断里逐字节喂入协议解析器, 组帧 + 和校验 + 取出 int16 原始值
 *     ③ 提供读取最新原始值、查询数据就绪、查询通信是否正常的接口
 *     ④ 提供发送配置命令(校准/保存/波特率)的接口 —— 等价于"写寄存器"
 *
 *   集成说明 (2026-08-27):
 *     本驱动原在 my_code/movemethod/imu/ 下, 使用 USART2。为避免与 OpenMV
 *     (USART2@115200) 冲突, IMU 已改到 USART3@115200。原驱动自带弱函数
 *     HAL_UART_RxCpltCallback/HAL_UART_ErrorCallback 与主工程 stm32f1xx_it.c
 *     里的同名回调冲突, 故本驱动不再定义这两个弱函数, 改为暴露
 *     IMU_OnUartRxComplete / IMU_OnUartError, 由 stm32f1xx_it.c 的统一回调
 *     按 Instance 分支调度。
 */
#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "stm32f1xx_hal.h"
#include "motion_config.h"

/**
 * @brief  IMU 原始读数 (模组直接输出的 int16 值, 未换算物理量)
 */
typedef struct {
    int16_t ax, ay, az;        /* 加速度 (原始 int16) */
    int16_t wx, wy, wz;        /* 角速度 (原始 int16) */
    int16_t roll, pitch, yaw;  /* 角度 (原始 int16) */
    int16_t temp;              /* 温度 (原始 int16) */
} IMU_RawData_t;

/**
 * @brief  初始化 IMU 硬件 (配置 USART3@115200 + 启动中断接收)
 *
 *   ① 把 USART3 配置为 115200 8N1
 *   ② 使能 USART3 中断
 *   ③ 复位协议解析状态与诊断计数
 *   ④ 启动单字节中断接收 (之后每来一字节自动进中断解析, 不阻塞主循环)
 *
 * @return 0=成功, -1=UART 初始化失败, -2=启动接收失败
 *
 */
int IMU_Init(void);

/**
 * @brief  UART 接收完成回调入口 (供 stm32f1xx_it.c 的 HAL_UART_RxCpltCallback 调度)
 *
 *   本函数做三件事:
 *     ① 把刚收到的字节喂入协议解析器 IMU_FeedByte
 *     ② 累计接收字节计数
 *     ③ 重新启动下一次单字节中断接收 (持续接收, 不阻塞)
 *
 *   调用方应在 HAL_UART_RxCpltCallback 里判断 huart->Instance == IMU_UART_INSTANCE
 *   后再调用本函数, 以免与其他串口模块相互干扰。
 *
 * @note   中断上下文调用。
 */
void IMU_OnUartRxComplete(void);

/**
 * @brief  UART 错误回调入口 (供 stm32f1xx_it.c 的 HAL_UART_ErrorCallback 调度)
 *
 *   发生溢出(Overrun)等错误时, HAL 会停止接收。本函数在错误后重新启动接收,
 *   保证数据流不会断掉。
 *
 *   调用方应在 HAL_UART_ErrorCallback 里判断 huart->Instance == IMU_UART_INSTANCE
 *   后再调用本函数。
 *
 * @note   溢出错误在长时间运行或高负载时偶发, 重启接收即可自愈。中断上下文调用。
 */
void IMU_OnUartError(void);

/**
 * @brief  查询是否有新的完整周期数据就绪
 *
 *   维特模组每周期依次发 0x51/0x52/0x53 三帧。收到 0x53(角度帧)时,
 *   一个周期结束, 本函数返回 1。调用 IMU_GetRawData 后标志清零。
 *
 * @return 1=有新数据, 0=暂无
 *
 * @note   非阻塞, 适合在主循环里轮询。
 */
uint8_t IMU_IsDataReady(void);

/**
 * @brief  读取最新一周期原始读数
 *
 *   把内部缓存的最新原始值拷贝到 out, 并清除"数据就绪"标志。
 *   读取过程会短暂关 UART 中断, 防止读到"半新半旧"的数据。
 *
 * @param  out  [输出] 原始读数 (调用方提供结构体)
 *
 * @note   即使没有新数据, 也会返回上一次的值 (调用方应先用 IMU_IsDataReady 判断)。
 */
void IMU_GetRawData(IMU_RawData_t *out);

/**
 * @brief  查询 IMU 通信是否正常 (是否近期收到过有效数据)
 *
 *   若距上次收到完整周期 < IMU_DATA_READY_TIMEOUT_MS, 返回 1; 否则 0。
 *   返回 0 通常意味着: 接线松了 / 模组没上电 / 波特率不对。
 *
 * @return 1=通信正常, 0=通信异常(超时)
 */
uint8_t IMU_IsAlive(void);

/**
 * @brief  查询驱动是否已初始化成功
 * @return 1=已初始化, 0=未初始化 (供自检用, 类比 hcsr04 检查 DWT)
 */
uint8_t IMU_IsInitialized(void);

/**
 * @brief  发送一条维特配置命令 (底层"写寄存器"原语)
 *
 *   发送 5 字节配置帧: [0xFF][0xAA][reg][dataL][dataH]
 *   模组收到后会执行相应配置(校准/保存/改波特率等)。
 *
 * @param  reg    寄存器地址 (见 WIT_REG_* 宏)
 * @param  dataL  数据低字节
 * @param  dataH  数据高字节
 *
 * @note   阻塞发送, 期间主循环暂停约 5ms。命令较罕见, 平时不用。
 *         具体 reg/data 含义请对照模组手册, 本驱动只负责"把命令发出去"。
 */
void IMU_SendConfig(uint8_t reg, uint8_t dataL, uint8_t dataH);

/** @name 配置快捷函数 (基于 IMU_SendConfig, 标准维特命令) */
/**@{*/
void IMU_Config_Save(void);                 /* 保存当前参数到模组 Flash */
void IMU_Config_AccelCalib(void);           /* 启动加速度校准 (需保持水平静止) */
void IMU_Config_GyroCalib(void);            /* 启动陀螺仪校准 (需保持静止) */
void IMU_Config_Baudrate(uint8_t code);      /* 修改模组波特率 (code 见手册) */
/**@}*/

/** @name 诊断计数 (供自检/调试) */
/**@{*/
uint32_t IMU_GetRxByteCount(void);           /* 累计接收字节数 */
uint32_t IMU_GetFrameCount(void);           /* 累计校验通过帧数 */
uint32_t IMU_GetCrcErrorCount(void);        /* 累计校验失败次数 */
/**@}*/

#endif /* IMU_DRIVER_H */
