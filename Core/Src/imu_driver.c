/**
 * @file    imu_driver.c
 * @brief   维特(Wit) IMU 低层硬件驱动实现
 *
 *   IMU_Init:     配 USART3@115200 → 开中断 → 启动单字节中断接收
 *   串口每来 1 字节 → stm32f1xx_it.c 的 HAL_UART_RxCpltCallback
 *     → 按 Instance==USART3 分支 → IMU_OnUartRxComplete → IMU_FeedByte:
 *     ① 等帧头 0x55
 *     ② 收满 11 字节
 *     ③ 和校验: 前 10 字节之和的低 8 位 == 第 11 字节
 *     ④ 按帧类型(0x51/0x52/0x53)取出 int16 原始值
 *     ⑤ 0x53(角度帧) → 置"周期就绪"标志
 *   IMU_GetRawData: 主循环调用, 原子拷贝最新原始值
 *
 *   字节:  [0]   [1]    [2..9]      [10]
 *         0x55  类型  数据(8字节)  校验和
 *   类型:  0x51=加速度(ax,ay,az,temp)
 *         0x52=角速度(wx,wy,wz,temp)
 *         0x53=角度(roll,pitch,yaw, 版本号2字节)
 *   数据按小端序(低字节在前)存 int16。
 *   校验和 = (前10字节之和) & 0xFF
 *
 *   集成说明 (2026-08-27):
 *     原驱动自带弱函数 HAL_UART_RxCpltCallback/HAL_UART_ErrorCallback, 与主工程
 *     stm32f1xx_it.c 里的同名回调冲突。本文件不再定义这两个弱函数, 改为暴露
 *     IMU_OnUartRxComplete / IMU_OnUartError, 由 stm32f1xx_it.c 的统一回调按
 *     Instance 分支调度。串口也从 USART2 改到 USART3, 以让出 USART2 给 OpenMV。
 */
#include "imu_driver.h"

/* 串口句柄绑定 (USART3, 让出 USART2 给 OpenMV) */
extern UART_HandleTypeDef huart3;
#define IMU_UART_HANDLE          (&huart3)

/* 内部状态 */

/* 单字节接收缓冲 (中断里填, 解析器读) */
static volatile uint8_t s_rx_byte = 0;

/* 协议解析状态机 */
static uint8_t  s_frame_buf[WIT_FRAME_LEN];  /* 帧组装缓冲 */
static uint8_t  s_frame_idx   = 0;            /* 当前帧已收字节数 */
static uint8_t  s_parse_state = 0;            /* 0=等帧头, 1=收集中 */

/* 最新原始读数 (中断里写, 主循环读) —— volatile 因为跨中断访问 */
static volatile IMU_RawData_t s_imu;
static volatile uint8_t  s_cycle_ready   = 0;  /* 1=收到 0x53, 周期就绪 */
static volatile uint8_t  s_has_cycle     = 0;  /* 1=上电后至少收到过一帧有效角度 */
static volatile uint32_t s_last_data_tick = 0; /* 上次收到完整周期的时刻 */
static volatile uint8_t  s_initialized    = 0; /* 1=IMU_Init 成功 */

/* 诊断计数 */
static volatile uint32_t s_rx_byte_count  = 0;
static volatile uint32_t s_frame_ok_count = 0;
static volatile uint32_t s_crc_err_count  = 0;

/*  内部工具函数  */

/**
 * @brief  小端 2 字节转 int16
 * @param  p  指向 2 字节数组 (低字节在前, 维特协议的小端格式)
 * @return 对应的有符号 16 位整数
 */
static int16_t ToInt16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief  解析一帧 (校验已通过), 按类型更新 s_imu
 *
 *   0x51 加速度: [ax][ax][ay][ay][az][az][temp][temp]
 *   0x52 角速度: [wx][wx][wy][wy][wz][wz][temp][temp]
 *   0x53 角度  : [roll][roll][pitch][pitch][yaw][yaw][版本号×2]
 *
 * @param  f  11 字节帧
 */
static void IMU_ParseFrame(const uint8_t *f)
{
    uint8_t type = f[1];
    switch (type)
    {
        case WIT_TYPE_ACCEL:  /* 0x51 加速度 */
            s_imu.ax   = ToInt16(&f[2]);
            s_imu.ay   = ToInt16(&f[4]);
            s_imu.az   = ToInt16(&f[6]);
            s_imu.temp = ToInt16(&f[8]);
            break;

        case WIT_TYPE_GYRO:   /* 0x52 角速度 */
            s_imu.wx   = ToInt16(&f[2]);
            s_imu.wy   = ToInt16(&f[4]);
            s_imu.wz   = ToInt16(&f[6]);
            s_imu.temp = ToInt16(&f[8]);
            break;

        case WIT_TYPE_ANGLE:  /* 0x53 角度 (一周期结束标志) */
            s_imu.roll  = ToInt16(&f[2]);
            s_imu.pitch = ToInt16(&f[4]);
            s_imu.yaw   = ToInt16(&f[6]);
            /* f[8],f[9] 是版本号, 不是温度, 这里不读 */
            s_last_data_tick = HAL_GetTick();
            s_has_cycle      = 1;
            s_cycle_ready    = 1;   /* 周期就绪 */
            break;

        default:
            break;
    }
}

/**
 * @brief  喂入一个字节, 组帧 + 校验 + 解析
 *
 *   state 0: 等帧头 0x55。收到 0x55 → 进 state 1, 开始组帧。
 *   state 1: 逐字节填入缓冲。收满 11 字节 → 算校验和:
 *            通过 → IMU_ParseFrame; 失败 → 计错。然后回 state 0 等下一帧。
 *
 * @param  b  新收到的字节
 */
static void IMU_FeedByte(uint8_t b)
{
    if (s_parse_state == 0)
    {
        /* 等待帧头 */
        if (b == WIT_FRAME_HEADER)
        {
            s_frame_buf[0] = WIT_FRAME_HEADER;
            s_frame_idx    = 1;
            s_parse_state  = 1;
        }
    }
    else
    {
        /* 收集中 */
        s_frame_buf[s_frame_idx++] = b;
        if (s_frame_idx >= WIT_FRAME_LEN)
        {
            /* 收满 11 字节, 算和校验: (前 10 字节之和) & 0xFF == 第 11 字节 */
            uint16_t sum = 0;
            for (uint8_t i = 0; i < (WIT_FRAME_LEN - 1U); i++)
            {
                sum += s_frame_buf[i];
            }
            if ((sum & 0xFFu) == s_frame_buf[WIT_FRAME_LEN - 1])
            {
                IMU_ParseFrame(s_frame_buf);
                s_frame_ok_count++;
            }
            else
            {
                s_crc_err_count++;
            }
            /* 复位, 等下一帧 */
            s_parse_state = 0;
            s_frame_idx   = 0;
        }
    }
}

/* 中断回调入口 (供 stm32f1xx_it.c 调度) */

void IMU_OnUartRxComplete(void)
{
    /* 每收到 1 字节: 喂入解析器 + 计数 + 重新启动下一次单字节中断接收 */
    IMU_FeedByte(s_rx_byte);
    s_rx_byte_count++;
    /* 重新启动下一次单字节中断接收 (持续接收, 不阻塞) */
    (void)HAL_UART_Receive_IT(IMU_UART_HANDLE, (uint8_t *)&s_rx_byte, 1);
}

void IMU_OnUartError(void)
{
    /* 溢出等错误后重启接收, 保证数据流不断。
     * HAL_UART_Receive_IT 会先复位 huart->RxState 再启动。 */
    (void)HAL_UART_Receive_IT(IMU_UART_HANDLE, (uint8_t *)&s_rx_byte, 1);
}

/* 公共接口实现 */

int IMU_Init(void)
{
    /* ① 配置 USART3 为实际 IMU 波特率 115200 8N1 */
    IMU_UART_HANDLE->Instance          = IMU_UART_INSTANCE;
    IMU_UART_HANDLE->Init.BaudRate     = IMU_UART_BAUDRATE;
    IMU_UART_HANDLE->Init.WordLength   = UART_WORDLENGTH_8B;
    IMU_UART_HANDLE->Init.StopBits     = UART_STOPBITS_1;
    IMU_UART_HANDLE->Init.Parity       = UART_PARITY_NONE;
    IMU_UART_HANDLE->Init.Mode         = UART_MODE_TX_RX;
    IMU_UART_HANDLE->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    IMU_UART_HANDLE->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(IMU_UART_HANDLE) != HAL_OK)
    {
        return -1;
    }

    /* ② 使能 USART3 中断 */
    HAL_NVIC_SetPriority(IMU_UART_IRQn, IMU_UART_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(IMU_UART_IRQn);

    /* ③ 复位协议解析状态 + 计数 */
    s_frame_idx      = 0;
    s_parse_state    = 0;
    s_cycle_ready    = 0;
    s_has_cycle      = 0;
    s_rx_byte_count  = 0;
    s_frame_ok_count = 0;
    s_crc_err_count  = 0;
    s_last_data_tick = 0U;

    /* ④ 启动单字节中断接收 */
    if (HAL_UART_Receive_IT(IMU_UART_HANDLE, (uint8_t *)&s_rx_byte, 1) != HAL_OK)
    {
        return -2;
    }

    s_initialized = 1;
    return 0;
}

uint8_t IMU_IsDataReady(void)
{
    return s_cycle_ready;
}

void IMU_GetRawData(IMU_RawData_t *out)
{
    if (out == 0)
    {
        return;
    }

    /* 关 UART 中断, 原子拷贝, 防止读到"半新半旧"的数据 */
    HAL_NVIC_DisableIRQ(IMU_UART_IRQn);
    out->ax    = s_imu.ax;
    out->ay    = s_imu.ay;
    out->az    = s_imu.az;
    out->wx    = s_imu.wx;
    out->wy    = s_imu.wy;
    out->wz    = s_imu.wz;
    out->roll  = s_imu.roll;
    out->pitch = s_imu.pitch;
    out->yaw   = s_imu.yaw;
    out->temp  = s_imu.temp;
    s_cycle_ready = 0;   /* 清除就绪标志, 等下一周期 */
    HAL_NVIC_EnableIRQ(IMU_UART_IRQn);
}

uint8_t IMU_IsAlive(void)
{
    /* 距上次收到完整周期 < 超时阈值 → 通信正常。
     * uint32_t 减法自动处理 tick 回绕。 */
    return (s_has_cycle &&
            ((HAL_GetTick() - s_last_data_tick) < IMU_DATA_READY_TIMEOUT_MS))
           ? 1u : 0u;
}

uint8_t IMU_IsInitialized(void)
{
    return s_initialized;
}

/* 配置命令发送 */

void IMU_SendConfig(uint8_t reg, uint8_t dataL, uint8_t dataH)
{
    /* 维特配置帧: [0xFF][0xAA][reg][dataL][dataH] */
    uint8_t cmd[WIT_CFG_CMD_LEN] = { WIT_CFG_HEADER0, WIT_CFG_HEADER1, reg, dataL, dataH };
    /* 阻塞发送。TX 与 RX 是独立方向, 不会打断正在进行的中断接收。 */
    (void)HAL_UART_Transmit(IMU_UART_HANDLE, cmd, WIT_CFG_CMD_LEN, IMU_CFG_TX_TIMEOUT_MS);
}

void IMU_Config_Save(void)
{
    /* 保存当前参数到模组 Flash, 掉电不丢 */
    IMU_SendConfig(WIT_REG_SAVE, 0x00, 0x00);
}

void IMU_Config_AccelCalib(void)
{
    /* 加速度校准: 需保持模组水平静止约 1 秒 */
    IMU_SendConfig(WIT_REG_CALIB, 0x01, 0x00);
}

void IMU_Config_GyroCalib(void)
{
    /* 陀螺仪(角速度)校准: 需保持模组静止约 1 秒 */
    IMU_SendConfig(WIT_REG_CALIB, 0x03, 0x00);
}

void IMU_Config_Baudrate(uint8_t code)
{
    /* 修改模组波特率, code 取值见模组手册 (改后需同步改 IMU_UART_BAUDRATE 并重启) */
    IMU_SendConfig(WIT_REG_BAUD, code, 0x00);
}

/* 诊断计数访问 */

uint32_t IMU_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

uint32_t IMU_GetFrameCount(void)
{
    return s_frame_ok_count;
}

uint32_t IMU_GetCrcErrorCount(void)
{
    return s_crc_err_count;
}
