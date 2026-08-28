/**
 * @file    block_alignment.c
 * @brief   物块对准模块实现 (水平模糊控制 + 垂直俯仰跟踪)
 *
 * 【整体组成】
 *   ① 水平模糊控制 (x_offset_px → 角速度): 7 档模糊集 + 加权平均解模糊
 *   ② 垂直俯仰跟踪 (y_offset_px → 舵机角度): 比例控制 + 死区 + 限速
 *
 * 【水平模糊控制流程 (3 步)】
 *   ① 算隶属度: 输入 x_offset_px 属于 7 个档位的"程度"各是多少 (0~1)
 *      类比: 给当前偏移量打 7 张选票, 每张 0~1 分。
 *            例如 x=-60, 可能 NS 档 0.5 票、NM 档 0.5 票 (跨在两档之间)。
 *
 *   ② 查规则表: 每个档位对应一个输出角速度 (singleton)。
 *      规则: 偏左 → 左转 (正), 偏右 → 右转 (负), 正中 → 0。
 *      例如 NS (稍偏左) → +0.15 rad/s (稍微左转)。
 *
 *   ③ 加权平均解模糊: 把 7 张选票 × 对应输出, 求平均, 得最终角速度。
 *      类比: 选举计票——每张选票的力度 × 候选人的政策立场, 加权平均得结果。
 *
 * 【垂直俯仰跟踪流程】
 *   ① 死区: |y_offset_px| < DEADZONE → 不调整 (防抖)
 *   ② 比例: 新角度 = 当前角度 + y_offset_px × KP
 *   ③ 限幅: 钳位到 [MIN, MAX]
 *   ④ 限速: 单周期变化 ≤ MAX_STEP
 *   ⑤ 仅整数变化时才发指令
 */
#include "block_alignment.h"

/* 垂直俯仰跟踪内部状态 *
 * 记录当前已发送给舵机的角度, 用于:
 *   - 限速: 新角度相对旧角度的变化不超过 MAX_STEP
 *   - 死区返回时不重复发指令 (保持上次的值)
 */
static uint16_t s_tilt_angle = CAMERA_TILT_CENTER_DEG;  /* 当前指令角度 (度) */

/**
 * @brief  三角隶属函数 (计算输入属于某档的程度)
 *
 * @param  x           输入值 (例如 x_offset_px)
 * @param  center      档位中心 (例如 NB 档中心是 -120)
 * @param  half_width  半宽 (距离 center 多少像素内才算"属于"这个档)
 * @return 隶属度 [0, 1], 1=完全属于, 0=完全不属于
 */
static float TriangularMF(float x, float center, float half_width)
{
    float diff = x - center;
    /* 超过半宽 → 隶属度 0 (完全不属于) */
    if (diff < -half_width || diff > half_width) {
        return 0.0f;
    }
    /* 在半宽内 → 线性递减到 0 */
    float abs_diff = (diff < 0.0f) ? -diff : diff;
    return 1.0f - abs_diff / half_width;
}

/**
 * @brief   模糊控制物块对准 (主函数)
 *
 * 【完整流程】
 *   1. 输入越界保护: 把 x_offset_px 限制在 [-160, +160] 内
 *   2. 死区判断: |x_offset_px| < 15 像素 → 返回 0 (避免抖动)
 *   3. 算 7 个档位的隶属度
 *   4. 加权平均: Σ(隶属度 × 对应输出) / Σ隶属度
 *   5. 输出限幅: 限制在 ±1.0 rad/s 内
 *
 * @param   x_offset_px  目标水平偏移 (像素, 负=偏左, 正=偏右)
 * @return  角速度修正 (rad/s, 正=左转, 负=右转)
 */
float BlockAlignment_GetAngularCorrection(int16_t x_offset_px)
{
    /* 输入越界保护:
     * FUZZY_X_OFFSET_MAX_PX=160 为 OpenMV 半宽, 超出此范围的输入
     * 会使所有隶属函数返回 0 (sum_mu=0 触发安全返回), 但提前 clamp
     * 可使 FUZZY_X_OFFSET_MAX_PX 常量生效, 符合"参数集中且实际使用"原则。 */
    if (x_offset_px >  FUZZY_X_OFFSET_MAX_PX) x_offset_px =  FUZZY_X_OFFSET_MAX_PX;
    if (x_offset_px < -FUZZY_X_OFFSET_MAX_PX) x_offset_px = -FUZZY_X_OFFSET_MAX_PX;

    /* 死区内不修正, 避免中心附近方向抖动:
     * 目标在正中央 ±15 像素内, 视为"已对准", 不再微调, 防止车左右晃。 */
    if (x_offset_px > -TRACKING_DEADZONE_PX && x_offset_px < TRACKING_DEADZONE_PX) {
        return 0.0f;
    }

    const float x  = (float)x_offset_px;
    const float hw = (float)FUZZY_MF_HALF_WIDTH_PX;

    /* 1. 计算 7 个模糊集的隶属度 */
    const float mu_NB = TriangularMF(x, (float)FUZZY_CENTER_NB, hw);  /* 极大偏左的程度 */
    const float mu_NM = TriangularMF(x, (float)FUZZY_CENTER_NM, hw);  /* 中等偏左的程度 */
    const float mu_NS = TriangularMF(x, (float)FUZZY_CENTER_NS, hw);  /* 稍偏左的程度 */
    const float mu_ZE = TriangularMF(x, (float)FUZZY_CENTER_ZE, hw);  /* 正中的程度 */
    const float mu_PS = TriangularMF(x, (float)FUZZY_CENTER_PS, hw);  /* 稍偏右的程度 */
    const float mu_PM = TriangularMF(x, (float)FUZZY_CENTER_PM, hw);  /* 中等偏右的程度 */
    const float mu_PB = TriangularMF(x, (float)FUZZY_CENTER_PB, hw);  /* 极大偏右的程度 */

    /* 2. 规则表: x 偏左 → ω 正 (左转CCW), x 偏右 → ω 负 (右转CW)
     * 每个档位对应一个固定的输出角速度 (singleton):
     *   NB → PB (偏左极大 → 大幅左转, +0.5 rad/s)
     *   NM → PM (偏左中等 → 中等左转, +0.3 rad/s)
     *   NS → PS (偏左稍小 → 稍微左转, +0.15 rad/s)
     *   ZE → ZE (正中     → 不转,     0 rad/s)
     *   PS → NS (偏右稍小 → 稍微右转, -0.15 rad/s)
     *   PM → NM (偏右中等 → 中等右转, -0.3 rad/s)
     *   PB → NB (偏右极大 → 大幅右转, -0.5 rad/s) */
    const float sum_mu = mu_NB + mu_NM + mu_NS + mu_ZE + mu_PS + mu_PM + mu_PB;
    /* 除零保护: 所有隶属度都为 0 (输入完全在边界外) → 安全返回 0 */
    if (sum_mu < 1e-6f) {
        return 0.0f;
    }

    /* 加权求和: 每个档位的"力度" × 对应输出, 累加得分母分子 */
    const float sum_w = mu_NB * FUZZY_OUT_PB_RAD_S   /* NB → PB: 偏左极大 × 左转极大 */
                      + mu_NM * FUZZY_OUT_PM_RAD_S   /* NM → PM */
                      + mu_NS * FUZZY_OUT_PS_RAD_S   /* NS → PS */
                      + mu_ZE * FUZZY_OUT_ZE_RAD_S   /* ZE → ZE: 正中 × 不转 */
                      + mu_PS * FUZZY_OUT_NS_RAD_S   /* PS → NS */
                      + mu_PM * FUZZY_OUT_NM_RAD_S   /* PM → NM */
                      + mu_PB * FUZZY_OUT_NB_RAD_S;  /* PB → NB: 偏右极大 × 右转极大 */

    /* 3. 加权平均解模糊
     * 最终角速度 = Σ(隶属度 × 输出) / Σ隶属度
     * 类比: 选举计票——总票数 Σ(mu), 每张票权重对应一个政策立场,
     * 加权平均得最终"政策方向" (角速度)。 */
    float omega = sum_w / sum_mu;

    /* 4. 输出限幅
     * 防止极端情况下输出超限 (硬件保护) */
    if (omega >  MAX_ANGULAR_SPEED_RAD_S) omega =  MAX_ANGULAR_SPEED_RAD_S;
    if (omega < -MAX_ANGULAR_SPEED_RAD_S) omega = -MAX_ANGULAR_SPEED_RAD_S;
    return omega;
}

/* 垂直俯仰跟踪实现 */

void BlockAlignment_ResetCameraTilt(void)
{
    /* 复位到正前方 (CAMERA_TILT_CENTER_DEG = 90°)
     * 立即发送指令, 舵机自行平滑转到目标。 */
    s_tilt_angle = CAMERA_TILT_CENTER_DEG;
    Servo_Camera_Tilt(CAMERA_TILT_CENTER_DEG);
}

void BlockAlignment_UpdateCameraTilt(int16_t y_offset_px)
{
    /* ① 死区: y_offset_px 在死区内不调整, 保持当前角度 (防中心附近抖动)
     *    类比 x_offset_px 的 TRACKING_DEADZONE_PX, 一致使用 15 像素。 */
    if (y_offset_px > -CAMERA_TILT_DEADZONE_PX &&
        y_offset_px <  CAMERA_TILT_DEADZONE_PX) {
        return;
    }

    /* ② 比例控制: 算出"想要的"新角度
     *   y > 0 (物块偏下) → correction 正 → 增大角度 (俯视把物块拉回中央)
     *   y < 0 (物块偏上) → correction 负 → 减小角度 (仰视)
     *   以当前已指令角度为基准累加 (积分式), 使小修正逐周期累积到位。 */
    float correction = (float)y_offset_px * CAMERA_TILT_KP_DEG_PER_PX;
    float new_angle  = (float)s_tilt_angle + correction;

    /* ③ 限幅: 钳位到 [MIN, MAX] (防超出俯仰舵机机械范围) */
    if (new_angle < (float)CAMERA_TILT_MIN_DEG) new_angle = (float)CAMERA_TILT_MIN_DEG;
    if (new_angle > (float)CAMERA_TILT_MAX_DEG) new_angle = (float)CAMERA_TILT_MAX_DEG;

    /* ④ 限速: 单周期变化不超过 MAX_STEP 度
     *   即使 y_offset_px 突变 (如刚检测到物块), 舵机也只逐步转动, 不会猛甩,
     *   避免视野抖动和目标丢失。多周期累积最终会到达目标角度。 */
    float delta = new_angle - (float)s_tilt_angle;
    if (delta >  (float)CAMERA_TILT_MAX_STEP_DEG) delta =  (float)CAMERA_TILT_MAX_STEP_DEG;
    if (delta < -(float)CAMERA_TILT_MAX_STEP_DEG) delta = -(float)CAMERA_TILT_MAX_STEP_DEG;

    /* ⑤ 仅整数角度变化时才发指令 (减少总线流量, 避免无意义的微指令) */
    uint16_t target_angle = (uint16_t)((float)s_tilt_angle + delta);
    if (target_angle != s_tilt_angle) {
        s_tilt_angle = target_angle;
        Servo_Camera_Tilt(s_tilt_angle);
    }
}
