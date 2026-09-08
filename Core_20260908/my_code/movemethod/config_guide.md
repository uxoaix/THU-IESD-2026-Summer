# movemethod 模块参数修改操作手册

## 目录
1. [文件位置与修改原则](#1-文件位置与修改原则)
2. [物理尺寸参数（车体长/宽/高/轮距/轴距/车轮直径）](#2-物理尺寸参数)
3. [运动学与速度参数（最大速度/角速度/加速度/各阶段速度）](#3-运动学与速度参数)
4. [编码器与里程计参数（脉冲数/静止死区）](#4-编码器与里程计参数)
5. [模糊控制参数（对准算法的档位与力度）](#5-模糊控制参数)
6. [超声波与墙体扫描参数](#6-超声波与墙体扫描参数)
7. [执行器时序参数（滚刷/后斗/摄像头舵机）](#7-执行器时序参数)
8. [状态机时序与超时参数](#8-状态机时序与超时参数)
9. [视觉追踪参数（阈值/距离/死区）](#9-视觉追踪参数)
10. [格式规范与修改步骤](#10-格式规范与修改步骤)
11. [修改后验证方法](#11-修改后验证方法)
12. [常见问题与参数关联速查表](#12-常见问题与参数关联速查表)

---

## 1. 文件位置与修改原则

### 📁 修改文件路径
> **唯一需要修改的文件：**
> ```
> d:\STM32\test\Core\my_code\movemethod\config.h
> ```
>
> 所有可配置参数都集中在这一个文件里，其他 9 个文件（motion_strategy.h/c、block_alignment.h/c 等）
> **绝对不要直接改数值**——它们都是从 `config.h` 读取的。

### 🎯 修改原则（三条铁律）
1. **只改 `config.h`**：其他文件是逻辑代码，非参数。误改会造成逻辑错误。
2. **小步慢调**：一次只改 1~3 个参数，改完验证再改下一批。
3. **备份原数值**：改之前把原数值记下来，万一调崩了能回退。

---

## 2. 物理尺寸参数

> 【**什么时候需要改？**】
> 重新设计了车身、换了大轮子、换了更宽的底盘时，都必须改这一组参数。
> 这些参数直接参与差速运动学计算，错了的话：
> - 转 180° 实际只转了 150° 或转了 200°
> - 差速左右转不动 / 原地转圈过猛打滑

| 参数名 | 行号 | 默认值 | 单位 | 说明 | 建议范围 | 调整示例 |
|--------|------|--------|------|------|----------|----------|
| `WHEEL_DIAMETER_CM` | [L165](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L165) | 6.5 | cm | **车轮直径**。实际量轮胎外径（从地面到轮顶 ×2）。 | 3.0 ~ 15.0 | 从 65mm 轮换 90mm 轮：改为 `9.0f` |
| `WHEEL_TRACK_WIDTH_CM` | [L167](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L167) | 15.0 | cm | **左右轮中心距（轮距）**。左轮中心到右轮中心的距离。差速运动学核心参数——决定转多少度要左右轮转多少。 | 8.0 ~ 40.0 | 车宽由 15cm 改到 20cm：改为 `20.0f` |
| `WHEEL_BASE_CM` | [L168](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L168) | 12.0 | cm | **前后轮中心距（轴距）**。前轮轴到后轮轴的距离（当前主要用于参考，未在核心运动学中强制使用）。 | 5.0 ~ 30.0 | 长底盘轴距 18cm：改为 `18.0f` |

### ⚠️ 注意：`WHEEL_CIRCUMFERENCE_CM` 不能直接改
```
#define WHEEL_CIRCUMFERENCE_CM    (3.14159f * WHEEL_DIAMETER_CM)
```
它是**自动计算**的（π × 直径）。要改轮胎周长只改 `WHEEL_DIAMETER_CM`，不要动这个公式。

---

## 3. 运动学与速度参数

> 【**什么时候需要改？**】
> 换了更强的电机 / 更重的电池 / 场地更滑需要降速 / 竞赛要求更快时调整。
> 调错风险：速度太高会打滑、PID 跟不上；速度太低太慢完不成任务。

| 参数名 | 行号 | 默认值 | 单位 | 说明 | 建议范围 | 影响大的场景 |
|--------|------|--------|------|------|----------|-------------|
| `MAX_LINEAR_SPEED_CM_S` | [L180](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L180) | 50.0 | cm/s | **直线速度硬上限**。所有目标速度会被等比例缩放不超过此值。硬件极限（电机/PWM/摩擦）决定。 | 20 ~ 100 | 墙扫 2/3 前进、追踪 |
| `MAX_ANGULAR_SPEED_RADPS` | [L181](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L181) | 1.0 | rad/s | **角速度硬上限**（1.0 rad/s ≈ 57°/秒）。模糊控制输出最后限幅到这个范围内。 | 0.3 ~ 2.0 | 物块对中、追踪修正 |
| `MAX_WHEEL_ACCEL_CM_S2` | [L182](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L182) | 100.0 | cm/s² | **最大轮加速度**。防止停车→全速瞬间输出尖峰让车猛冲。 | 30 ~ 300 | 所有状态切换瞬间 |
| `SEARCH_ROTATION_SPEED_DEG` | [L183](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L183) | 12.0 | 度/秒 | **搜索旋转速度**。找物块/找黑区时原地顺时针转的速度。越大越快，但视觉帧率可能跟不上。 | 5 ~ 30 | 搜索、墙扫旋转 |
| `TRACKING_LINEAR_SPEED` | [L184](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L184) | 25.0 | cm/s | **远距离追踪速度**。目标>60cm 时用这个速度直行。 | 10 ~ 40 | 远距离追踪 |
| `COLLECT_LINEAR_SPEED` | [L185](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L185) | 8.0 | cm/s | **收集时前进速度**。离物块≤20cm 时，慢速靠近 + 启动滚刷。 | 3 ~ 15 | 收集阶段 |
| `WALL_APPROACH_SPEED_CM_S` | [L186](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L186) | 15.0 | cm/s | **墙扫后前进 2/3 距离的速度**。 | 5 ~ 30 | WALL_MEASURE_MOVE |

### 🎯 速度参数调整经验
- **场地地面很滑（泡沫/漆面地板）**：把 `SEARCH_ROTATION_SPEED_DEG` 降到 8，`TRACKING_LINEAR_SPEED` 降到 18。
- **竞赛要求提速**：电机给力的前提下，`SEARCH_ROTATION_SPEED_DEG` 调到 18~20，`TRACKING_LINEAR_SPEED` 调到 30~35。
- **启动后猛冲打滑**：把 `MAX_WHEEL_ACCEL_CM_S2` 从 100 降到 50。

---

## 4. 编码器与里程计参数

> 【**什么时候需要改？**】
> 换了不同脉冲数的编码器 / 车静止时编码器还在缓慢跳数（零漂大）。

| 参数名 | 行号 | 默认值 | 单位 | 说明 | 正确获取方法 |
|--------|------|--------|------|------|-------------|
| `ENCODER_PULSES_PER_REV` | [L169](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L169) | 480 | 脉冲/转 | 编码器每转一圈产生的脉冲总数（含 4 倍频后的值）。 | 查编码器规格书 / 让轮子转一圈数脉冲数 |
| `ODOMETRY_STATIC_DEADZONE_CM_S` | [L170](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L170) | 0.5 | cm/s | **静止零漂死区**。左右轮速绝对值都低于此值视为"车没动"，不积分航向。 | 静止时通过串口看编码器速度读数（如稳定 ±0.3 就设 0.5） |

### ⚠️ 编码器脉冲数正确校验
脉冲数错了的后果：所有轮速实际值 = 读数 × 正确脉冲数/错误脉冲数。
**验证方法**：让车轮正转 10 圈，里程计报告走了多少距离。报告值 ≠ `10 × WHEEL_CIRCUMFERENCE_CM` → 调 `ENCODER_PULSES_PER_REV`。

---

## 5. 模糊控制参数（物块对准）

> 【**什么时候需要改？**】
> 换了不同分辨率的摄像头 / 视场更宽或更窄 / 对中太"肉"或太"猛"需要调手感。
> 模糊控制 = "偏多少 → 转多少"的翻译表。

### 5.1 输入边界与平滑度

| 参数名 | 行号 | 默认值 | 说明 | 调参影响 |
|--------|------|--------|------|---------|
| `FUZZY_X_OFFSET_MAX_PX` | [L208](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L208) | 160 | 视觉半宽（像素）。OpenMV 320 宽半宽=160，QVGA=160。 | 摄像头换 QVGA（240 宽）→ 改为 120 |
| `FUZZY_MF_HALF_WIDTH_PX` | [L209](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L209) | 40 | 三角隶属函数半宽（像素）。越大过渡越平滑，越小切换越"硬"。 | 猛 → 降 30；肉 → 升 50 |

### 5.2 档位中心位置（一般不用改）
```
FUZZY_CENTER_NB = -120  NM = -80  NS = -40  ZE = 0
FUZZY_CENTER_PS =  40   PM =  80  PB = 120
```
默认等距 40 像素一档。只有在改了 `FUZZY_MF_HALF_WIDTH_PX` 且需要移中心时才调。

### 5.3 输出 singleton（角速度档位）⭐ 常调项

| 参数（档 → 方向） | 行号 | 默认值(rad/s) | 度/秒约等于 | 调"猛" | 调"肉" |
|--------------------|------|--------------|-----------|--------|--------|
| `FUZZY_OUT_PB_RADPS`（偏左极大 → 大幅左转） | [L225](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L225) | +0.50 | +28.6° | +0.70 | +0.35 |
| `FUZZY_OUT_PM_RADPS`（偏左中等 → 中等左转） | [L224](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L224) | +0.30 | +17.2° | +0.42 | +0.20 |
| `FUZZY_OUT_PS_RADPS`（偏左稍小 → 稍微左转） | [L223](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L223) | +0.15 | +8.6° | +0.22 | +0.10 |
| `FUZZY_OUT_ZE_RADPS`（正中 → 不转） | [L222](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L222) | 0.00 | 0° | — | — |
| `FUZZY_OUT_NS_RADPS`（偏右稍小 → 稍微右转） | [L221](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L221) | -0.15 | -8.6° | -0.22 | -0.10 |
| `FUZZY_OUT_NM_RADPS`（偏右中等 → 中等右转） | [L220](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L220) | -0.30 | -17.2° | -0.42 | -0.20 |
| `FUZZY_OUT_NB_RADPS`（偏右极大 → 大幅右转） | [L219](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L219) | -0.50 | -28.6° | -0.70 | -0.35 |

> **⚠️ 符号规则（不能改）：正=左转/CCW，负=右转/CW**。7 个档位必须 **PB>PM>PS>ZE>NS>NM>NB**（符号单调递减），否则方向错乱。

---

## 6. 超声波与墙体扫描参数

> 【**什么时候需要改？**】
> 换了不同量程的超声波 / 场地大小变了 / 墙扫总不退出或退出过早。

### 6.1 超声波传感器参数

| 参数名 | 行号 | 默认值 | 单位 | 说明 | 建议 |
|--------|------|--------|------|------|------|
| `WALL_MIN_VALID_DIST_CM` | [L231](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L231) | 5 | cm | **有效距离下限**。传感器盲区以内的数据会被忽略。 | HC-SR04 典型盲区 2~5cm，按实测调整 |
| `WALL_MAX_VALID_DIST_CM` | [L230](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L230) | 400 | cm | **有效距离上限**。超出量程视为无效。 | HC-SR04 典型 400cm；小场地可降为 200 减少无效采样 |
| `WALL_SAMPLE_INTERVAL_MS` | [L232](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L232) | 100 | ms | **采样间隔**。旋转 12°/s × 0.1s = 1.2° 采一次 → 一圈 360° ≈ 300 个样本。 | 采样太多丢帧 → 升到 150；太少定位不准 → 降到 80 |

### 6.2 墙体扫描行为参数

| 参数名 | 行号 | 默认值 | 说明 | 调参影响 |
|--------|------|--------|------|---------|
| `WALL_HEADING_TOL_DEG` | [L238](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L238) | 8 | **大旋转容差（度）**。360° / 180° 等大旋转到位时允许误差 ±8°。 | 航向漂移大（编码器不准）→ 升到 12；很准 → 降到 4 |
| `WALL_SMALL_ROTATE_TOL_DEG` | [L240](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L240) | 1 | **15° 微调容差**（度）。只用 1° 吸收响应滞后。 | 别改！此值小是故意的，改大了微调失真 |
| `SMALL_ROTATION_DEG` | [L239](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L239) | 15 | 微调旋转总角度（度）。 | 需求改为 20° → 改这里；⚠️ 保持奇顺偶逆逻辑不变 |
| `WALL_SCAN_TIMEOUT_SEC` | [L241](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L241) | 20 | 单圈墙扫超时（秒）。默认 12°/s × 30s ≈ 360°，给了很大冗余。 | 一般不用改。转速翻倍 → 降到 10 |
| `WALL_TRAVEL_NUMERATOR` / `DENOMINATOR` | [L242](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L242) / [L243](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L243) | 2 / 3 | 前进距离比例。测到前方墙距为 D，前进 D×2/3 就停。 | 场地很小 → 改 1/2（1/2）；要更靠空区 → 改 3/4（3/4） |

---

## 7. 执行器时序参数

> 【**什么时候需要改？**】
> 换了慢舵机 / 滚刷需要更长时间转完 / 物块滑出速度慢。
> 所有时序参数单位都是 **毫秒 (ms)**。

### 7.1 滚刷控制

| 参数名 | 行号 | 默认值 | 说明 | 调参方法 |
|--------|------|--------|------|---------|
| `BRUSH_ROTATE_DURATION_MS` | [L248](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L248) | 1200 | **滚刷旋转持续时间**（ms）。正常完成计时。 | 转 180° 需要 1.5s → 改 1500；快舵机 0.8s → 改 800 |
| `BRUSH_TIMEOUT_MS` | [L249](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L249) | 3000 | **滚刷超时保护**（ms）。到 3s 还没完成视为卡住，不增加计数直接继续。 | 规则：`BRUSH_TIMEOUT_MS` > `BRUSH_ROTATE_DURATION_MS`，建议 ≥ 2× |

### 7.2 后斗卸载

| 参数名 | 行号 | 默认值 | 说明 | 调参方法 |
|--------|------|--------|------|---------|
| `BUCKET_LIFT_DURATION_MS` | [L254](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L254) | 2000 | **后斗升起时间**（ms）。从 0° 升到最大角度所需时间。 | 慢齿轮舵机 → 改 3000 |
| `UNLOAD_DURATION_MS` | [L255](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L255) | 4000 | **开门卸货保持时间**（ms）。物块数量多/滑出慢 → 延长。 | 5 个物块滑出慢 → 改 5000；很顺滑 → 改 3000 |
| `DOOR_CLOSE_DURATION_MS` | [L256](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L256) | 500 | **后门关闭时间**（ms）。确保关门动作到位。 | 一般不用改，关闭不到位 → 升 800 |

### 7.3 摄像头舵机

| 参数名 | 行号 | 默认值 | 说明 |
|--------|------|--------|------|
| `CAMERA_SERVO_ROTATE_DURATION_MS` | [L261](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L261) | 1000 | 摄像头 0°↔180° 切换时间（ms）。 |
| `CAMERA_SERVO_TIMEOUT_MS` | [L262](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L262) | 2500 | 超时保护。规则：> 2× rotate_duration |
| `CAMERA_SERVO_ANGLE_BLACK` | [L263](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L263) | 180 | 黑色区域阶段的舵机角度（度）。物理安装不同需按实测值校准。 |
| `CAMERA_SERVO_ANGLE_BLOCK` | [L264](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L264) | 0 | 物块阶段的舵机角度（度）。 |

---

## 8. 状态机时序与超时参数

> 【**什么时候需要改？**】
> 比赛场地物块少难找 / 场地太大允许更长搜索时间。

| 参数名 | 行号 | 默认值 | 说明 | 调参经验 |
|--------|------|--------|------|---------|
| `STATE_MACHINE_PERIOD_MS` | [L269](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L269) | 50 | **状态机主周期**（ms）。**⚠️ 绝对不要轻易改！** 所有计时器累加都用它。 | 除非主频大幅下调，否则保持 50 |
| `SEARCH_TIMEOUT_SEC` | [L270](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L270) | 30 | 物块搜索阶段**总超时**（秒）。含墙扫，到时回 INIT 重新开始。 | 大赛场 → 升到 60；小场地 → 降到 20 |
| `SEARCH_NO_BLOCK_TIMEOUT_SEC` | [L271](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L271) | 10 | 连续**看不到物块**触发墙扫的阈值（秒）。 | 场地复杂可能挡住 → 升到 15；视野快 → 降到 8 |
| `BLACK_AREA_SEARCH_TIMEOUT_SEC` | [L272](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L272) | 10 | 连续**看不到黑区**触发墙扫（秒）。 | 同上 |
| `BLACK_AREA_TOTAL_TIMEOUT_SEC` | [L273](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L273) | 120 | 黑区阶段**总超时**（秒）。到时直接去卸货，防止卡死。 | 大赛场 → 升 180 |
| `LOCK_WINDOW_MS` | [L274](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L274) | 800 | **目标丢失锁定窗口**（ms）。追踪中偶尔 1~2 帧识别不到，先停车等一等，超过这个时间才真的回去搜索。 | 识别不稳 → 升 1500；很准 → 降 400 |
| `PRE_CENTERING_TIMEOUT_MS` | [L275](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L275) | 2000 | 对中内部超时（ms）。对中对不上 2s 就放弃继续转。 | 模糊控制太"肉" → 升 3000 |

---

## 9. 视觉追踪参数

> 【**什么时候需要改？**】
> 换了不同镜头焦距（视场变化） / 距离算法重新标定 / 物块大小变化。

| 参数名 | 行号 | 默认值 | 说明 | 调参经验 |
|--------|------|--------|------|---------|
| `TRACKING_DEADZONE_PX` | [L280](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L280) | 15 | **x_offset 死区（像素）**。对中时 ±15 像素内算"已对齐"。 | 对中精度要求高 → 降 8；总左右晃 → 升 25 |
| `TARGET_DISTANCE_MAX_CM` | [L281](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L281) | 60.0 | **远距离阈值**（cm）。超过此距离用全速 25cm/s 追踪，低于就减速。 | 摄像头能识别最远距离的 ~85% |
| `COLLECT_DISTANCE_CM` | [L282](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L282) | 20.0 | **触发收集距离**（cm）。车到这个距离就启动滚刷。 | 滚刷够不到 → 降 15；还没到物块就跑了 → 升 25 |
| `BLACK_AREA_ARRIVE_CM` | [L283](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L283) | 15.0 | **黑区处理阈值**（cm）。车到这个距离就升后斗开门。 | 后斗门位置靠后 → 升 20 |
| `TRACKING_SLOW_RATIO` | [L284](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L284) | 0.8 | **近距离减速系数**。距离≤60cm 时速度 = TRACKING_LINEAR_SPEED × 此系数。 | 0.8 即 20cm/s。要更慢 → 0.6；更快 → 0.9 |
| `TOTAL_OBJECTS_TO_COLLECT` | [L285](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L285) | 5 | **需收集物块总数**。收满就切黑区阶段。 | 赛题要求 3 个 → 改 3 |
| `BLACK_AREA_OBJECT_TYPE` | [L286](file:///d:/STM32/test/Core/my_code/movemethod/config.h#L286) | 3 | **黑色区域类型号**。OpenMV 端 object_type 字段约定值。 | OpenMV 改了枚举 → 同步改 |

---

## 10. 格式规范与修改步骤

### ✅ 正确修改格式

`#define` 格式：
```c
// 正确：参数名   数值    // 注释说明
#define WHEEL_DIAMETER_CM    8.0f     /* 从 6.5cm 轮改为 80mm 直径轮 */
```

**格式规范**：
1. **浮点加 `f` 后缀**：`6.5f`，不能写 `6.5`（double 会浪费 RAM 和算力）。
2. **整型不用后缀**：`480`，不要加 `u/l`（除了故意指定类型）。
3. **宏替换表达式加括号**：`WHEEL_CIRCUMFERENCE_CM (3.14159f * WHEEL_DIAMETER_CM)`，不加括号可能被优先级坑。
4. **单位写在注释里**：每个数值后面注明单位（cm / cm/s / rad/s / ms / 度）。
5. **注释对齐好看**：`/* */` 右对齐，方便快速扫描。

### 📋 修改步骤 SOP

```
步骤 1. 备份 config.h
   → 复制一份命名为 config.h_backup_20260823

步骤 2. 按分类改 1~3 个参数
   → （只改数值，不动宏名）

步骤 3. 保存文件，CubeIDE 重新编译
   → （必须先编译，把错误拦截在编译期）

步骤 4. 编译无错 → 烧录到板卡 → 按 §11 做验证

步骤 5. 验证通过 → 再改下一批参数；验证不通过 → 回退用备份文件
```

### ❌ 常见格式错误（会导致编译错 / 运行错）

| 错误写法 | 为什么错 | 正确写法 |
|---------|---------|---------|
| `#define WHEEL_DIAMETER_CM 8` | 浮点写成整型，差速计算整数除法精度丢失 | `8.0f` |
| `#define WHEEL_DIAMETER_CM 8cm` | 单位不能写在数字里，会编译错 | `8.0f`，单位放注释 |
| `#define MAX_LINEAR_SPEED 50` | 宏名打错（应为 `_CM_S`），别处引用找不到 | `MAX_LINEAR_SPEED_CM_S` |
| `// #define XXX 50` | 注释掉了宏，引用处编译失败 | 去掉 `//` 或给新的数值 |
| 改 `block_alignment.c` 里的 `0.5f` 写死值 | 违反"只改 config.h"原则，下次改完不一致 | 改 `config.h` 对应 `FUZZY_OUT_PB_RADPS` |

---

## 11. 修改后验证方法

不同的参数组有不同验证方法，分类列出：

### 🛞 11.1 物理尺寸 / 轮距 验证（§2 + §4）
**测试项目：直线距离准确性**
```
操作：
  1. 让车从 0 点出发直线走 5 秒，用 `TRACKING_LINEAR_SPEED=25cm/s`
     ⇒ 理论距离 = 5 × 25 = 125 cm
  2. 尺子量实际走了多少 cm
判定：
  实际距离 / 125cm 应在 [0.95, 1.05] 之间（误差≤5%）
  偏差大 → 检查 WHEEL_DIAMETER_CM（最可能的错）、ENCODER_PULSES_PER_REV
```

**测试项目：原地 180° 旋转准确性**
```
操作：
  1. 让车原地执行一次 WALL_TURN_OPPOSITE（收满 5 个后触发或手动触发）
  2. 用手机指南针或地面参考线量车头实际转了多少度
判定：
  实际转了 170°~190° 之间为正常
  转不够（如 150°）→ WHEEL_TRACK_WIDTH_CM 偏小 → 调大
  转过多（如 210°）→ WHEEL_TRACK_WIDTH_CM 偏大 → 调小
```

### 🎚️ 11.2 速度 / 加速度 验证（§3）
```
操作：
  1. 启动后观察 INIT→ROTATE_SEARCH 切换瞬间车轮是否"冲一下打滑"
  2. 追踪模式下速度是否稳定
判定：
  打滑 → 降 MAX_WHEEL_ACCEL_CM_S2 / TRACKING_LINEAR_SPEED
  旋转太慢完不成搜索 → 升 SEARCH_ROTATION_SPEED_DEG
```

### 🎯 11.3 模糊控制验证（§5）
```
操作：
  1. 把物块故意放在视野偏左 120 像素处（NB 档）
  2. 观察对中过程：是"平滑转过去"还是"猛冲一下过冲再回来"
判定：
  太"肉"（5s 还没到中心）→ 升 FUZZY_OUT_PM/PB
  太"猛"（到中心后左右来回晃）→ 降 FUZZY_OUT_PS/PM，或升 TRACKING_DEADZONE_PX
```

### 🧱 11.4 墙扫机制验证（§6）
```
操作：
  1. 把车放在场地角落（三面墙有一面远），故意不放置物块 10s 触发墙扫
  2. 观察：
     - 第一圈 360° 能否完整转完（退出条件正常）
     - 退出 WALL_TURN_OPPOSITE 后车头是否朝向空区域
     - 微调 15° 后是不是方向改了（奇顺偶逆）
     - 最后前进 2/3 距离是否没撞墙
判定：
  没转完就停 → 升 WALL_HEADING_TOL_DEG
  15° 微调看不出来动 → 降 WALL_SMALL_ROTATE_TOL_DEG 到 0
  前进撞墙了 → 改 WALL_TRAVEL_NUMERATOR/DENOMINATOR = 1/2
```

### 🔧 11.5 执行器时序验证（§7）
```
操作：
  1. 单独测试：手动进入 BRUSH_ROTATE_180 → 滚刷是否转够 180° 再停
  2. 收满 5 个物块 → 黑区 → 卸货流程
     - 升后斗 2s → 门开 4s → 关 0.5s
判定：
  滚刷转不到位 → 升 BRUSH_ROTATE_DURATION_MS
  物块还没滑完门就关了 → 升 UNLOAD_DURATION_MS
  后斗没升到位就开门 → 升 BUCKET_LIFT_DURATION_MS
```

---

## 12. 常见问题与参数关联速查表

| 症状（用户观察到的问题） | 可能的参数 | 调整方向 |
|--------------------------|-----------|---------|
| 车直线走 1m 实际走 1.1m | `WHEEL_DIAMETER_CM` | 减 10% |
| 车直线走 1m 实际走 0.9m | `WHEEL_DIAMETER_CM` | 加 10% |
| 原地转 180° 只转了 150° | `WHEEL_TRACK_WIDTH_CM` | 增大 |
| 原地转 180° 转过了到 210° | `WHEEL_TRACK_WIDTH_CM` | 减小 |
| 对中一直左晃右晃（震荡） | `FUZZY_OUT_*_RADPS` 太大 / `TRACKING_DEADZONE_PX` 太小 | 降输出 / 升死区 |
| 对中太慢（5s 还没对准） | `FUZZY_OUT_*_RADPS` 太小 | 升 PS/PM/PB 档输出 |
| 启动瞬间猛冲打滑 | `MAX_WHEEL_ACCEL_CM_S2` | 降到 50 |
| 搜索旋转太慢，OpenMV 漏看物块 | `SEARCH_ROTATION_SPEED_DEG` | 降到 8 |
| 搜索旋转太快，OpenMV 帧率跟不上 | `SEARCH_ROTATION_SPEED_DEG` | 升到 18 |
| 10s 触发墙扫后，2/3 前进撞墙了 | `WALL_TRAVEL_NUMERATOR` / `DENOMINATOR` | 改成 1/2 |
| 滚刷转 1.2s 不够没收集到 | `BRUSH_ROTATE_DURATION_MS` | 升到 1500 |
| 货门开了 4s 物块还没滑完 | `UNLOAD_DURATION_MS` | 升到 5000 |
| 车静止放着，航向自己慢慢变 | `ODOMETRY_STATIC_DEADZONE_CM_S` | 升到 1.0 |
| 追踪距离 100cm 就减速了 | `TARGET_DISTANCE_MAX_CM` | 升到 80~100 |
| 离 25cm 就启动滚刷，距离不够 | `COLLECT_DISTANCE_CM` | 降到 15 |
| 对中对不上 2s 就放弃了 | `PRE_CENTERING_TIMEOUT_MS` | 升到 3000 |

---

## 附录：修改影响链（改一个参数可能影响哪些功能）

```
WHEEL_DIAMETER_CM
   └─→ WHEEL_CIRCUMFERENCE_CM (自动)
       └─→ 轮速计算 (所有运动)
           └─→ 里程计航向 (Odometry_Update)
               └─→ 墙扫旋转判断 (所有 WALL_* 状态)
                    ⚠️ 高影响参数！

WHEEL_TRACK_WIDTH_CM
   └─→ 差速运动学 (DifferentialDrive_Control)
       └─→ 里程计航向
           └─→ 旋转到位判断 (所有 WALL_* + 搜索)
                ⚠️ 高影响参数！

FUZZY_OUT_*_RADPS (7 个)
   └─→ 模糊对准 (PRE_CENTERING / TRACKING / BLACK_AREA_TRACK)
       只影响对中手感，不改变运动学
            ⭐ 低风险，适合新手调
```
