# 驱动模块详解

## Control — v2.0 控制核心

> 文件: `Drivers/Control/Control.c` / `Control.h`

v2.0 新增的控制模块，包含后车状态机、AprilTag 数据解析、距离 PID、蜂鸣器控制和超车逻辑。

### 后车状态机

```c
typedef enum {
    FOLLOWER_FOLLOWING = 0,   // 跟随模式: 循迹 + 距离保持
    FOLLOWER_OVERTAKING       // 超车模式: 开环时序控制
} FollowerState_t;
```

| 状态 | 默认 | 进入条件 | 退出条件 |
|------|------|---------|---------|
| `FOLLOWING` | ✓ | 上电 / 超车完成 | 收到 "OV" 蓝牙指令 |
| `OVERTAKING` | — | 收到 "OV" | 四相位执行完毕 |

### K230 AprilTag 数据解析

```c
typedef struct {
    int16_t distance_cm;   // 距前车距离 (cm), -1 表示无效
    int16_t angle_deg;     // 相对角度 (°)
} K230_Data_t;
```

**协议格式**: `TAG,dist=50,angle=-10\n`

| 字段 | 范围 | 说明 |
|------|------|------|
| `dist` | 0~200 cm | 前车 AprilTag 距离 |
| `angle` | -90~+90° | 相对角度，正值偏右 |

**解析流程**:
1. 主循环调用 `k230_read_line()` 从环形缓冲区提取一行
2. 调用 `k230_parse_line()` 解析字段，更新 `g_k230_data`
3. 无效或错误数据自动置 `distance_cm = -1`

### 超车四相位

```c
typedef enum {
    OV_PHASE_SWERVE_LEFT,   // 向左变道
    OV_PHASE_PASS,           // 直行超车
    OV_PHASE_SWERVE_RIGHT,  // 向右回原道
    OV_PHASE_DONE            // 完成
} OvertakingPhase_t;
```

| 相位 | 时间 | 左轮 | 右轮 | 效果 |
|------|------|------|------|------|
| SWERVE_LEFT | 300ms | 30 | 70 | 差速左转 |
| PASS | 1500ms | 60 | 60 | 直行超越 |
| SWERVE_RIGHT | 300ms | 70 | 30 | 差速右转 |
| DONE | — | STOP | STOP | 停车 + 蜂鸣器关 |

### 蜂鸣器

| 函数 | 说明 |
|------|------|
| `buzzer_on()` | 超车开始/进行中持续鸣响 |
| `buzzer_off()` | 超车完成后关闭 |

---

## K230 UART 环形缓冲区

> 文件: `Drivers/MSPM0/interrupt.c` / `interrupt.h`

128 字节环形缓冲区，中断方式无阻塞接收 K230 数据。

```
            写入方向 (中断)
               │
               ▼
┌────┬────┬────┬────┬────┬────┬────┬────┐
│    │    │ $  │ A  │ G  │ ,  │ d  │    │ ...
└────┴────┴────┴────┴────┴────┴────┴────┘
               ▲
               │
          读取方向 (主循环)
```

| 参数 | 值 |
|------|-----|
| 缓冲区大小 | 128 字节 |
| 行缓冲区大小 | 128 字节 |
| 中断触发 | UART1 RX (任意字节到达) |
| 行终止符 | `\n` |
| 临界区保护 | `__disable_irq()` / `__set_PRIMASK()` |

---

## 灰度传感器

> 文件: `Drivers/Grayscale/Grayscale.c` / `Grayscale.h`

8 路模拟输出灰度传感器，检测黑线位置。

### 偏差算法

```
传感器排列 (从左到右):
  S0  S1  S2  S3  S4  S5  S6  S7
  0   1   2   3   4   5   6   7

偏差 = (加权平均位置 - 3.5) × 100
范围: -350 ~ +350 (负=偏左, 正=偏右)
```

### 多传感器场景处理

| 触发数 | 场景 | 行为 |
|--------|------|------|
| 0 | 丢线 | 返回 999.0，保持上一帧偏差 |
| 1~2 | 正常循迹 | 加权平均计算精确偏差 |
| 3~5 | 路口/急弯 | 返回 0.0，直行通过 |
| 6~8 | 终点/全黑 | 返回 9999.0，触发停车 |

---

## 编码器

> 文件: `Drivers/Encoder/encoder.c` / `encoder.h`

N20 电机霍尔编码器，AB 相四倍频计数。

| 参数 | 值 |
|------|-----|
| 分辨率 | 13 脉冲/圈 (AB 相 → 52 计数/圈) |
| 采样周期 | 50ms (TIMG6 定时器) |
| 方向判别 | ChA 上升沿时采样 ChB 电平 |
| 读取方式 | `extern volatile int16_t encoder_left/right_speed` |

---

## OLED 显示

> 文件: `Drivers/OLED_Software_I2C/oled_software_i2c.c`

0.96" 128×64 单色 OLED，软件 I2C 驱动。

### 显示布局

```
行 0: Dev:+0042     ← 偏差值
行 1:
行 2: L: 45   R: 38 ← PWM 输出
行 3:
行 4: P:  5.1 I: 2.3 D: -0.8  ← PID 分量
行 5:
行 6: SPD:45 D:18   ← 左轮速度 + 前车距离
行 7: 11011010      ← 灰度二进制模式
```

超车模式下，第 0 行显示 `OV00`~`OV03` (当前相位编号)。

---

## PID 控制器

> 文件: `Drivers/PID/pid.c` / `pid.h`

位置式 PID，支持积分限幅和输出限幅。

```c
typedef struct {
    float Kp, Ki, Kd;          // PID 系数
    float integral;             // 积分累计值
    float last_error;           // 上次误差 (微分用)
    float integral_limit;       // 积分限幅
    float output_limit;         // 输出限幅
} PID_t;

float PID_Calculate(PID_t *pid, float error);
```
