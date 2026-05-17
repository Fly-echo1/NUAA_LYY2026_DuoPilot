# PID 控制

## 三路 PID 控制器

系统采用 **串级 PID** 结构，一个转向 PID 输出作为两个速度 PID 的目标输入：

```
                     ┌─────────────┐
  偏差 ─────────────►│  转向 PID   │───► 目标速度差
                     │ Kp=0.8      │      +
                     │ Ki=0.005    │      │
                     │ Kd=0.3      │      ▼
                     └─────────────┘   ┌──────────────┐
                     ┌─────────────┐   │ 左速度 PID   │
  目标速度 L ◄───────┤  目标速度   │◄──│ Kp=0.5       │◄── 编码器反馈 L
                     │  分配       │   │ Ki=0.01      │
                     └─────────────┘   │ Kd=0.1       │
                                       └──────────────┘
                     ┌─────────────┐
  目标速度 R ◄───────┤  目标速度   │   ┌──────────────┐
                     │  分配       │──►│ 右速度 PID   │
                     └─────────────┘   │ Kp=0.5       │◄── 编码器反馈 R
                                       │ Ki=0.01      │
                                       │ Kd=0.1       │
                                       └──────────────┘
```

## PID 参数

| 控制器 | Kp | Ki | Kd | 输出限幅 | 积分限幅 |
|--------|-----|------|------|----------|----------|
| 转向 PID | 0.8 | 0.005 | 0.3 | ±30 | ±60 |
| 左速度 PID | 0.5 | 0.01 | 0.1 | ±20 | ±50 |
| 右速度 PID | 0.5 | 0.01 | 0.1 | ±20 | ±50 |

## 积分分离

```c
if (fabs(error) <= integral_limit) {
    integral += error * Ki;
    integral = CLAMP(integral, -integral_limit, integral_limit);
} else {
    integral = 0;  // 误差过大时清除积分
}
```

## 速度合成

```c
// 基础速度
const int16_t base_speed = 40;

// 转向量叠加
target_speed_L = base_speed + turn_output;
target_speed_R = base_speed - turn_output;

// 限幅到 [0, 100]
target_speed_L = CLAMP(target_speed_L, 0, 100);
target_speed_R = CLAMP(target_speed_R, 0, 100);
```

## 控制周期

主循环运行在 **20ms** 周期，即控制频率为 **50Hz**。
