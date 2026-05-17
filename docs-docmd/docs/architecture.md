# 软件架构

## 模块结构

```
car.102/
├── main.c                    # 主循环：传感器读取 → PID 计算 → 电机控制
├── main.h                    # 头文件包含
├── Drivers/
│   ├── BNO08X_UART_RVC/      # BNO08X 姿态传感器
│   ├── Control/               # 控制逻辑 (按键、蜂鸣器)
│   ├── Encoder/               # 霍尔编码器读取
│   ├── Grayscale/             # 8 路灰度传感器
│   ├── IMU660RB/              # 姿态融合算法
│   ├── LSM6DSV16X/            # 6 轴 IMU 驱动
│   ├── MPU6050/               # MPU6050 + DMP 驱动
│   ├── MSPM0/                 # 时钟和中断配置
│   ├── Motor/                 # 直流电机 PWM 驱动
│   ├── OLED_* /               # OLED 显示 (I2C/SPI 多版本)
│   ├── PID/                   # PID 控制器
│   ├── Ultrasonic_*/          # 超声波测距
│   ├── VL53L0X/               # 激光测距
│   └── WIT/                   # WIT 串口通信
└── mspm0-modules.syscfg      # SysConfig 硬件配置
```

## 主循环流程

```
初始化:
  ├── SYSCFG_DL_init()       — 系统时钟与外设
  ├── SysTick_Init()          — SysTick 定时器
  ├── OLED_Init()             — OLED 屏幕
  ├── WIT_Init()              — 姿态传感器
  ├── Motor_Init()            — 电机 PWM
  ├── Encoder_Init()          — 编码器
  ├── Interrupt_Init()        — 中断配置
  └── PID_Init() ×3           — 转向 PID + 左右速度 PID

主循环 (20ms 周期):
  ┌─────────────────────────────────┐
  │  Grayscale_Sensor_Read_All()    │ ← 读取 8 路灰度值
  │  Grayscale_CalcError()          │ ← 计算赛道偏差
  │  ┌─ 丢线检测?                   │
  │  │  ├─ 全白 → STOP              │
  │  │  ├─ 部分丢线 → 保持上次偏差   │
  │  │  └─ 连续 >10次 → 停车        │
  │  PID_Calculate(&pid_turn)       │ ← 转向 PID
  │  计算目标速度 L/R                │
  │  PID_Calculate(&pid_speed_L)    │ ← 左速度 PID
  │  PID_Calculate(&pid_speed_R)    │ ← 右速度 PID
  │  CLAMP 限幅                      │
  │  Motor_SetSpeed()               │ ← 输出 PWM
  │  OLED 刷新显示                   │
  │  UART 日志输出                   │
  └─────────────────────────────────┘
```

## 中断系统

| 中断 | 触发源 | 功能 |
|------|--------|------|
| TIMER_Encoder_Read | 定时器 | 定期读取编码器值 |
| TIMER_Control | 定时器 | 控制周期管理 |
| GPIO 按键 | 外部中断 | 模式切换 / 急停 |
