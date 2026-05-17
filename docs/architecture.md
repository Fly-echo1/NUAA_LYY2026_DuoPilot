# 软件架构

## 目录结构

```
car.102/
├── main.c                     # 主程序入口，while(1) 主循环
├── main.h                     # 主头文件，集中包含所有模块
├── mspm0-modules.syscfg       # SysConfig 外设配置
│
├── Drivers/
│   ├── Control/               # v2.0 控制系统 (状态机/距离PID/超车)
│   │   ├── Control.c
│   │   └── Control.h
│   ├── MSPM0/                 # 底层 MSPM0 驱动
│   │   ├── clock.c/.h         # 时钟与 SysTick (tick_ms)
│   │   └── interrupt.c/.h     # 中断处理 + K230 环形缓冲区
│   ├── PID/                   # 位置式 PID 控制器
│   │   ├── pid.c/.h
│   ├── Motor/                 # 电机 PWM 驱动
│   │   ├── motor.c/.h
│   ├── Encoder/               # 编码器脉冲采集
│   │   ├── encoder.c/.h
│   ├── Grayscale/             # 8 路灰度传感器
│   │   ├── Grayscale.c/.h
│   ├── OLED_Software_I2C/     # OLED 显示 (软件 I2C)
│   │   ├── oled_software_i2c.c/.h
│   ├── WIT/                   # WIT 姿态传感器 (预留)
│   ├── MPU6050/               # MPU6050 驱动 (预留)
│   ├── VL53L0X/               # 激光测距 (预留)
│   └── ...                    # 其他选装驱动
│
├── Debug/                     # 编译输出 + SysConfig 生成文件
│   ├── ti_msp_dl_config.c/.h  # 外设配置 (自动生成)
│   ├── device.opt
│   └── ccsObjs.opt
│
├── docs/                      # 文档
│   └── ...
├── targetConfigs/             # CCS 调试配置
└── .cproject / .project       # CCS 项目文件
```

---

## 主循环流程图

```
Reset ──► SYSCFG_DL_init()
           SysTick_Init()
           OLED_Init()
           Motor_Init()
           Encoder_Init()
           Interrupt_Init()
           PID_Init()
           distance_pid_init()
           Buzzer Startup Beep
           OLED 显示标签
                    │
                    ▼
            ┌───────────────┐
            │  while(1)     │ ◄──────────────────────┐
            │  (≈50Hz)      │                        │
            └───────┬───────┘                        │
                    │                                │
                    ▼                                │
            ┌───────────────┐                        │
            │ ① 读取灰度     │                        │
            └───────┬───────┘                        │
                    │                                │
                    ▼                                │
            ┌───────────────┐                        │
            │ ② K230 数据    │                        │
            │   (环形缓冲)    │                        │
            └───────┬───────┘                        │
                    │                                │
                    ▼                                │
            ┌───────────────┐                        │
            │ ③ BT "OV"     │──── OV? ──► overtaking │
            │   指令检测     │        │   _start()    │
            └───────┬───────┘        │               │
                    │                ▼               │
                    ▼         ┌───────────┐          │
            ┌───────────────┐ │ OVERTAKING│          │
            │ ④ 状态机派发   │ │ 状态      │          │
            └───┬───┬───────┘ │           │          │
         FOLLOW │   │ OV'ING │ overtaking │          │
                │   └─────────►_update()  │          │
                │              │ OLED 显示 │          │
                │              └─────┬─────┘          │
                ▼                    │                │
        ┌───────────────┐           delay(20ms) ──────┘
        │ ⑤ 灰度偏差     │
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ ⑥ 距离 PID    │   (自适应巡航)
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ ⑦ 转向环 PID  │
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ ⑧ 目标速度    │   差速 + 距离修正
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ ⑨ 速度环 PID  │   编码器反馈
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ ⑩ OLED 显示   │
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ ⑪ BT 调试输出  │
        └───────┬───────┘
                │
                ▼
        ┌───────────────┐
        │ delay(20ms)   │────────► 回到 ①
        └───────────────┘
```

---

## 中断系统

| 中断 | 触发条件 | 频率 | 功能 |
|------|---------|------|------|
| `SysTick_Handler` | SysTick 溢出 | 1ms | `tick_ms++` 全局时间戳 |
| `TIMG6_IRQHandler` | 定时器计数归零 | 50ms | 采样编码器脉冲 → 速度值 |
| `GROUP1_IRQHandler` | GPIO 中断 | 编码器 A/B 上升沿 | 四倍频计数，方向判别 |
| `UART1_IRQHandler` | UART1 RX | K230 数据到达 | 逐字节写入环形缓冲区 |

---

## 模块依赖关系

```
main.c
 ├── main.h ──┬── clock.h       (tick_ms)
 │            ├── interrupt.h   (k230_read_line)
 │            ├── motor.h       (Motor_SetSpeed/Stop)
 │            ├── encoder.h     (encoder_left/right_speed)
 │            ├── pid.h         (PID_Init/Calculate)
 │            └── oled_software_i2c.h
 ├── Control.h ──┬── pid.h
 │               ├── Grayscale.h
 │               └── motor.h
 ├── Grayscale.h
 └── ti_msp_dl_config.h
```

## 双车系统拓扑

```
┌───────── 前车 ─────────────────────┐
│  K230(红标志检测) ──UART──► MSPM0  │
│                                    │
│  MSPM0 ──UART(BT)──► HC-05 发射    │
└────────────────────────────────────┘
                    │ "OV\n"
                    ▼
┌───────── 后车 ─────────────────────┐
│  HC-05 接收 ──UART──► MSPM0        │
│                                    │
│  K230(AprilTag) ──UART──► MSPM0    │
│                                    │
│  MSPM0 ──► OLED / 蜂鸣器 / 电机    │
└────────────────────────────────────┘
```
