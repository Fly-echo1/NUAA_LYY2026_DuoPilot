# 引脚映射

## MSPM0G3507 引脚分配

> 基于 SysConfig (`mspm0-modules.syscfg`) 配置

### UART

| 外设 | 接口 | TX | RX | 用途 |
|------|------|----|----|------|
| UART0 | HC-05 蓝牙 | PA0 | PA1 | 调试输出 + 超车指令接收 (9600) |
| UART1 | K230 视觉 | PB6 | PB7 | AprilTag 数据接收 (9600) |
| UART_WIT | WIT 姿态 | — | — | IMU 通讯 (选装) |

### 电机 PWM

| 信号 | GPIO | 说明 |
|------|------|------|
| 左 PWM | — | TIMG 定时器输出，可调频率 10~20kHz |
| 右 PWM | — | TIMG 定时器输出，可调频率 10~20kHz |
| 左方向 | — | GPIO 控制 TB6612 方向引脚 |
| 右方向 | — | GPIO 控制 TB6612 方向引脚 |

### 编码器

| 信号 | GPIO | 中断 | 说明 |
|------|------|------|------|
| 左编码器 ChA | PA21 | GPIOA 上升沿 | 左轮 AB 相计数 |
| 左编码器 ChB | PA22 | GPIOA 上升沿 | 左轮方向判别 |
| 右编码器 ChA | PA24 | GPIOA 上升沿 | 右轮 AB 相计数 |
| 右编码器 ChB | PA23 | GPIOA 上升沿 | 右轮方向判别 |

编码器中断全部归入 **GROUP1_IRQHandler**，通过 `DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)` 派发。

### 灰度传感器

| 通道 | 说明 |
|------|------|
| 8 路模拟输入 | GPIO 配置为模拟模式 |
| 采集方式 | 逐通道轮询 ADC 或 GPIO 数字读取 (配置可选) |

### OLED (软件 I2C)

| 信号 | GPIO | 说明 |
|------|------|------|
| SCL | — | 软件模拟 I2C 时钟 |
| SDA | — | 软件模拟 I2C 数据 |

### 蜂鸣器

| 信号 | GPIO | 说明 |
|------|------|------|
| BEEP | GPIO_BEEP_PIN_BEEP_PIN | 有源蜂鸣器，高电平驱动 |

### 编码器速度采样定时器

| 外设 | 周期 | 说明 |
|------|------|------|
| TIMG6 | 50ms | 定时读取编码器脉冲计数值 |
