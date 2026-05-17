# 传感器驱动

## 灰度传感器 (Grayscale)

8 路数字灰度传感器，用于检测赛道黑线。

### 数据读取

```c
uint8_t sensor_values[GRAYSCALE_CHANNELS];  // 8 路数据
Grayscale_Sensor_Read_All(sensor_values);
```

### 偏差计算

```c
float deviation = Grayscale_CalcError(sensor_values, &sensor_count, &pattern);
```

- **返回**: 浮点偏差值，范围约 `[-1000, 1000]`
- **全白**: 返回 `10000`，触发 STOP
- **模式**: 检测丢线、弯道、十字等特征

### OLED 显示

灰度值实时显示在 OLED 最后一行，格式为 8 位二进制字符串：
```
11000111  ← 8 路传感器值 ('1'=白, '0'=黑)
```

## IMU 姿态传感器

### BNO08X (UART RVC 模式)

- 串口通信，输出欧拉角/四元数
- 内置姿态融合算法

### IMU660RB

- 6 轴 IMU + Fusion 库
- 支持 AHRS 姿态解算

### MPU6050 + DMP

- 6 轴 + 硬件 DMP 引擎
- 直接输出四元数/欧拉角

## 测距模块

| 模块 | 接口 | 量程 | 精度 |
|------|------|------|------|
| VL53L0X | I2C | 0~2m | ±3% |
| HC-SR04 (Capture) | 输入捕获 | 2cm~4m | ±1cm |
| HC-SR04 (GPIO) | GPIO 模拟 | 2cm~4m | ±1cm |
