# 调试与日志

## UART 调试输出

系统通过 UART 以 **50Hz** 频率输出运行数据。

### 输出格式

```
dev:+23 L: 45 R: 38 cnt:5 SL: 45 SR: 42
dev:+25 L: 47 R: 36 cnt:4 SL: 46 SR: 41
dev:+20 L: 44 R: 39 cnt:5 SL: 44 SR: 43
```

| 字段 | 含义 | 范围 |
|------|------|------|
| dev | 赛道偏差 | ±1000 |
| L | 左电机 PWM | 0-100 |
| R | 右电机 PWM | 0-100 |
| cnt | 检测到的传感器数 | 0-8 |
| SL | 左编码器速度 | 0-100 |
| SR | 右编码器速度 | 0-100 |

## 蜂鸣器

```c
// 开机蜂鸣提示 (500Hz, 100ms)
DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
delay_cycles(3200000);
DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
```

## 日志分析技巧

```python
# Python 数据可视化示例
import serial, matplotlib.pyplot as plt

ser = serial.Serial('COM3', 115200)
dev, l, r = [], [], []

while len(dev) < 500:
    line = ser.readline().decode().strip()
    parts = line.split()
    dev.append(int(parts[0].split(':')[1]))
    l.append(int(parts[1].split(':')[1]))
    r.append(int(parts[2].split(':')[1]))

plt.plot(dev, label='Deviation')
plt.plot(l, label='Motor L')
plt.plot(r, label='Motor R')
plt.legend(); plt.show()
```
