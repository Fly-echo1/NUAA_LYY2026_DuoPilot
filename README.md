# DuoPilot — Dual-Car Cooperative Overtaking System

DuoPilot is a dual-car cooperative driving system built on **TI MSPM0G3507** and **Canaan K230**, designed for the NUAA "Tianmu Qihang" innovation project. The system features autonomous line-following, adaptive distance keeping via AprilTag detection, and signal-flag-triggered overtaking.

## Features

- **Autonomous Line Following** — 8-channel grayscale sensor with cascade PID control (turn + speed)
- **Adaptive Cruise Control** — K230 detects lead car's AprilTag; follower adjusts speed to maintain target distance
- **Signal Flag Triggered Overtaking** — Red flag detected by lead car's K230 triggers Bluetooth "OV" command → follower executes open-loop overtaking maneuver
- **Dual-Processor Architecture** — MSPM0 handles real-time motor control; K230 handles vision processing
- **Real-Time Telemetry** — OLED status display + Bluetooth UART debug output
- **Buzzer Feedback** — Buzzer active during overtaking

## System Architecture

```
Lead Car                          Follow Car
┌────────────────────┐           ┌────────────────────┐
│ K230               │           │ K230               │
│  ├─ Red flag       │           │  ├─ AprilTag       │
│  │  color detect   │           │  │  detection      │
│  └─ UART→MSPM0     │           │  └─ UART→MSPM0     │
│       FLAG,1/0     │           │       TAG,dist,angle│
└────────┬───────────┘           └────────┬───────────┘
         │                                │
    ┌────▼────┐                      ┌────▼────┐
    │ MSPM0   │                      │ MSPM0   │
    │ G3507   │                      │ G3507   │
    │         │◄── Bluetooth HC-05 ──┤         │
    │ 循迹    │      "OV\n"          │ 循迹    │
    │ 跟随    │                      │ 超车    │
    └─────────┘                      └─────────┘
```

## Communication Protocol

### K230 → MSPM0 (UART)

```
TAG,dist=<cm>,angle=<deg>\n    — Lead car AprilTag data (follower side)
FLAG,1\n                        — Red flag detected (lead car side)
FLAG,0\n                        — Red flag lost (lead car side)
```

### Lead MSPM0 → Follow MSPM0 (Bluetooth HC-05)

```
OV\n    — Trigger overtaking on follower
```

## Follower State Machine

```
                    BT "OV" received
    ┌──────────────────────────────────────┐
    │                                      │
    ▼                         Done         │
┌─────────┐     ┌────────────┐     ┌───────┘
│FOLLOWING│────→│ OVERTAKING │────→┐
│ (idle)  │     │ (buzzer ON) │     │
└────┬────┘     └────────────┘     │
     │ ▲                           │
     │ └───────────────────────────┘
     │     auto resume following
```

### Overtaking Sequence (open-loop)

| Phase | Left Motor | Right Motor | Duration | Buzzer |
|-------|-----------|-------------|----------|--------|
| ① Swerve left | 30 | 70 | ~300ms | ON |
| ② Pass | 60 | 60 | ~1500ms | ON |
| ③ Swerve right | 70 | 30 | ~300ms | ON |
| ④ Resume | — | — | auto | OFF |

## Hardware Requirements

- **TI MSPM0G3507** LaunchPad × 2 (lead + follower)
- **Canaan K230** (Canmv) × 2
- **HC-05** Bluetooth module × 2
- **Grayscale sensor array** — 8-channel multiplexed digital
- **DC motors + H-Bridge** — with quadrature encoder feedback
- **OLED display** — 128×64, I2C
- **Buzzer** — active
- **AprilTag** — printed tag mounted on lead car rear

## Project Structure

```
E:/TI/project/new/car.102/
├── main.c                     # Firmware main loop
├── main.h                     # Project header
├── Drivers/
│   ├── Control/               # PID controllers, overtaking logic
│   ├── Encoder/               # Quadrature encoder reading
│   ├── Grayscale/             # 8-ch grayscale sensor driver
│   ├── Motor/                 # PWM motor control
│   ├── PID/                   # Positional PID implementation
│   ├── MSPM0/                 # Clock, interrupts, SysTick
│   ├── OLED_Software_I2C/     # OLED display driver
│   ├── WIT/                   # WIT IMU driver
│   ├── BNO08X_UART_RVC/       # BNO08X orientation sensor
│   ├── IMU660RB/              # IMU660RB + Fusion lib
│   ├── LSM6DSV16X/            # LSM6DSV16X IMU
│   ├── MPU6050/               # MPU6050 + DMP
│   ├── Ultrasonic_Capture/    # Ultrasonic (capture mode)
│   ├── Ultrasonic_GPIO/       # Ultrasonic (GPIO mode)
│   └── VL53L0X/               # VL53L0X ToF sensor
├── mspm0-modules.syscfg       # SysConfig hardware config
├── targetConfigs/             # CCS target configuration
└── docs/design/               # Design documents
```

## Development Environment

- **IDE**: Code Composer Studio (TI)
- **SDK**: TI MSPM0 SDK
- **Toolchain**: ARM GCC / TI CLANG
- **Vision SDK**: Canaan K230 SDK

## License

This project is developed for educational purposes at Nanjing University of Aeronautics and Astronautics (NUAA).
