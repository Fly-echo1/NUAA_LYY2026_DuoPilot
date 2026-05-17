# DuoPilot Overtaking — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the follower (rear) car firmware changes for AprilTag-based distance keeping and signal-flag-triggered overtaking on MSPM0G3507.

**Architecture:** Extend existing cascade PID line-follower with a new UART RX path for K230 AprilTag data (`TAG,dist=xx,angle=xx\n`) and a Bluetooth RX command path for overtaking trigger (`OV\n`). Add a simple state machine (FOLLOWING → OVERTAKING → FOLLOWING) in the main loop.

**Tech Stack:** C, TI MSPM0 SDK, Code Composer Studio, MSPM0G3507

**Prerequisites:** SysConfig already defines `UART_VISION_INST` (UART1, 9600 baud, GPIOB 6/7). No SysConfig changes needed.

---

### Task 1: K230 UART RX Ring Buffer + Interrupt

**Files:**
- Modify: `Drivers/MSPM0/interrupt.c`
- Modify: `Drivers/MSPM0/interrupt.h`

This task adds a simple ring buffer to capture K230 UART data. K230 sends lines like `TAG,dist=50,angle=-10\n` at ~20Hz. A ring buffer with line extraction is the simplest approach.

- [ ] **Step 1: Add ring buffer and K230 driver declarations to interrupt.h**

```c
#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include <stdint.h>
#include <stdbool.h>

extern uint8_t enable_group1_irq;

void Interrupt_Init(void);

/* ===== K230 UART Ring Buffer ===== */
#define K230_RX_BUF_SIZE    128
#define K230_LINE_BUF_SIZE  64

extern volatile uint8_t  k230_rx_buf[K230_RX_BUF_SIZE];
extern volatile uint16_t k230_rx_head;
extern volatile uint16_t k230_rx_tail;

/* Get a complete line from the ring buffer (newline-terminated).
 * Returns true if a line was copied to 'line', false if no line available.
 * The line is NOT null-terminated; 'len' is set to the line length. */
bool k230_read_line(uint8_t *line, uint16_t *len);

#endif /* #ifndef _INTERRUPT_H_ */
```

- [ ] **Step 2: Implement ring buffer and UART1 IRQ handler in interrupt.c**

Add to `Drivers/MSPM0/interrupt.c` (after existing includes):

```c
/* ===== K230 UART Ring Buffer ===== */
volatile uint8_t  k230_rx_buf[K230_RX_BUF_SIZE];
volatile uint16_t k230_rx_head = 0;
volatile uint16_t k230_rx_tail = 0;

bool k230_read_line(uint8_t *line, uint16_t *len)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();

    bool found = false;
    uint16_t tail = k230_rx_tail;
    uint16_t bytes = 0;

    /* Scan ring buffer for '\n' starting from tail */
    while (tail != k230_rx_head && bytes < K230_LINE_BUF_SIZE - 1)
    {
        uint8_t c = k230_rx_buf[tail];
        line[bytes++] = c;
        tail = (tail + 1) % K230_RX_BUF_SIZE;

        if (c == '\n')
        {
            found = true;
            break;
        }
    }

    if (found)
    {
        line[bytes] = '\0';           /* null-terminate */
        k230_rx_tail = tail;         /* consume the line */
    }

    __set_PRIMASK(key);
    return found;
}
```

Add the UART1 IRQ handler (after `TIMG6_IRQHandler`):

```c
/* UART1_IRQHandler — K230 AprilTag data RX */
void UART1_IRQHandler(void)
{
    uint32_t status = DL_UART_Main_getInterruptStatus(UART_VISION_INST);

    if (status & DL_UART_MAIN_INTERRUPT_RX)
    {
        while (!DL_UART_isRXFIFOEmpty(UART_VISION_INST))
        {
            uint8_t data = DL_UART_Main_receiveData(UART_VISION_INST);
            uint16_t next = (k230_rx_head + 1) % K230_RX_BUF_SIZE;
            if (next != k230_rx_tail)  /* buffer not full */
            {
                k230_rx_buf[k230_rx_head] = data;
                k230_rx_head = next;
            }
        }
    }
}
```

- [ ] **Step 3: Enable UART1 RX interrupt in main.c's initialization section**

Find the `/* 开启编码器 50ms 速度采样定时器 */` block in main.c, add after it:

```c
/* 开启 K230 UART RX 中断 */
NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
DL_UART_Main_enableInterrupt(UART_VISION_INST, DL_UART_MAIN_INTERRUPT_RX);
```

- [ ] **Step 4: Commit**

```bash
git add Drivers/MSPM0/interrupt.c Drivers/MSPM0/interrupt.h main.c
git commit -m "feat: add K230 UART ring buffer and RX interrupt"
```

---

### Task 2: Control Module — State Machine + Data Types

**Files:**
- Modify: `Drivers/Control/Control.h`
- Modify: `Drivers/Control/Control.c`

This task defines the follower state machine, K230 data structure, and declares all new control functions.

- [ ] **Step 1: Add new types and declarations to Control.h**

```c
#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

/* PID controller instances */
extern PID_t pid_turn;       /* Turn PID: grayscale deviation → differential speed */
extern PID_t pid_speed_L;    /* Left wheel speed PID */
extern PID_t pid_speed_R;    /* Right wheel speed PID */

/* ===== Follower State Machine ===== */
typedef enum {
    FOLLOWER_FOLLOWING = 0,
    FOLLOWER_OVERTAKING
} FollowerState_t;

/* ===== K230 AprilTag Data ===== */
typedef struct {
    int16_t distance_cm;     /* -1 = invalid/no tag */
    int16_t angle_deg;       /* -128..127, negative = left, positive = right */
} K230_Data_t;

/* ===== Overtaking Phases (open-loop) ===== */
typedef enum {
    OV_PHASE_SWERVE_LEFT = 0,
    OV_PHASE_PASS,
    OV_PHASE_SWERVE_RIGHT,
    OV_PHASE_DONE
} OvertakingPhase_t;

/* Global state */
extern FollowerState_t   g_follower_state;
extern OvertakingPhase_t g_ov_phase;
extern uint32_t          g_ov_tick_start;  /* SysTick when current phase started */
extern K230_Data_t       g_k230_data;

/* ===== Function Declarations ===== */

/* Grayscale deviation (existing) */
float Grayscale_CalcError(uint8_t *sensor_values,
                          uint8_t *count_out, uint8_t *pattern_out);

/* UART debug (existing) */
void uart0_send_char(char ch);
void uart0_send_string(char* str);

/* K230 line parser — parse "TAG,dist=50,angle=-10" into g_k230_data */
void k230_parse_line(uint8_t *line);

/* Distance PID — compute speed correction from AprilTag distance */
void distance_pid_init(float Kp, float Ki, float Kd,
                       float integral_limit, float output_limit);
float distance_pid_calculate(int16_t current_dist, int16_t target_dist);

/* Overtaking control */
void overtaking_start(void);          /* Enter overtaking, turn on buzzer */
void overtaking_update(uint32_t tick);/* Run phase machine, call each main loop */
bool overtaking_is_done(void);        /* True when back in FOLLOWING */

/* Buzzer */
void buzzer_on(void);
void buzzer_off(void);

#endif /* __CONTROL_H */
```

- [ ] **Step 2: Add missing includes to top of Control.c**

Update the includes at the top of Control.c:

```c
#include "ti_msp_dl_config.h"
#include "Grayscale.h"
#include "Control.h"
#include "motor.h"    /* Motor_SetSpeed, Motor_Stop */
#include "clock.h"    /* tick_ms */
```

- [ ] **Step 3: Add global definitions to Control.c**

Append after existing PID_t definitions (after the `PID_t pid_speed_R;` line):

```c
/* ===== Follower State Machine ===== */
FollowerState_t   g_follower_state = FOLLOWER_FOLLOWING;
OvertakingPhase_t g_ov_phase       = OV_PHASE_SWERVE_LEFT;
uint32_t          g_ov_tick_start  = 0;
K230_Data_t       g_k230_data      = { .distance_cm = -1, .angle_deg = 0 };

/* Distance PID */
static PID_t pid_distance;

/* Overtaking timing constants (SysTick ms) */
#define OV_TIME_SWERVE_LEFT  300
#define OV_TIME_PASS        1500
#define OV_TIME_SWERVE_RIGHT 300

/* Overtaking speed constants (0-100) */
#define OV_SPEED_SLOW    30
#define OV_SPEED_FAST    70
#define OV_SPEED_CRUISE  60
```

- [ ] **Step 4: Implement K230 line parser in Control.c**

```c
/* Parse "TAG,dist=50,angle=-10" — updates g_k230_data */
void k230_parse_line(uint8_t *line)
{
    /* Reset on invalid data */
    g_k230_data.distance_cm = -1;
    g_k230_data.angle_deg   = 0;

    /* Expect line format: "TAG,dist=50,angle=-10" or "TAG,dist=0,angle=0"  */
    if (line[0] != 'T' || line[1] != 'A' || line[2] != 'G')
        return;

    /* Find "dist=" */
    uint8_t *p = line;
    while (*p && *p != '\n')
    {
        if (p[0] == 'd' && p[1] == 'i' && p[2] == 's' && p[3] == 't' && p[4] == '=')
        {
            int val = 0;
            int sign = 1;
            p += 5;
            if (*p == '-') { sign = -1; p++; }
            while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
            g_k230_data.distance_cm = (int16_t)(val * sign);
        }
        if (p[0] == 'a' && p[1] == 'n' && p[2] == 'g' && p[3] == 'l' && p[4] == 'e' && p[5] == '=')
        {
            int val = 0;
            int sign = 1;
            p += 6;
            if (*p == '-') { sign = -1; p++; }
            while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
            g_k230_data.angle_deg = (int16_t)(val * sign);
        }
        p++;
    }
}
```

- [ ] **Step 5: Commit**

```bash
git add Drivers/Control/Control.h Drivers/Control/Control.c
git commit -m "feat: add follower state machine and K230 data parser"
```

---

### Task 3: Distance PID + Buzzer Controls

**Files:**
- Modify: `Drivers/Control/Control.c`

This task implements the adaptive distance-keeping PID and buzzer hardware control.

- [ ] **Step 1: Find buzzer GPIO defines**

Check `ti_msp_dl_config.h` for buzzer GPIO names. Based on existing `main.c`:

```c
/* Already used in main.c: */
/* DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN); */
/* DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN); */
```

- [ ] **Step 2: Add buzzer and distance PID functions to Control.c**

```c
/* ===== Buzzer ===== */
void buzzer_on(void)
{
    DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
}

void buzzer_off(void)
{
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
}

/* ===== Distance PID ===== */
/* Target follow distance: 20 cm */
#define DIST_PID_TARGET     20
#define DIST_PID_KP         2.0f
#define DIST_PID_KI         0.05f
#define DIST_PID_KD         0.0f
#define DIST_PID_INTEGRAL   10.0f
#define DIST_PID_OUTPUT     20.0f

/* Base speed range during following */
#define FOLLOW_SPEED_MIN    20
#define FOLLOW_SPEED_MAX    60
#define FOLLOW_SPEED_DEFAULT 40

void distance_pid_init(float Kp, float Ki, float Kd,
                       float integral_limit, float output_limit)
{
    PID_Init(&pid_distance, Kp, Ki, Kd, integral_limit, output_limit);
}

/* Returns speed correction to add to base_speed.
 *   positive = speed up (too far), negative = slow down (too close) */
float distance_pid_calculate(int16_t current_dist, int16_t target_dist)
{
    if (current_dist < 0)
        return 0.0f;               /* No tag — no correction */

    float error = (float)(current_dist - target_dist);
    return PID_Calculate(&pid_distance, error);
}
```

- [ ] **Step 3: Commit**

```bash
git add Drivers/Control/Control.c
git commit -m "feat: add distance PID and buzzer control"
```

---

### Task 4: Overtaking Open-Loop Sequence

**Files:**
- Modify: `Drivers/Control/Control.c`

This task implements the open-loop overtaking phase machine.

- [ ] **Step 1: Add overtaking state machine to Control.c**

```c
/* ===== Overtaking ===== */
void overtaking_start(void)
{
    g_follower_state  = FOLLOWER_OVERTAKING;
    g_ov_phase        = OV_PHASE_SWERVE_LEFT;
    g_ov_tick_start   = tick_ms;     /* from clock.h, SysTick ms counter */

    buzzer_on();
}

bool overtaking_is_done(void)
{
    return (g_follower_state == FOLLOWER_FOLLOWING);
}

void overtaking_update(uint32_t tick)
{
    if (g_follower_state != FOLLOWER_OVERTAKING)
        return;

    uint32_t elapsed = tick - g_ov_tick_start;

    switch (g_ov_phase)
    {
    case OV_PHASE_SWERVE_LEFT:
        /* Left wheel slow, right wheel fast → swerve left off line */
        Motor_SetSpeed(OV_SPEED_SLOW, OV_SPEED_FAST);

        if (elapsed >= OV_TIME_SWERVE_LEFT)
        {
            g_ov_phase      = OV_PHASE_PASS;
            g_ov_tick_start = tick;
        }
        break;

    case OV_PHASE_PASS:
        /* Both wheels same speed → drive straight past lead car */
        Motor_SetSpeed(OV_SPEED_CRUISE, OV_SPEED_CRUISE);

        if (elapsed >= OV_TIME_PASS)
        {
            g_ov_phase      = OV_PHASE_SWERVE_RIGHT;
            g_ov_tick_start = tick;
        }
        break;

    case OV_PHASE_SWERVE_RIGHT:
        /* Left wheel fast, right wheel slow → swerve right back to line */
        Motor_SetSpeed(OV_SPEED_FAST, OV_SPEED_SLOW);

        if (elapsed >= OV_TIME_SWERVE_RIGHT)
        {
            g_ov_phase      = OV_PHASE_DONE;
            g_ov_tick_start = tick;
        }
        break;

    case OV_PHASE_DONE:
        /* Stop motors briefly then return to FOLLOWING */
        Motor_Stop();
        g_follower_state = FOLLOWER_FOLLOWING;
        buzzer_off();
        break;
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add Drivers/Control/Control.c
git commit -m "feat: add overtaking open-loop phase machine"
```

---

### Task 5: Main Loop — State Machine + Integration

**Files:**
- Modify: `main.c`

This task wires everything together in the main loop: parse K230 data, run distance PID in FOLLOWING mode, dispatch overtaking sequence on Bluetooth "OV" command.

- [ ] **Step 1: Rewrite main.c with new state machine**

The file `main.c` needs the following changes:

1. After existing globals, add:
```c
/* K230 line buffer for parsing */
static uint8_t k230_line[K230_LINE_BUF_SIZE];
static uint16_t k230_line_len;
```

2. In the init section (after `NVIC_EnableIRQ(TIMER_Encoder_Read_INST_INT_IRQN);`), add:
```c
/* 开启 K230 UART RX 中断 */
NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
DL_UART_Main_enableInterrupt(UART_VISION_INST, DL_UART_MAIN_INTERRUPT_RX);

/* 初始化距离PID */
distance_pid_init(2.0f, 0.05f, 0.0f, 10.0f, 20.0f);
```

3. Replace the entire `while (1)` loop body with:

```c
    while (1)
    {
        /* ========== 0. 循环局部队列，用于OLED/UART显示 ========== */
        int16_t display_motor_L = 0, display_motor_R = 0;
        int16_t display_base_speed = FOLLOW_SPEED_DEFAULT;

        /* ========== 1. 读取灰度传感器 ========== */
        Grayscale_Sensor_Read_All(sensor_values);

        /* ========== 2. 读取并解析 K230 AprilTag 数据 ========== */
        if (k230_read_line(k230_line, &k230_line_len))
        {
            k230_parse_line(k230_line);
        }

        /* ========== 3. 计算灰度偏差 ========== */
        uint8_t sensor_count, pattern;
        float deviation = Grayscale_CalcError(sensor_values, &sensor_count, &pattern);

        if (deviation >= 9990.0f)
        {
            /* 全黑/终点: 停车 */
            Motor_Stop();
            sprintf((char *)oled_buffer, "STOP");
            OLED_ShowString(5*6, 0, oled_buffer, 8);
            uart0_send_string("STOP\r\n");
            mspm0_delay_ms(100);
            continue;
        }

        /* ========== 4. 检查蓝牙超车指令 ========== */
        if (DL_UART_isRXFIFOEmpty(UART_BLUETEETH_INST) == false)
        {
            uint8_t bt_cmd = DL_UART_Main_receiveData(UART_BLUETEETH_INST);
            if ((bt_cmd == 'O' || bt_cmd == 'o') &&
                g_follower_state == FOLLOWER_FOLLOWING)
            {
                overtaking_start();
                uart0_send_string("OV_START\r\n");
            }
        }

        /* ========== 5. 状态机派发 ========== */
        if (g_follower_state == FOLLOWER_FOLLOWING)
        {
            /* ---- 5a. 转向环 PID (灰度偏差 → 差速) ---- */
            float turn_output = PID_Calculate(&pid_turn, deviation);

            /* ---- 5b. 距离PID (AprilTag → 速度修正) ---- */
            float speed_correction = distance_pid_calculate(
                g_k230_data.distance_cm, DIST_PID_TARGET);

            /* 目标速度 = 基础速度 + 距离修正 */
            int16_t base = FOLLOW_SPEED_DEFAULT + (int16_t)speed_correction;
            base = CLAMP(base, FOLLOW_SPEED_MIN, FOLLOW_SPEED_MAX);
            display_base_speed = base;

            int16_t target_speed_L = base + (int16_t)turn_output;
            int16_t target_speed_R = base - (int16_t)turn_output;
            target_speed_L = CLAMP(target_speed_L, 0, 100);
            target_speed_R = CLAMP(target_speed_R, 0, 100);

            /* ---- 5c. 速度环 PID (编码器反馈) ---- */
            float speed_out_L = PID_Calculate(&pid_speed_L,
                                  (float)(target_speed_L - encoder_left_speed));
            float speed_out_R = PID_Calculate(&pid_speed_R,
                                  (float)(target_speed_R - encoder_right_speed));

            float motor_L = (float)target_speed_L + speed_out_L;
            float motor_R = (float)target_speed_R + speed_out_R;
            motor_L = CLAMP(motor_L, 0.0f, 100.0f);
            motor_R = CLAMP(motor_R, 0.0f, 100.0f);

            display_motor_L = (int16_t)motor_L;
            display_motor_R = (int16_t)motor_R;

            Motor_SetSpeed(display_motor_L, display_motor_R);
        }
        else /* FOLLOWER_OVERTAKING */
        {
            overtaking_update(tick_ms);
        }

        /* ========== 6. OLED 显示 ========== */
        sprintf((char *)oled_buffer, "%s",
                g_follower_state == FOLLOWER_OVERTAKING ? "OV" : "  ");
        OLED_ShowString(0, 0, oled_buffer, 8);
        sprintf((char *)oled_buffer, "%+4d", (int16_t)deviation);
        OLED_ShowString(3*6, 0, oled_buffer, 8);
        /* distance */
        if (g_k230_data.distance_cm >= 0)
            sprintf((char *)oled_buffer, "D%-3d", g_k230_data.distance_cm);
        else
            sprintf((char *)oled_buffer, "D---");
        OLED_ShowString(12*6, 0, oled_buffer, 8);

        sprintf((char *)oled_buffer, "%-3d", display_motor_L);
        OLED_ShowString(3*6, 2, oled_buffer, 8);
        sprintf((char *)oled_buffer, "%-3d", display_motor_R);
        OLED_ShowString(9*6, 2, oled_buffer, 8);

        /* 编码器速度 */
        sprintf((char *)oled_buffer, "%-3d", encoder_left_speed);
        OLED_ShowString(5*6, 6, oled_buffer, 8);

        /* 灰度位图 */
        for (int i = 0; i < GRAYSCALE_CHANNELS; i++)
            grayscale_str[i] = sensor_values[i] ? '1' : '0';
        grayscale_str[GRAYSCALE_CHANNELS] = '\0';
        OLED_ShowString(0, 7, (uint8_t *)grayscale_str, 8);

        /* ========== 7. 蓝牙调试 ========== */
        sprintf((char *)oled_buffer,
                "dev:%+4d dist:%3d spd:%3d SL:%3d SR:%3d\r\n",
                (int16_t)deviation, g_k230_data.distance_cm,
                display_base_speed,
                encoder_left_speed, encoder_right_speed);
        uart0_send_string((char *)oled_buffer);

        /* ========== 8. 控制周期 ≈ 50Hz ========== */
        mspm0_delay_ms(20);
    }
```

- [ ] **Step 2: Commit**

```bash
git add main.c
git commit -m "feat: integrate follower state machine with K230 and BT commands"
```

---

### Task 6: Verify Build

**Files:** None (build only)

- [ ] **Step 1: Build in CCS**

Open project in Code Composer Studio. Clean build.
Expected: No errors, no warnings.

- [ ] **Step 2: Test on hardware**

Flash to MSPM0G3507 follower car. Verify:
1. Serial output appears on Bluetooth UART at 9600 baud
2. Sending "TAG,dist=30,angle=0\n" on UART_VISION pins → OLED shows distance
3. Sending 'O' via Bluetooth → car enters overtaking sequence, buzzer sounds
4. After overtaking completes → car returns to following, buzzer off
