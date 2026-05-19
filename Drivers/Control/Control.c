#include "ti_msp_dl_config.h"
#include "Grayscale.h"
#include "Control.h"
#include "motor.h"
#include "clock.h"
#include <stdint.h>
#include <stdbool.h>

/* PID 控制器实例 */
PID_t pid_turn;       // 转向环PID: 灰度偏差 → 差速
PID_t pid_speed_L;    // 左轮速度环PID: 目标速度→PWM
PID_t pid_speed_R;    // 右轮速度环PID: 目标速度→PWM

/*
 * 灰度偏差计算 — 改进版(查表+场景识别)
 *
 * 返回偏差值(归一化到 -400~+400), 正值=偏右, 负值=偏左
 * count_out: 输出检测到的传感器数量
 * pattern_out: 输出8位二进制模式
 */
float Grayscale_CalcError(uint8_t *sensor_values,
                          uint8_t *count_out, uint8_t *pattern_out)
{
    uint8_t pattern = 0;
    uint8_t count   = 0;
    int32_t sum_pos = 0;  // 位置加权和
    int32_t sum_w   = 0;  // 权重和

    /* 将8路传感器打包成二进制模式并计算加权位置 */
    for (int i = 0; i < GRAYSCALE_CHANNELS; i++)
    {
        if (sensor_values[i])
        {
            pattern |= (1 << (7 - i));  // bit7=左, bit0=右
            count++;
            sum_pos += i;  // 传感器位置: 0=最左, 7=最右
            sum_w++;
        }
    }

    if (count_out)  *count_out  = count;
    if (pattern_out) *pattern_out = pattern;

    /* ---- 根据检测到的传感器数量分类处理 ---- */
    if (count == 0)
    {
        /* 丢线: 无法返回有效偏差, 标记特殊值 */
        return 999.0f;
    }
    else if (count <= 2)
    {
        /* 正常循迹: 1~2个传感器在线, 用加权平均算精确位置 */
        /* 传感器位置 0~7, 中心在 3.5, 映射到 -350~+350 */
        float center = (float)sum_pos / sum_w;        // 0.0~7.0
        return (center - 3.5f) * 100.0f;              // -350~+350
    }
    else if (count <= 5)
    {
        /* 路口/急弯: 3~5个传感器触发, 置零偏差直行通过 */
        return 0.0f;
    }
    else
    {
        /* 全黑/终点: 6个以上触发, 返回超大偏差触发停车 */
        return 9999.0f;
    }
}

/* 串口0发送单个字符 */
void uart0_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_BLUETEETH_INST ) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_BLUETEETH_INST , ch);
}

/* 串口0发送字符串 */
void uart0_send_string(char* str)
{
    while(str && *str)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_send_char(*str++);
    }
}

/* Forward declarations */
float PID_Calculate(PID_t *pid, float error);
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit);

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

/* Parse "TAG,dist=50,angle=-10" — updates g_k230_data */
void k230_parse_line(uint8_t *line)
{
    if (!line) return;

    g_k230_data.distance_cm = -1;
    g_k230_data.angle_deg   = 0;

    if (line[0] != 'T' || line[1] != 'A' || line[2] != 'G')
        return;

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
            if (*p == '\0') break;
        }
        if (p[0] == 'a' && p[1] == 'n' && p[2] == 'g' && p[3] == 'l' && p[4] == 'e' && p[5] == '=')
        {
            int val = 0;
            int sign = 1;
            p += 6;
            if (*p == '-') { sign = -1; p++; }
            while (*p >= '0' && *p <= '9') { val = val * 10 + (*p - '0'); p++; }
            g_k230_data.angle_deg = (int16_t)(val * sign);
            if (*p == '\0') break;
        }
        p++;
    }
}

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
#define DIST_PID_TARGET     20
#define DIST_PID_KP         2.0f
#define DIST_PID_KI         0.05f
#define DIST_PID_KD         0.0f
#define DIST_PID_INTEGRAL_LIMIT   10.0f
#define DIST_PID_OUTPUT_LIMIT     20.0f

#define FOLLOW_SPEED_MIN    20
#define FOLLOW_SPEED_MAX    60
#define FOLLOW_SPEED_DEFAULT 40

void distance_pid_init(float Kp, float Ki, float Kd,
                       float integral_limit, float output_limit)
{
    PID_Init(&pid_distance, Kp, Ki, Kd, integral_limit, output_limit);
}

float distance_pid_calculate(int16_t current_dist, int16_t target_dist)
{
    if (current_dist < 0)
        return 0.0f;

    float error = (float)(current_dist - target_dist);
    return PID_Calculate(&pid_distance, error);
}

/* ===== Overtaking ===== */
void overtaking_start(void)
{
    g_follower_state  = FOLLOWER_OVERTAKING;
    g_ov_phase        = OV_PHASE_SWERVE_LEFT;
    g_ov_tick_start   = tick_ms;

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
        Motor_SetSpeed(OV_SPEED_SLOW, OV_SPEED_FAST);

        if (elapsed >= OV_TIME_SWERVE_LEFT)
        {
            g_ov_phase      = OV_PHASE_PASS;
            g_ov_tick_start = tick;
        }
        break;

    case OV_PHASE_PASS:
        Motor_SetSpeed(OV_SPEED_CRUISE, OV_SPEED_CRUISE);

        if (elapsed >= OV_TIME_PASS)
        {
            g_ov_phase      = OV_PHASE_SWERVE_RIGHT;
            g_ov_tick_start = tick;
        }
        break;

    case OV_PHASE_SWERVE_RIGHT:
        Motor_SetSpeed(OV_SPEED_FAST, OV_SPEED_SLOW);

        if (elapsed >= OV_TIME_SWERVE_RIGHT)
        {
            g_ov_phase      = OV_PHASE_DONE;
            g_ov_tick_start = tick;
        }
        break;

    case OV_PHASE_DONE:
        Motor_Stop();
        g_follower_state = FOLLOWER_FOLLOWING;
        buzzer_off();
        break;
    }
}
