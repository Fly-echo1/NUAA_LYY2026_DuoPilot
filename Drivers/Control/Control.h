#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"

/* PID 控制器实例 (在 Control.c 中定义) */
extern PID_t pid_turn;       // 转向环PID: 灰度偏差 → 差速
extern PID_t pid_speed_L;    // 左轮速度环PID: 目标速度→PWM
extern PID_t pid_speed_R;    // 右轮速度环PID: 目标速度→PWM

/*
 * 灰度偏差计算
 * 返回偏差值(归一化到 -400~+400), 正值=偏右, 负值=偏左
 * count_out: 输出检测到的传感器数量
 * pattern_out: 输出8位二进制模式
 */
float Grayscale_CalcError(uint8_t *sensor_values,
                          uint8_t *count_out, uint8_t *pattern_out);

/* 串口0发送单个字符 */
void uart0_send_char(char ch);

/* 串口0发送字符串 */
void uart0_send_string(char* str);

#endif /* __CONTROL_H */
