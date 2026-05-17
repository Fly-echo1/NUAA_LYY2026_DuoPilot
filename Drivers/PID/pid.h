#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"

/* PID 结构体：位置式 PID */
typedef struct {
    float Kp;              // 比例系数
    float Ki;              // 积分系数
    float Kd;              // 微分系数

    float integral;        // 积分累计值
    float last_error;      // 上一次误差（用于微分项）

    float integral_limit;  // 积分限幅 (防积分饱和)
    float output_limit;    // 输出限幅
} PID_t;

/* 初始化 PID 参数 */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float integral_limit, float output_limit);

/* 重置 PID (清零积分和历史误差) */
void PID_Reset(PID_t *pid);

/* 计算 PID 输出: 输入当前误差，返回控制量 */
float PID_Calculate(PID_t *pid, float error);

/* 更新 PID 参数 (运行时调参) */
void PID_SetParam(PID_t *pid, float Kp, float Ki, float Kd);

#endif /* __PID_H */
