#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"

/* 脉冲计数器 (在中断中自增, 在主循环中读取/清零) */
extern volatile int32_t encoder_left_pulse;
extern volatile int32_t encoder_right_pulse;

/* 速度值 (每 50ms 更新一次, 单位: 脉冲数/50ms) */
extern volatile int16_t encoder_left_speed;
extern volatile int16_t encoder_right_speed;

void Encoder_Init(void);
int32_t Encoder_GetPulseLeft(void);
int32_t Encoder_GetPulseRight(void);
int16_t Encoder_GetSpeedLeft(void);
int16_t Encoder_GetSpeedRight(void);

#endif /* __ENCODER_H */
