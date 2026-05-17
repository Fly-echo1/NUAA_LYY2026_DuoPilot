#include "encoder.h"

volatile int32_t encoder_left_pulse  = 0;
volatile int32_t encoder_right_pulse = 0;

volatile int16_t encoder_left_speed  = 0;
volatile int16_t encoder_right_speed = 0;

void Encoder_Init(void)
{
    encoder_left_pulse  = 0;
    encoder_right_pulse = 0;
    encoder_left_speed  = 0;
    encoder_right_speed = 0;
}

int32_t Encoder_GetPulseLeft(void)
{
    return encoder_left_pulse;
}

int32_t Encoder_GetPulseRight(void)
{
    return encoder_right_pulse;
}

int16_t Encoder_GetSpeedLeft(void)
{
    return encoder_left_speed;
}

int16_t Encoder_GetSpeedRight(void)
{
    return encoder_right_speed;
}

/* TIMG6_IRQHandler 定义在 interrupt.c 中 */
