#include "motor.h"

/*
 * 电机方向控制 (H桥):
 *   IN1=1, IN2=0 → 正转 (前进)
 *   IN1=0, IN2=1 → 反转 (后退)
 *   IN1=0, IN2=0 → 停止 (滑行)
 *   IN1=1, IN2=1 → 刹车
 */

/**
 * @brief 电机初始化函数
 * @details 此函数用于初始化电机的控制引脚和PWM信号，确保电机在初始状态下处于停止状态
 */
void Motor_Init(void)
{
    /* 方向引脚初始化为输出，默认停止 (LOW) */
    /* 清除左侧电机和右侧电机的方向控制引脚 */
    DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN |
                                               GPIO_MOTOR_LEFT_2_PIN |
                                               GPIO_MOTOR_RIGHT_2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);

    /* PWM 初始占空比为 0 */
    /* 设置左侧电机PWM的初始捕获比较值为0 */
    DL_TimerA_setCaptureCompareValue(PWM_L_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_R_INST, 0, DL_TIMER_CC_0_INDEX);
}

/*
 * left/right: -100 ~ 100
 *   正数 → 前进
 *   负数 → 后退
 *   0    → 停止
 */
void Motor_SetSpeed(int16_t left, int16_t right)
{
    /* === 左轮 (LEFT_1/2 都在 PORTA) === */
    if (left > 0)
    {
        DL_GPIO_setPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
    }
    else if (left < 0)
    {
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN |
                                                   GPIO_MOTOR_LEFT_2_PIN);
    }

    /* === 右轮 (RIGHT_1=PORTB, RIGHT_2=PORTA) === */
    if (right > 0)
    {
        DL_GPIO_setPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
    }
    else if (right < 0)
    {
        DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
    }

    /* 限幅并设置 PWM 占空比 (0~100) */
    if (left < 0) left = -left;
    if (left > 100) left = 100;
    DL_TimerA_setCaptureCompareValue(PWM_L_INST, (uint16_t)left, DL_TIMER_CC_0_INDEX);

    if (right < 0) right = -right;
    if (right > 100) right = 100;
    DL_TimerA_setCaptureCompareValue(PWM_R_INST, (uint16_t)right, DL_TIMER_CC_0_INDEX);
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}
