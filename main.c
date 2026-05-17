/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "Grayscale.h"
#include "Control.h"

#define CLAMP(x, min, max)  ((x) < (min) ? (min) : (x) > (max) ? (max) : (x))

uint8_t oled_buffer[64];
uint8_t Grayscale_buffer[8];
uint8_t sensor_values[GRAYSCALE_CHANNELS]; // 定义数组存储8路数据
char grayscale_str[GRAYSCALE_CHANNELS + 1]; //用于OLED显示的字符串

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    OLED_Init();
    WIT_Init();
    Motor_Init();
    Encoder_Init();

    /* Don't remove this! */
    enable_group1_irq = 1;              /* 启用 GROUP1 中断 (编码器 GPIO 中断) */
    Interrupt_Init();
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);  /* 使能 GPIOA 中断 (编码器A+B) */

    /* 开启编码器 50ms 速度采样定时器 */
    NVIC_EnableIRQ(TIMER_Encoder_Read_INST_INT_IRQN);

    /* 开启 K230 UART RX 中断 */
    NVIC_EnableIRQ(UART_VISION_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(UART_VISION_INST, DL_UART_MAIN_INTERRUPT_RX);

    /* ---- PID 参数初始化 ---- */
    /* 转向环 (灰度偏差→差速): Kp 控制转向力度 */
    PID_Init(&pid_turn, 0.8f, 0.005f, 0.3f, 30.0f, 60.0f);

    /* 速度环 (目标速度→PWM): 每个电机独立 */
    PID_Init(&pid_speed_L, 0.5f, 0.01f, 0.1f, 20.0f, 50.0f);
    PID_Init(&pid_speed_R, 0.5f, 0.01f, 0.1f, 20.0f, 50.0f);

    /* 蜂鸣器响一声表示系统启动 */
    DL_GPIO_setPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);
    delay_cycles(3200000);
    DL_GPIO_clearPins(GPIO_BEEP_PORT, GPIO_BEEP_PIN_BEEP_PIN);

    /* OLED 显示标签 */
    OLED_ShowString(0, 0, (uint8_t *)"Car.101 PID", 16);
    mspm0_delay_ms(500);

    OLED_ShowString(0, 0, (uint8_t *)"Dev:", 8);
    OLED_ShowString(0, 2, (uint8_t *)"L:", 8);
    OLED_ShowString(7*6, 2, (uint8_t *)"R:", 8);
    OLED_ShowString(0, 4, (uint8_t *)"P:", 8);
    OLED_ShowString(7*6, 4, (uint8_t *)"I:", 8);
    OLED_ShowString(14*6, 4, (uint8_t *)"D:", 8);
    OLED_ShowString(0, 6, (uint8_t *)"SPD:", 8);

    /* 控制参数 */
    const int16_t base_speed = 40;      // 基础速度 0~100
    float last_deviation = 0.0f;         // 上一帧偏差 (丢线保持)
    uint8_t no_line_count = 0;           // 连续丢线计数
    float motor_output_L = 0;            // 左轮最终PWM值(用于显示)
    float motor_output_R = 0;            // 右轮最终PWM值

    while (1)
    {
        /* ========== 1. 读取8路灰度传感器 ========== */
        Grayscale_Sensor_Read_All(sensor_values);

        /* ========== 2. 计算灰度偏差 ========== */
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
        else if (deviation >= 900.0f)
        {
            /* 丢线: 保持上一帧偏差, 计数 */
            deviation = last_deviation;
            no_line_count++;
        }
        else
        {
            /* 正常检测到线 */
            no_line_count = 0;
        }

        /* ========== 3. 转向环 PID ========== */
        float turn_output = PID_Calculate(&pid_turn, deviation);
        last_deviation = deviation;

        /* ========== 4. 计算目标速度 ========== */
        int16_t target_speed_L, target_speed_R;

        if (no_line_count > 10)
        {
            /* 连续丢线 ~200ms: 停车 */
            target_speed_L = 0;
            target_speed_R = 0;
        }
        else
        {
            /* 差速转向: 转向环输出直接修正左右轮目标速度 */
            target_speed_L = base_speed + (int16_t)turn_output;
            target_speed_R = base_speed - (int16_t)turn_output;

            /* 限幅 */
            target_speed_L = CLAMP(target_speed_L, 0, 100);
            target_speed_R = CLAMP(target_speed_R, 0, 100);
        }

        /* ========== 5. 速度环 PID (编码器反馈) ========== */
        /* 注: 编码器速度 encoder_left_speed 每50ms更新一次, */
        /*     20ms控制周期下同一数值会持续2-3个周期 */
        float speed_out_L = PID_Calculate(&pid_speed_L,
                              (float)(target_speed_L - encoder_left_speed));
        float speed_out_R = PID_Calculate(&pid_speed_R,
                              (float)(target_speed_R - encoder_right_speed));

        /* 速度环输出作为最终PWM值 */
        motor_output_L = (float)target_speed_L + speed_out_L;
        motor_output_R = (float)target_speed_R + speed_out_R;

        /* 限幅 [0, 100] */
        motor_output_L = CLAMP(motor_output_L, 0.0f, 100.0f);
        motor_output_R = CLAMP(motor_output_R, 0.0f, 100.0f);

        Motor_SetSpeed((int16_t)motor_output_L, (int16_t)motor_output_R);

        /* ========== 6. OLED 显示 ========== */
        /* 偏差 */
        sprintf((char *)oled_buffer, "%+4d", (int16_t)deviation);
        OLED_ShowString(5*6, 0, oled_buffer, 8);

        /* PWM输出 */
        sprintf((char *)oled_buffer, "%-3d", (int16_t)motor_output_L);
        OLED_ShowString(3*6, 2, oled_buffer, 8);
        sprintf((char *)oled_buffer, "%-3d", (int16_t)motor_output_R);
        OLED_ShowString(9*6, 2, oled_buffer, 8);

        /* PID分量 (转向环的P,I,D用于显示) */
        sprintf((char *)oled_buffer, "%5.1f", pid_turn.Kp * deviation);
        OLED_ShowString(3*6, 4, oled_buffer, 8);
        sprintf((char *)oled_buffer, "%5.1f", pid_turn.integral);
        OLED_ShowString(9*6, 4, oled_buffer, 8);
        sprintf((char *)oled_buffer, "%5.1f", pid_turn.Kd * (deviation - pid_turn.last_error));
        OLED_ShowString(16*6, 4, oled_buffer, 8);

        /* 编码器速度 */
        sprintf((char *)oled_buffer, "%-3d", encoder_left_speed);
        OLED_ShowString(5*6, 6, oled_buffer, 8);

        /* 灰度二进制串（底部）和传感器计数 */
        for (int i = 0; i < GRAYSCALE_CHANNELS; i++)
            grayscale_str[i] = sensor_values[i] ? '1' : '0';
        grayscale_str[GRAYSCALE_CHANNELS] = '\0';
        OLED_ShowString(0, 7, (uint8_t *)grayscale_str, 8);

        /* ========== 7. 蓝牙调试输出 ========== */
        sprintf((char *)oled_buffer,
                "dev:%+4d L:%3d R:%3d cnt:%d SL:%3d SR:%3d\r\n",
                (int16_t)deviation, (int16_t)motor_output_L, (int16_t)motor_output_R,
                sensor_count, encoder_left_speed, encoder_right_speed);
        uart0_send_string((char *)oled_buffer);

        /* ========== 8. 控制周期 ≈ 50Hz ========== */
        mspm0_delay_ms(20);
    }

}

