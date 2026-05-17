#include "ti_msp_dl_config.h"
#include "Grayscale.h"
#include "Control.h"

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
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_send_char(*str++);
    }
}
