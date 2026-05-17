#include "pid.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float integral_limit, float output_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

void PID_SetParam(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

float PID_Calculate(PID_t *pid, float error)
{
    /* P: 比例项 */
    float P = pid->Kp * error;

    /* I: 积分项 (带限幅) */
    pid->integral += pid->Ki * error;
    if (pid->integral >  pid->integral_limit) pid->integral =  pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    /* D: 微分项 */
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    /* 总输出 (带限幅) */
    float output = P + pid->integral + D;
    if (output >  pid->output_limit) output =  pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
