#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Motor_SetSpeed(int16_t left, int16_t right);
void Motor_Stop(void);

#endif /* __MOTOR_H */
