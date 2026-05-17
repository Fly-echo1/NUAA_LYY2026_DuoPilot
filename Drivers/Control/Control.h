#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

/* PID 控制器实例 */
extern PID_t pid_turn;
extern PID_t pid_speed_L;
extern PID_t pid_speed_R;

/* ===== Follower State Machine ===== */
typedef enum {
    FOLLOWER_FOLLOWING = 0,
    FOLLOWER_OVERTAKING
} FollowerState_t;

/* ===== K230 AprilTag Data ===== */
typedef struct {
    int16_t distance_cm;
    int16_t angle_deg;
} K230_Data_t;

/* ===== Overtaking Phases ===== */
typedef enum {
    OV_PHASE_SWERVE_LEFT = 0,
    OV_PHASE_PASS,
    OV_PHASE_SWERVE_RIGHT,
    OV_PHASE_DONE
} OvertakingPhase_t;

/* Global state */
extern FollowerState_t   g_follower_state;
extern OvertakingPhase_t g_ov_phase;
extern uint32_t          g_ov_tick_start;
extern K230_Data_t       g_k230_data;

/* ===== Function Declarations ===== */

/* Grayscale deviation (existing) */
float Grayscale_CalcError(uint8_t *sensor_values,
                          uint8_t *count_out, uint8_t *pattern_out);

/* UART debug (existing) */
void uart0_send_char(char ch);
void uart0_send_string(char* str);

/* K230 line parser */
void k230_parse_line(uint8_t *line);

/* Distance PID */
void distance_pid_init(float Kp, float Ki, float Kd,
                       float integral_limit, float output_limit);
float distance_pid_calculate(int16_t current_dist, int16_t target_dist);

/* Overtaking control */
void overtaking_start(void);
void overtaking_update(uint32_t tick);
bool overtaking_is_done(void);

/* Buzzer */
void buzzer_on(void);
void buzzer_off(void);

#endif /* __CONTROL_H */
