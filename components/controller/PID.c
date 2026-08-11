//
// Created by salen on 2026/8/11.
#include "../PID.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->e1 = pid->e2 = pid->u = 0.0f;
}