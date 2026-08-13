//
// Created by salen on 2026/8/11.
//
#include "PID.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->e1 = pid->e2 = pid->u = 0.0f;
}

float pid_calc(pid_t *pid, float ref, float fdb)
{
    float err = ref - fdb;
    pid->u += pid->kp * (err - pid->e1)
           +  pid->ki * err
           +  pid->kd * (err - 2.0f * pid->e1 + pid->e2);   // 增量式
    pid->e2 = pid->e1;
    pid->e1 = err;

    /* 输出限幅 */
    if (pid->u >  pid->max_output) pid->u =  pid->max_output;
    if (pid->u < -pid->max_output) pid->u = -pid->max_output;
    return pid->u;
}

void pid_reset(pid_t *pid)
{
    pid->e1 = pid->e2 = pid->u = 0.0f;   // 失联复位：清状态，参数不动
}
