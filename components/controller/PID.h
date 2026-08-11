//
// Created by salen on 2026/8/11.
//

#ifndef PID_H
#define PID_H

typedef struct
{
    float kp, ki, kd;
    float e1, e2;
    float u;
    float max_output;
}   pid_t;

void  pid_init(pid_t *pid, float kp, float ki, float kd, float max_output);
float pid_calc(pid_t *pid, float tagt, float fedb);
void  pid_reset(pid_t *pid);

#endif
