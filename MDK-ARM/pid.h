#ifndef __PID_H
#define __PID_H

#include "app_config.h"

typedef struct {
    PidParam_t param;
    float set;
    float fdb;
    float err;
    float last_err;
    float integral;
    float output;
} Pid_t;

void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit);
float Pid_Calc(Pid_t *pid, float set, float fdb);
void Pid_Reset(Pid_t *pid);

#endif