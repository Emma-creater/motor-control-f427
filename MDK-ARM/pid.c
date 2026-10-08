#include "pid.h"

void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit)
{
    pid->param.kp = kp;
    pid->param.ki = ki;
    pid->param.kd = kd;
    pid->param.i_limit = i_limit;
    pid->param.out_limit = out_limit;
    Pid_Reset(pid);
}

void Pid_Reset(Pid_t *pid)
{
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float Pid_Calc(Pid_t *pid, float set, float fdb)
{
    float p_out, i_out, d_out, out;

    pid->set = set;
    pid->fdb = fdb;
    pid->err = set - fdb;

    p_out = pid->param.kp * pid->err;

    pid->integral += pid->err;
    if (pid->integral > pid->param.i_limit) {
        pid->integral = pid->param.i_limit;
    } else if (pid->integral < -pid->param.i_limit) {
        pid->integral = -pid->param.i_limit;
    }
    i_out = pid->param.ki * pid->integral;

    d_out = pid->param.kd * (pid->err - pid->last_err);
    pid->last_err = pid->err;

    out = p_out + i_out + d_out;
    if (out > pid->param.out_limit) {
        out = pid->param.out_limit;
    } else if (out < -pid->param.out_limit) {
        out = -pid->param.out_limit;
    }

    pid->output = out;
    return out;
}