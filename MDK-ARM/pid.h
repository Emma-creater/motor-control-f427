#ifndef __PID_H                 /* 防重复包含 */
#define __PID_H

#include "app_config.h"

/* PID 控制器：分"参数"和"运行状态"两部分 */
typedef struct {
    PidParam_t param;   /* 参数：kp/ki/kd/限幅（初始化时设定，运行中不变） */
    float set;          /* 本次目标值（记录用） */
    float fdb;          /* 本次反馈值（记录用） */
    float err;          /* 本次误差 = set - fdb */
    float last_err;     /* 上次误差（算微分用） */
    float integral;     /* 积分累加器（带限幅） */
    float output;       /* 本次输出（记录用） */
} Pid_t;

/* 初始化：装参数 + 清状态。同一个 PID 只调一次 */
void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit);

/* 计算一次：传目标与反馈，返回输出。每控制周期调一次 */
float Pid_Calc(Pid_t *pid, float set, float fdb);

/* 清空积分和历史误差。切模式时调用，防旧状态污染新模式 */
void Pid_Reset(Pid_t *pid);

#endif
