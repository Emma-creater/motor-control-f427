#include "pid.h"                    /* 拿到 Pid_t 定义 */

/* ===== 初始化：装参数、清状态 ===== */
void Pid_Init(Pid_t *pid, float kp, float ki, float kd,
              float i_limit, float out_limit)
{
    pid->param.kp = kp;             /* 存比例系数 */
    pid->param.ki = ki;             /* 存积分系数 */
    pid->param.kd = kd;             /* 存微分系数 */
    pid->param.i_limit = i_limit;   /* 存积分限幅（抗饱和） */
    pid->param.out_limit = out_limit; /* 存输出限幅 */
    Pid_Reset(pid);                 /* 顺手清状态，防残留 */
}

/* ===== 清零：切模式时调用 ===== */
void Pid_Reset(Pid_t *pid)
{
    pid->set = 0.0f;        /* 目标清零 */
    pid->fdb = 0.0f;        /* 反馈清零 */
    pid->err = 0.0f;        /* 当前误差清零 */
    pid->last_err = 0.0f;   /* 上次误差清零（否则下次微分用到旧值） */
    pid->integral = 0.0f;   /* ★积分清零最关键：防止旧积分瞬间释放成冲击电流 */
    pid->output = 0.0f;     /* 输出清零 */
}

/* ===== 计算一次：PID 的核心 ===== */
float Pid_Calc(Pid_t *pid, float set, float fdb)
{
    float p_out, i_out, d_out, out;   /* 三个分量 + 最终输出 */

    pid->set = set;                   /* 记录目标 */
    pid->fdb = fdb;                   /* 记录反馈 */
    pid->err = set - fdb;             /* ★误差定义：目标-反馈，符号约定在此确立 */

    /* --- 比例项：误差越大纠正越猛 --- */
    p_out = pid->param.kp * pid->err;

    /* --- 积分项：累加消除稳态误差 --- */
    pid->integral += pid->err;        /* 误差累加 */
    if (pid->integral > pid->param.i_limit) {          /* 上限幅 */
        pid->integral = pid->param.i_limit;            /* 防积分饱和（堵转时不无限涨） */
    } else if (pid->integral < -pid->param.i_limit) {  /* 下限幅 */
        pid->integral = -pid->param.i_limit;
    }
    i_out = pid->param.ki * pid->integral;             /* 累加值×Ki */

    /* --- 微分项：误差变化率，提供阻尼 --- */
    d_out = pid->param.kd * (pid->err - pid->last_err); /* 本次误差-上次误差 */
    pid->last_err = pid->err;                           /* 更新"上次误差"，供下周期用 */

    /* --- 求和 + 总输出限幅 --- */
    out = p_out + i_out + d_out;            /* 三项叠加 */
    if (out > pid->param.out_limit) {        /* 上限 */
        out = pid->param.out_limit;          /* 保证不超电机能力 */
    } else if (out < -pid->param.out_limit) { /* 下限 */
        out = -pid->param.out_limit;
    }

    pid->output = out;                /* 记录输出 */
    return out;                       /* 返回给调用者（当电流用） */
}
