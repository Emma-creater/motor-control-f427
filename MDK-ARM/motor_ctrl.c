#include "motor_ctrl.h"
#include "pid.h"
#include "can_motor.h"

/* ===== 全局共享变量的"定义"（声明在app_config.h，实体在这里） ===== */
MotorFeed_t g_motor;        /* 电机反馈：CAN中断写，控制任务读 */
RcInput_t   g_rc;           /* 遥控器输入：SBUS任务写，控制任务读 */
CtrlRef_t   g_ref;          /* 控制目标/输出：本文件写 */

/* ===== JScope 观测变量 ===== */
#if JSCOPE_ENABLE
volatile float js_target_angle = 0.0f;   /* 目标角度 */
volatile float js_actual_angle = 0.0f;   /* 实际角度 */
volatile float js_target_speed = 0.0f;   /* 目标转速 */
volatile float js_actual_speed = 0.0f;   /* 实际转速 */
volatile float js_pid_out      = 0.0f;   /* PID输出 */
#endif
/* volatile：防止编译器把"没人读的变量"优化掉，JScope就找不到它们 */

static Pid_t s_pos_pid;     /* 角度环（外环） */
static Pid_t s_spd_pid;     /* 速度环（内环） */

#define RC_DEADZONE  0.05f  /* 摇杆死区：±0.05内视为0，消中位蠕动 */

/* ===== 死区处理 ===== */
static float ApplyDeadzone(float x)
{
    if (x > -RC_DEADZONE &amp;&amp; x < RC_DEADZONE) return 0.0f;  /* 死区内→0 */
    return x;                                              /* 死区外→原值 */
}

/* ===== 初始化（上电一次） ===== */
void MotorCtrl_Init(void)
{
    Pid_Init(&amp;s_spd_pid, SPD_PID_KP, SPD_PID_KI, SPD_PID_KD,
             SPD_I_LIMIT, SPD_OUT_LIMIT);     /* 装速度环参数 */
    Pid_Init(&amp;s_pos_pid, POS_PID_KP, POS_PID_KI, POS_PID_KD,
             POS_I_LIMIT, POS_OUT_LIMIT);     /* 装角度环参数 */

    g_motor.online = 0;                /* 初始无反馈 */
    g_ref.target_angle_deg = 0.0f;     /* 目标清零 */
    g_ref.target_speed_dps = 0.0f;
    g_ref.pid_out = 0.0f;

    CanMotor_Init();                   /* 初始化CAN */
}

/* ===== 控制任务（每2ms，系统"大脑"） ===== */
void MotorCtrl_Task(void)
{
    float lv, rv;                      /* 左右摇杆归一化值 */
    float current = 0.0f;              /* 本轮输出电流 */

    /* ---- 安全闸：任一不安全 → 停车 ---- */
    if (g_rc.mode == MOTOR_MODE_SAFE || !g_rc.online || !g_motor.online) {
        CanMotor_SendCurrent(0);       /* 主动发0电流 */
        Pid_Reset(&amp;s_pos_pid);         /* 清PID，防旧积分突放 */
        Pid_Reset(&amp;s_spd_pid);
        g_ref.target_angle_deg = 0.0f; /* 目标清零 */
        g_ref.target_speed_dps = 0.0f;
        g_ref.pid_out = 0.0f;
        #if JSCOPE_ENABLE
        js_target_angle = 0.0f;
        js_actual_angle = g_motor.angle_deg;  /* 实际值保留，便于看当前位置 */
        js_target_speed = 0.0f;
        js_actual_speed = g_motor.speed_dps;
        js_pid_out      = 0.0f;
        #endif
        return;                        /* 结束本轮 */
    }

    if (g_rc.mode == MOTOR_MODE_POS) {
        /* ===== 位置模式：串级双环 ===== */
        lv = ApplyDeadzone(g_rc.lv_norm);       /* 左摇杆，过死区 */
        g_ref.target_angle_deg = lv * 90.0f;    /* -1~+1 → -90~+90° */

        /* 外环：角度误差 → 目标角速度 */
        g_ref.target_speed_dps = Pid_Calc(&amp;s_pos_pid,
                                          g_ref.target_angle_deg,
                                          g_motor.angle_deg);
        /* 内环：角速度误差 → 电流 */
        current = Pid_Calc(&amp;s_spd_pid,
                           g_ref.target_speed_dps,
                           g_motor.speed_dps);
        /* ★外环输出直接作内环输入 = "串级" */

    } else if (g_rc.mode == MOTOR_MODE_SPD) {
        /* ===== 速度模式：单环 ===== */
        rv = ApplyDeadzone(g_rc.rv_norm);       /* 右摇杆，过死区 */
        g_ref.target_speed_dps = rv * SPEED_MODE_MAX_DPS;  /* → 目标角速度 */

        Pid_Reset(&amp;s_pos_pid);                  /* 位置环不参与，清掉 */
        current = Pid_Calc(&amp;s_spd_pid,          /* 只用速度环 */
                           g_ref.target_speed_dps,
                           g_motor.speed_dps);
    }

    g_ref.pid_out = current;                    /* 记录输出 */
    CanMotor_SendCurrent((int16_t)current);     /* 转整数发给C620 */

    /* ===== 刷新 JScope ===== */
    #if JSCOPE_ENABLE
    js_target_angle = g_ref.target_angle_deg;
    js_actual_angle = g_motor.angle_deg;
    js_target_speed = g_ref.target_speed_dps;
    js_actual_speed = g_motor.speed_dps;
    js_pid_out      = g_ref.pid_out;
    #endif
}
