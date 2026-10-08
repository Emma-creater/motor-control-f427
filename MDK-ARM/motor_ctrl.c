#include "motor_ctrl.h"
#include "pid.h"
#include "can_motor.h"

MotorFeed_t g_motor;
RcInput_t   g_rc;
CtrlRef_t   g_ref;

#if JSCOPE_ENABLE
volatile float js_target_angle = 0.0f;
volatile float js_actual_angle = 0.0f;
volatile float js_target_speed = 0.0f;
volatile float js_actual_speed = 0.0f;
volatile float js_pid_out      = 0.0f;
#endif

static Pid_t s_pos_pid;
static Pid_t s_spd_pid;

#define RC_DEADZONE  0.05f

static float ApplyDeadzone(float x)
{
    if (x > -RC_DEADZONE && x < RC_DEADZONE) return 0.0f;
    return x;
}

void MotorCtrl_Init(void)
{
    Pid_Init(& s_spd_pid, SPD_PID_KP, SPD_PID_KI, SPD_PID_KD,
             SPD_I_LIMIT, SPD_OUT_LIMIT);
    Pid_Init(& s_pos_pid, POS_PID_KP, POS_PID_KI, POS_PID_KD,
             POS_I_LIMIT, POS_OUT_LIMIT);

    g_motor.online = 0;
    g_ref.target_angle_deg = 0.0f;
    g_ref.target_speed_dps = 0.0f;
    g_ref.pid_out = 0.0f;

    CanMotor_Init();
}

void MotorCtrl_Task(void)
{
    float lv, rv;
    float current = 0.0f;

    if (g_rc.mode == MOTOR_MODE_SAFE || !g_rc.online || !g_motor.online) {
        CanMotor_SendCurrent(0);
        Pid_Reset(& s_pos_pid);
        Pid_Reset(& s_spd_pid);
        g_ref.target_angle_deg = 0.0f;
        g_ref.target_speed_dps = 0.0f;
        g_ref.pid_out = 0.0f;
        #if JSCOPE_ENABLE
        js_target_angle = 0.0f;
        js_actual_angle = g_motor.angle_deg;
        js_target_speed = 0.0f;
        js_actual_speed = g_motor.speed_dps;
        js_pid_out      = 0.0f;
        #endif
        return;
    }

    if (g_rc.mode == MOTOR_MODE_POS) {
        lv = ApplyDeadzone(g_rc.lv_norm);
        g_ref.target_angle_deg = lv * 90.0f;

        g_ref.target_speed_dps = Pid_Calc(& s_pos_pid,
                                          g_ref.target_angle_deg,
                                          g_motor.angle_deg);
        current = Pid_Calc(& s_spd_pid,
                           g_ref.target_speed_dps,
                           g_motor.speed_dps);

    } else if (g_rc.mode == MOTOR_MODE_SPD) {
        rv = ApplyDeadzone(g_rc.rv_norm);
        g_ref.target_speed_dps = rv * SPEED_MODE_MAX_DPS;

        Pid_Reset(& s_pos_pid);
        current = Pid_Calc(& s_spd_pid,
                           g_ref.target_speed_dps,
                           g_motor.speed_dps);
    }

    g_ref.pid_out = current;
    CanMotor_SendCurrent((int16_t)current);

    #if JSCOPE_ENABLE
    js_target_angle = g_ref.target_angle_deg;
    js_actual_angle = g_motor.angle_deg;
    js_target_speed = g_ref.target_speed_dps;
    js_actual_speed = g_motor.speed_dps;
    js_pid_out      = g_ref.pid_out;
    #endif
}