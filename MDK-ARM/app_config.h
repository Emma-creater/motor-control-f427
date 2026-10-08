#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define CTRL_PERIOD_MS        2U
#define SBUS_PERIOD_MS        4U

#define MOTOR_ID              1U
#define C620_TX_ID            (0x200U)
#define C620_RX_ID_BASE       (0x200U)
#define C620_CURRENT_MAX      16384.0f
#define MOTOR_GEAR_RATIO      19.2032f

#define SBUS_FRAME_SIZE       25U
#define SBUS_HEADER           0x0FU
#define SBUS_FOOTER           0x00U
#define SBUS_FLAG_CH17        0x01U
#define SBUS_FLAG_CH18        0x02U
#define SBUS_FLAG_FRAME_LOST  0x04U
#define SBUS_FLAG_FAILSAFE    0x08U
#define SBUS_RC_DEADLINE      100U
#define SBUS_CH_MIN           172
#define SBUS_CH_MAX           1811
#define SBUS_CH_MID           992.0f
#define SBUS_CH_HALF_RANGE    820.0f

enum {
    RC_CH_RH = 0,
    RC_CH_RV,
    RC_CH_LV,
    RC_CH_LH,
    RC_CH_SWA5,
    RC_CH_SWB,
    RC_CH_SWC,
    RC_CH_SWD,
    RC_CH_SA,
    RC_CH_SB,
    RC_CH_SC,
    RC_CH_SD,
    RC_CH_SE,
    RC_CH_SF,
    RC_CH_SG,
    RC_CH_SH,
    RC_CH_MAX
};

#define SW_HIGH_TH            1250
#define SW_LOW_TH             700

typedef enum {
    MOTOR_MODE_SAFE = 0,
    MOTOR_MODE_POS,
    MOTOR_MODE_SPD
} MotorMode_e;

typedef struct {
    uint16_t ecd;
    int16_t  speed_rpm;
    int16_t  current_raw;
    uint8_t  temperature;
    uint16_t last_ecd;
    int32_t  total_round;
    float    angle_deg;
    float    speed_dps;
    uint32_t last_update_tick;
    uint8_t  online;
} MotorFeed_t;

typedef struct {
    int16_t  ch[RC_CH_MAX];
    float    lv_norm;
    float    rv_norm;
    MotorMode_e mode;
    uint8_t  frame_lost;
    uint8_t  failsafe;
    uint32_t last_rx_tick;
    uint8_t  online;
} RcInput_t;

typedef struct {
    float target_angle_deg;
    float target_speed_dps;
    float pid_out;
} CtrlRef_t;

typedef struct {
    float kp, ki, kd;
    float i_limit;
    float out_limit;
} PidParam_t;

#define SPD_PID_KP   2.0f
#define SPD_PID_KI   0.0f
#define SPD_PID_KD   0.0f
#define SPD_I_LIMIT  2000.0f
#define SPD_OUT_LIMIT C620_CURRENT_MAX

#define POS_PID_KP   3.0f
#define POS_PID_KI   0.0f
#define POS_PID_KD   0.05f
#define POS_I_LIMIT  500.0f
#define POS_OUT_LIMIT 500.0f

#define SPEED_MODE_MAX_DPS   1446.0f

#define JSCOPE_ENABLE   1
#if JSCOPE_ENABLE
extern volatile float js_target_angle;
extern volatile float js_actual_angle;
extern volatile float js_target_speed;
extern volatile float js_actual_speed;
extern volatile float js_pid_out;
#endif

extern MotorFeed_t g_motor;
extern RcInput_t   g_rc;
extern CtrlRef_t   g_ref;

#endif