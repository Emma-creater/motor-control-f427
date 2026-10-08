#ifndef __APP_CONFIG_H          /* 防重复包含：第一次进来时这个宏没定义 */
#define __APP_CONFIG_H          /* 立刻定义它；第二次被包含时上面的 #ifndef 为假，整个文件跳过 */

#include "main.h"               /* 提供 HAL 相关基础类型，及 HAL_GetTick 等 */
#include "cmsis_os.h"           /* FreeRTOS 的 CMSIS 封装，提供 osDelay 等 */
#include <stdint.h>             /* 标准整型：uint8_t / int16_t / uint32_t */

/* ========== 1. 任务周期 ========== */
#define CTRL_PERIOD_MS        2U    /* 控制任务每 2ms 跑一次 = 500Hz */
#define SBUS_PERIOD_MS        4U    /* 遥控器解析每 4ms 跑一次（SBUS 一帧 14ms，够用） */

/* ========== 2. CAN 与电机参数 ========== */
#define MOTOR_ID              1U    /* 本工程的电机编号，必须等于 C620 拨码设的 ID */
#define C620_TX_ID            (0x200U) /* 发给电调的控制帧 ID（ID1~4 共用这一条） */
#define C620_RX_ID_BASE       (0x200U) /* 反馈帧 ID 基址：0x200 + ID */
#define C620_CURRENT_MAX      16384.0f /* C620 电流满量程 raw 值（对应 ±20A） */
#define MOTOR_GEAR_RATIO      19.2032f /* 3508 减速比：转子转 19.2 圈，输出轴转 1 圈 */

/* ========== 3. SBUS 协议常量 ========== */
#define SBUS_FRAME_SIZE       25U   /* SBUS 一帧固定 25 字节 */
#define SBUS_HEADER           0x0FU /* 帧头字节：用来在字节流里定位帧的起点 */
#define SBUS_FOOTER           0x00U /* 帧尾字节：配合帧头确认这是一帧完整的 */

/* 第 23 字节是标志位，4 个 bit 各有含义 */
#define SBUS_FLAG_CH17        0x01U /* bit0：通道17（开关）状态 */
#define SBUS_FLAG_CH18        0x02U /* bit1：通道18（开关）状态 */
#define SBUS_FLAG_FRAME_LOST  0x04U /* bit2：丢帧标志，接收机说"这帧是补的" */
#define SBUS_FLAG_FAILSAFE    0x08U /* bit3：失控保护，接收机说"我和遥控器断了" */

#define SBUS_RC_DEADLINE      100U  /* 超过 100ms 没收到新帧 → 判定遥控器失联 */

/* SBUS 通道原始值范围（11bit 编码，实际有效区间） */
#define SBUS_CH_MIN           172   /* 摇杆打到一边的最小值 */
#define SBUS_CH_MAX           1811  /* 打到另一边的最大值 */
#define SBUS_CH_MID           992.0f /* 中位（不是 1024，注意） */
#define SBUS_CH_HALF_RANGE    820.0f /* 半量程：用于把原始值归一化到 ±1 */

/* 通道索引表：SBUS 的通道顺序是固定的，用名字代替"第几路"，代码更好读 */
enum {
    RC_CH_RH = 0,   /* 通道0：右摇杆 水平 */
    RC_CH_RV,       /* 通道1：右摇杆 垂直  ← 速度模式用这个 */
    RC_CH_LV,       /* 通道2：左摇杆 垂直  ← 位置模式用这个 */
    RC_CH_LH,       /* 通道3：左摇杆 水平 */
    RC_CH_SWA5,     /* 通道4：三位开关 SWA5 ← 模式切换用这个（★需实机核对编号★） */
    RC_CH_SWB,      /* 通道5：开关 B */
    RC_CH_SWC,      /* 通道6 */
    RC_CH_SWD,      /* 通道7 */
    RC_CH_SA,       /* 通道8 */
    RC_CH_SB,       /* 通道9 */
    RC_CH_SC,       /* 通道10 */
    RC_CH_SD,       /* 通道11 */
    RC_CH_SE,       /* 通道12 */
    RC_CH_SF,       /* 通道13 */
    RC_CH_SG,       /* 通道14 */
    RC_CH_SH,       /* 通道15 */
    RC_CH_MAX       /* =16，数组长度用，也方便写循环 */
};

/* 三位开关位置判定阈值（介于中位 992 与两个端点之间） */
#define SW_HIGH_TH            1250  /* 大于此值 → 判为"最上方" */
#define SW_LOW_TH             700   /* 小于此值 → 判为"最下方" */

/* ========== 4. 控制模式枚举 ========== */
typedef enum {
    MOTOR_MODE_SAFE = 0,    /* 安全模式：失联/开关居中，输出 0，电机不动 */
    MOTOR_MODE_POS,         /* 位置模式：SWA5 最上，用左摇杆，串级双环 */
    MOTOR_MODE_SPD          /* 速度模式：SWA5 最下，用右摇杆，速度单环 */
} MotorMode_e;

/* ========== 5. 电机反馈结构 ========== */
typedef struct {
    uint16_t ecd;           /* 编码器原始值 0~8191（转子单圈） */
    int16_t  speed_rpm;     /* 转子转速，单位 rpm（带正负号，正负代表方向） */
    int16_t  current_raw;   /* 实际电流 raw 值 */
    uint8_t  temperature;   /* 电机温度（摄氏度） */
    uint16_t last_ecd;      /* 上一次的编码器值（备用） */
    int32_t  total_round;   /* 累计位置（备用） */
    float    angle_deg;     /* 输出轴累计角度，单位度 ← 位置环的反馈量 */
    float    speed_dps;     /* 输出轴角速度，单位度/秒 ← 速度环的反馈量 */
    uint32_t last_update_tick; /* 最后一次收到反馈的时间，可用来判断电机是否掉线 */
    uint8_t  online;        /* 1=在收反馈，0=没收到 */
} MotorFeed_t;

/* ========== 6. 遥控器输入结构 ========== */
typedef struct {
    int16_t  ch[RC_CH_MAX]; /* 16 个通道的原始值（172~1811） */
    float    lv_norm;       /* 左摇杆垂直，归一化到 -1~+1 */
    float    rv_norm;       /* 右摇杆垂直，归一化到 -1~+1 */
    MotorMode_e mode;       /* 由 SWA5 解算出的当前控制模式 */
    uint8_t  frame_lost;    /* SBUS 丢帧标志（透传给上层，暂未使用） */
    uint8_t  failsafe;      /* SBUS 失控标志，置位则强制安全模式 */
    uint32_t last_rx_tick;  /* 最后一次收到帧的时间，失联检测用 */
    uint8_t  online;        /* 1=遥控器在线 */
} RcInput_t;

/* ========== 7. 控制目标与输出 ========== */
typedef struct {
    float target_angle_deg; /* 位置模式的目标角（度） */
    float target_speed_dps; /* 速度模式的目标角速度（度/秒） */
    float pid_out;          /* PID 最终输出（电流 raw），JScope 要看 */
} CtrlRef_t;

/* ========== 8. PID 参数结构 ========== */
typedef struct {
    float kp, ki, kd;       /* 比例/积分/微分系数 */
    float i_limit;          /* 积分项限幅，抗积分饱和 */
    float out_limit;        /* 输出限幅，防止算出的电流超范围 */
} PidParam_t;

/* 速度环参数（初值，必须实机调） */
#define SPD_PID_KP   8.0f
#define SPD_PID_KI   0.3f
#define SPD_PID_KD   0.0f
#define SPD_I_LIMIT  2000.0f
#define SPD_OUT_LIMIT C620_CURRENT_MAX   /* 速度环输出就是电流，限到满量程 */

/* 角度环参数（它的输出是"目标角速度"，单位 dps） */
#define POS_PID_KP   6.0f
#define POS_PID_KI   0.0f
#define POS_PID_KD   0.2f
#define POS_I_LIMIT  500.0f
#define POS_OUT_LIMIT 3000.0f   /* 限外环输出的最大角速度，防冲过头 */

/* 速度模式下摇杆打满对应的角速度（考核要求：额定转速的一半） */
#define SPEED_MODE_MAX_DPS   7500.0f   /* ★需按实际电机参数核算★ */

/* ========== 9. JScope 观测变量开关 ========== */
#define JSCOPE_ENABLE   1       /* 置 1 打开；这些变量给 JScope 画波形用 */

#if JSCOPE_ENABLE
/* volatile 告诉编译器"别优化我"，否则可能被优化掉导致 JScope 找不到 */
extern volatile float js_target_angle;  /* 目标角度 */
extern volatile float js_actual_angle;  /* 实际角度 */
extern volatile float js_target_speed;  /* 目标转速 */
extern volatile float js_actual_speed;  /* 实际转速 */
extern volatile float js_pid_out;       /* PID 输出 */
#endif

/* ========== 10. 全局共享数据的"声明" ========== */
/* 注意：这里只是 extern 声明，说"这些变量在别处有"。
   真正的定义（分配内存）在 motor_ctrl.c 里。
   这样所有 .c 包含本头文件后都能用它们，且只有一份实体。 */
extern MotorFeed_t g_motor;     /* 电机反馈 */
extern RcInput_t   g_rc;        /* 遥控器输入 */
extern CtrlRef_t   g_ref;       /* 控制目标/输出 */

#endif                          /* __APP_CONFIG_H 结束 */

