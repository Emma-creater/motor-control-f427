#include "sbus.h"              /* 本模块头文件（含 app_config.h） */
#include <string.h>            /* memset / memcpy */

/* ===== 模块私有变量（static = 只在本文件可见） ===== */
static uint8_t  s_rx_buf[SBUS_FRAME_SIZE * 2];  /* 接收缓冲，2帧容量防跨包 */
static uint16_t s_rx_len;                       /* 0=没帧，25=已存好一帧 */
static uint8_t  s_frame[SBUS_FRAME_SIZE];       /* 定位好后的整帧副本 */

/* ===== 位解码：22字节 → 16个11bit通道 ===== */
static void Sbus_DecodeChannels(const uint8_t *f, int16_t *ch)
{
    uint32_t bit_pos = 0;                  /* 位游标，从头开始 */

    for (int i = 0; i < 16; i++) {         /* 外层：第 i 个通道 */
        int32_t value = 0;

        for (int b = 0; b < 11; b++) {     /* 内层：连续取 11 个 bit */
            uint32_t byte_index = 1 + (bit_pos >> 3);  /* 位号→字节下标（+1跳过帧头） */
            uint32_t bit_index  = bit_pos &amp; 0x07;      /* 位号→字节内位号0~7 */
            uint8_t  bit = (f[byte_index] >> bit_index) &amp; 0x01;  /* 取出该位 */

            value |= ((int32_t)bit << b);  /* 放到目标值第b位（低位在前） */
            bit_pos++;                     /* 游标前进1位 */
        }
        ch[i] = (int16_t)value;            /* 一个通道解完，存起来 */
    }
}

/* ===== 初始化（上电一次） ===== */
void Sbus_Init(void)
{
    memset(s_rx_buf, 0, sizeof(s_rx_buf));  /* 清缓冲 */
    memset(s_frame, 0, sizeof(s_frame));    /* 清整帧 */
    s_rx_len = 0;                           /* 标"暂无有效帧" */

    g_rc.last_rx_tick = HAL_GetTick();      /* 失联计时基准 */
    g_rc.online = 0;                        /* 未上线 */
    g_rc.mode = MOTOR_MODE_SAFE;            /* ★初始安全模式，防上电乱动 */
}

/* ===== 中断回调：找帧（要快，不解析） ===== */
void Sbus_OnRxData(uint8_t *buf, uint16_t len)
{
    if (len >= SBUS_FRAME_SIZE) {           /* 数据够一帧才处理 */
        for (uint16_t i = 0; i + SBUS_FRAME_SIZE <= len; i++) {  /* 滑动找起点 */
            if (buf[i] == SBUS_HEADER &amp;&amp;                        /* 是0x0F */
                buf[i + SBUS_FRAME_SIZE - 1] == SBUS_FOOTER) {  /* 且24字节后是0x00 */
                memcpy(s_frame, &amp;buf[i], SBUS_FRAME_SIZE);      /* 拷出整帧 */
                s_rx_len = SBUS_FRAME_SIZE;                     /* 置"有帧"标志 */
                break;                                          /* 一帧够，跳出 */
            }
        }
    }
}

/* ===== 解析任务（4ms 一次） ===== */
void Sbus_Task(void)
{
    uint32_t now = HAL_GetTick();           /* 当前时间 */

    /* ★安全闸1：失联检测 */
    if ((now - g_rc.last_rx_tick) > SBUS_RC_DEADLINE) {  /* 超100ms没新帧 */
        g_rc.online = 0;                    /* 标离线 */
        g_rc.mode   = MOTOR_MODE_SAFE;      /* 强制安全 */
        g_rc.lv_norm = 0.0f;                /* 摇杆清零 */
        g_rc.rv_norm = 0.0f;
        return;                             /* 不再往下解析 */
    }

    if (s_rx_len != SBUS_FRAME_SIZE) {      /* 还没收到完整帧 */
        return;
    }
    s_rx_len = 0;                           /* 消费标志：防同帧重复解析 */

    if (s_frame[0] != SBUS_HEADER || s_frame[24] != SBUS_FOOTER) {  /* ★安全闸2：再校验 */
        return;
    }

    Sbus_DecodeChannels(s_frame, g_rc.ch);  /* 解16通道 → g_rc.ch[] */
    g_rc.last_rx_tick = now;                /* 刷新收到时间 */
    g_rc.online = 1;                        /* 标在线 */

    g_rc.frame_lost = (s_frame[23] &amp; SBUS_FLAG_FRAME_LOST) ? 1 : 0;  /* 读丢帧标志 */
    g_rc.failsafe   = (s_frame[23] &amp; SBUS_FLAG_FAILSAFE)   ? 1 : 0;  /* 读失控标志 */

    /* 归一化：原始值 → -1~+1 */
    g_rc.lv_norm = ((float)g_rc.ch[RC_CH_LV] - SBUS_CH_MID) / SBUS_CH_HALF_RANGE;
    g_rc.rv_norm = ((float)g_rc.ch[RC_CH_RV] - SBUS_CH_MID) / SBUS_CH_HALF_RANGE;

    if (g_rc.lv_norm >  1.0f) g_rc.lv_norm =  1.0f;   /* 钳位上限 */
    if (g_rc.lv_norm < -1.0f) g_rc.lv_norm = -1.0f;   /* 钳位下限 */
    if (g_rc.rv_norm >  1.0f) g_rc.rv_norm =  1.0f;
    if (g_rc.rv_norm < -1.0f) g_rc.rv_norm = -1.0f;

    /* SWA5 三档判断 */
    int16_t swa5 = g_rc.ch[RC_CH_SWA5];
    if (swa5 > SW_HIGH_TH) {            /* 最上 >1250 */
        g_rc.mode = MOTOR_MODE_POS;     /* → 位置模式 */
    } else if (swa5 < SW_LOW_TH) {      /* 最下 <700 */
        g_rc.mode = MOTOR_MODE_SPD;     /* → 速度模式 */
    } else {                            /* 中间 */
        g_rc.mode = MOTOR_MODE_SAFE;    /* → 安全（等于急停档） */
    }

    /* ★安全闸3：失控标志最高优先级 */
    if (g_rc.failsafe) {
        g_rc.mode = MOTOR_MODE_SAFE;
    }
}

/* ===== 暴露接收缓冲给 main.c ===== */
uint8_t *Sbus_GetRxBuf(void)
{
    return s_rx_buf;
}
