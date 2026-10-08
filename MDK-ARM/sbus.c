#include "sbus.h"
#include <string.h>

static uint8_t  s_rx_buf[SBUS_FRAME_SIZE * 2];
static uint16_t s_rx_len;
static uint8_t  s_frame[SBUS_FRAME_SIZE];

static void Sbus_DecodeChannels(const uint8_t *f, int16_t *ch)
{
    uint32_t bit_pos = 0;

    for (int i = 0; i < 16; i++) {
        int32_t value = 0;
        for (int b = 0; b < 11; b++) {
            uint32_t byte_index = 1 + (bit_pos >> 3);
            uint32_t bit_index  = bit_pos & 0x07;
            uint8_t  bit = (f[byte_index] >> bit_index) & 0x01;
            value |= ((int32_t)bit << b);
            bit_pos++;
        }
        ch[i] = (int16_t)value;
    }
}

void Sbus_Init(void)
{
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    memset(s_frame, 0, sizeof(s_frame));
    s_rx_len = 0;
    g_rc.last_rx_tick = HAL_GetTick();
    g_rc.online = 0;
    g_rc.mode = MOTOR_MODE_SAFE;
}

void Sbus_OnRxData(uint8_t *buf, uint16_t len)
{
    if (len >= SBUS_FRAME_SIZE) {
        for (uint16_t i = 0; i + SBUS_FRAME_SIZE <= len; i++) {
            if (buf[i] == SBUS_HEADER &&
                buf[i + SBUS_FRAME_SIZE - 1] == SBUS_FOOTER) {
                memcpy(s_frame, &buf[i], SBUS_FRAME_SIZE);
                s_rx_len = SBUS_FRAME_SIZE;
                break;
            }
        }
    }
}

void Sbus_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - g_rc.last_rx_tick) > SBUS_RC_DEADLINE) {
        g_rc.online = 0;
        g_rc.mode   = MOTOR_MODE_SAFE;
        g_rc.lv_norm = 0.0f;
        g_rc.rv_norm = 0.0f;
        return;
    }

    if (s_rx_len != SBUS_FRAME_SIZE) {
        return;
    }
    s_rx_len = 0;

    if (s_frame[0] != SBUS_HEADER || s_frame[24] != SBUS_FOOTER) {
        return;
    }

    Sbus_DecodeChannels(s_frame, g_rc.ch);
    g_rc.last_rx_tick = now;
    g_rc.online = 1;

    g_rc.frame_lost = (s_frame[23] & SBUS_FLAG_FRAME_LOST) ? 1 : 0;
    g_rc.failsafe   = (s_frame[23] &  SBUS_FLAG_FAILSAFE)   ? 1 : 0;

    g_rc.lv_norm = ((float)g_rc.ch[RC_CH_LV] - SBUS_CH_MID) / SBUS_CH_HALF_RANGE;
    g_rc.rv_norm = ((float)g_rc.ch[RC_CH_RV] - SBUS_CH_MID) / SBUS_CH_HALF_RANGE;
    if (g_rc.lv_norm >  1.0f) g_rc.lv_norm =  1.0f;
    if (g_rc.lv_norm < -1.0f) g_rc.lv_norm = -1.0f;
    if (g_rc.rv_norm >  1.0f) g_rc.rv_norm =  1.0f;
    if (g_rc.rv_norm < -1.0f) g_rc.rv_norm = -1.0f;

    int16_t swa5 = g_rc.ch[RC_CH_SWA5];
    if (swa5 > SW_HIGH_TH) {
        g_rc.mode = MOTOR_MODE_POS;
    } else if (swa5 < SW_LOW_TH) {
        g_rc.mode = MOTOR_MODE_SPD;
    } else {
        g_rc.mode = MOTOR_MODE_SAFE;
    }

    if (g_rc.failsafe) {
        g_rc.mode = MOTOR_MODE_SAFE;
    }
}

uint8_t *Sbus_GetRxBuf(void)
{
    return s_rx_buf;
}