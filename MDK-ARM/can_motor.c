#include "can_motor.h"
#include "can.h"
#include <string.h>

static CAN_TxHeaderTypeDef s_tx_header;
static uint8_t             s_tx_data[8];
static uint32_t            s_tx_mailbox;

void CanMotor_Init(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (C620_RX_ID_BASE << 5);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (0x7FC << 5);
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(& hcan1, & filter);

    HAL_CAN_Start(& hcan1);
    HAL_CAN_ActivateNotification(& hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    s_tx_header.IDE   = CAN_ID_STD;
    s_tx_header.RTR   = CAN_RTR_DATA;
    s_tx_header.DLC   = 8;
    s_tx_header.StdId = C620_TX_ID;
}

void CanMotor_SendCurrent(int16_t current)
{
    int16_t cur[4] = {0, 0, 0, 0};
    cur[MOTOR_ID - 1] = current;

    s_tx_data[0] = (uint8_t)(cur[0] >> 8);
    s_tx_data[1] = (uint8_t)(cur[0] &  0xFF);
    s_tx_data[2] = (uint8_t)(cur[1] >> 8);
    s_tx_data[3] = (uint8_t)(cur[1] &  0xFF);
    s_tx_data[4] = (uint8_t)(cur[2] >> 8);
    s_tx_data[5] = (uint8_t)(cur[2] &  0xFF);
    s_tx_data[6] = (uint8_t)(cur[3] >> 8);
    s_tx_data[7] = (uint8_t)(cur[3] &  0xFF);

    if (HAL_CAN_GetTxMailboxesFreeLevel(& hcan1) > 0) {
        HAL_CAN_AddTxMessage(& hcan1, & s_tx_header, s_tx_data, & s_tx_mailbox);
    }
}

void CanMotor_OnRxFrame(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (len != 8) return;
    if (std_id != (C620_RX_ID_BASE + MOTOR_ID)) return;

    uint16_t ecd      = (uint16_t)((data[0] << 8) | data[1]);
    int16_t  rpm      = (int16_t)((data[2] << 8) | data[3]);
    int16_t  cur      = (int16_t)((data[4] << 8) | data[5]);
    uint8_t  temp     = data[6];

    static float last_angle_deg = 0.0f;
    static int32_t last_ecd_i   = -1;
    if (last_ecd_i < 0) last_ecd_i = ecd;
    int32_t d = (int32_t)ecd - last_ecd_i;
    if (d >  4096) d -= 8192;
    if (d < -4096) d += 8192;
    last_ecd_i = ecd;
    last_angle_deg += (float)d * (360.0f / 8192.0f) / MOTOR_GEAR_RATIO;

    g_motor.ecd        = ecd;
    g_motor.speed_rpm  = rpm;
    g_motor.current_raw = cur;
    g_motor.temperature = temp;
    g_motor.angle_deg  = last_angle_deg;
    g_motor.speed_dps  = (float)rpm * (360.0f / 60.0f) / MOTOR_GEAR_RATIO;
    g_motor.last_update_tick = HAL_GetTick();
    g_motor.total_round = ecd;
    g_motor.last_ecd    = ecd;
    g_motor.online = 1;
}