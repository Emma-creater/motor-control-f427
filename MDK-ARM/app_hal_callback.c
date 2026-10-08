#include "app_config.h"
#include "sbus.h"
#include "can_motor.h"
#include "usart.h"              /* huart1 声明在这里 */

/* ===== UART 回调：SBUS 字节流入口 =====
 * HAL 回调都是 __weak 空函数，这里写同名函数即"接管"
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {        /* 确认是 USART1 触发 */
        Sbus_OnRxData(Sbus_GetRxBuf(), Size);   /* 喂给 SBUS 找帧 */

        HAL_UARTEx_ReceiveToIdle_IT(&amp;huart1, Sbus_GetRxBuf(), SBUS_RXBUF_SIZE);
        /* ★HAL 接收是一次性的，必须重启，否则只收一帧就哑 */
    }
}

/* ===== CAN 回调：电机反馈入口 ===== */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;      /* 报文头 */
    uint8_t rx_data[8];                 /* 报文数据 */

    if (hcan->Instance == CAN1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &amp;rx_header, rx_data) == HAL_OK) {
            /* 从硬件 FIFO 取出报文（不取会堆满） */

            CanMotor_OnRxFrame(rx_header.StdId, rx_data, rx_header.DLC);
            /* 交给电机模块：匹配ID → 更新 g_motor */
        }
    }
}
