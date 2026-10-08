#include "app_config.h"
#include "sbus.h"
#include "can_motor.h"
#include "usart.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        Sbus_OnRxData(Sbus_GetRxBuf(), Size);
        HAL_UARTEx_ReceiveToIdle_IT(& huart1, Sbus_GetRxBuf(), SBUS_RXBUF_SIZE);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan->Instance == CAN1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, & rx_header, rx_data) == HAL_OK) {
            CanMotor_OnRxFrame(rx_header.StdId, rx_data, rx_header.DLC);
        }
    }
}