#ifndef __CAN_MOTOR_H
#define __CAN_MOTOR_H

#include "app_config.h"

void CanMotor_Init(void);
void CanMotor_SendCurrent(int16_t current);
void CanMotor_OnRxFrame(uint32_t std_id, uint8_t *data, uint8_t len);

#endif