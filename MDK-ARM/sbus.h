#ifndef __SBUS_H
#define __SBUS_H

#include "app_config.h"

#define SBUS_RXBUF_SIZE  (SBUS_FRAME_SIZE * 2U)

void Sbus_Init(void);
void Sbus_OnRxData(uint8_t *buf, uint16_t len);
void Sbus_Task(void);
uint8_t *Sbus_GetRxBuf(void);

#endif