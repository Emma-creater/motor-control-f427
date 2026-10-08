#ifndef __SBUS_H                /* 防重复包含 */
#define __SBUS_H

#include "app_config.h"         /* 需要 SBUS_FRAME_SIZE 等宏 */

/* UART 接收缓冲长度：开2帧容量，防一帧跨两次中断 */
#define SBUS_RXBUF_SIZE  (SBUS_FRAME_SIZE * 2U)

void Sbus_Init(void);                           /* 初始化：清缓冲、标离线 */
void Sbus_OnRxData(uint8_t *buf, uint16_t len); /* 中断里调：找帧、存整帧 */
void Sbus_Task(void);                           /* 任务里调：解析+模式判断+失联检测 */
uint8_t *Sbus_GetRxBuf(void);                   /* 暴露接收缓冲给 main.c */

#endif
