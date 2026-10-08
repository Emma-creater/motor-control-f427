#ifndef __CAN_MOTOR_H           /* 防重复包含 */
#define __CAN_MOTOR_H

#include "app_config.h"         /* 需要 MOTOR_ID 等宏 */

void CanMotor_Init(void);                   /* 配过滤器+启动CAN+开中断 */
void CanMotor_SendCurrent(int16_t current); /* 发电流给C620 */
void CanMotor_OnRxFrame(uint32_t std_id, uint8_t *data, uint8_t len);
                                            /* CAN中断回调里调用：解析反馈 */
#endif
