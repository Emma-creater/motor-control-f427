#ifndef __MOTOR_CTRL_H          /* 防重复包含 */
#define __MOTOR_CTRL_H

#include "app_config.h"

void MotorCtrl_Init(void);      /* 初始化：装PID参数+CAN（上电一次） */
void MotorCtrl_Task(void);      /* 每2ms调用：按模式跑PID并发电流 */

#endif
