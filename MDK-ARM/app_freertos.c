#include "app_config.h"
#include "sbus.h"
#include "motor_ctrl.h"

osThreadId sbusTaskHandle;      /* 任务句柄（创建后返回，备用） */
osThreadId ctrlTaskHandle;

/* 前置声明：osThreadDef 宏要用到这两个函数 */
void StartSbusTask(void const *argument);
void StartCtrlTask(void const *argument);

/* osThreadDef = 静态造"任务档案"：名字/函数/优先级/实例数/栈(字,512字=2KB) */
osThreadDef(sbusTask, StartSbusTask, osPriorityHigh, 0, 512);
osThreadDef(ctrlTask, StartCtrlTask, osPriorityAboveNormal, 0, 512);
/* 接收任务优先级更高：遥控数据讲究新鲜度 */

/* ===== 任务1：收遥控器，4ms 一拍 ===== */
void StartSbusTask(void const *argument)
{
    (void)argument;             /* 参数没用到，此行只为消警告 */

    for (;;) {                  /* RTOS 任务=死循环，绝不 return */
        Sbus_Task();            /* 解析一帧（含失联检测） */
        osDelay(SBUS_PERIOD_MS);/* 让出 CPU 4ms ★必须 osDelay，HAL_Delay 会饿死别的任务 */
    }
}

/* ===== 任务2：控电机，2ms 一拍 ===== */
void StartCtrlTask(void const *argument)
{
    (void)argument;

    MotorCtrl_Init();           /* 初始化 PID+CAN（只跑一次） */

    while (!g_rc.online) {      /* 等遥控器首次上线 */
        osDelay(10);            /* 防止遥控器没开时电机自己动 */
    }

    for (;;) {
        MotorCtrl_Task();       /* 读输入→PID→发CAN */
        osDelay(CTRL_PERIOD_MS);/* 睡 2ms */
    }
}

/* ===== 总入口：被 freertos.c 的 USER CODE 区调用 ===== */
void App_FreertosInit(void)
{
    sbusTaskHandle = osThreadCreate(osThread(sbusTask), NULL);  /* 雇员工A */
    ctrlTaskHandle = osThreadCreate(osThread(ctrlTask), NULL);  /* 雇员工B */
}
