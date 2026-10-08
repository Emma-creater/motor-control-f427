#include "app_config.h"
#include "sbus.h"
#include "motor_ctrl.h"

osThreadId sbusTaskHandle;
osThreadId ctrlTaskHandle;

void StartSbusTask(void const *argument);
void StartCtrlTask(void const *argument);

osThreadDef(sbusTask, StartSbusTask, osPriorityHigh, 0, 512);
osThreadDef(ctrlTask, StartCtrlTask, osPriorityAboveNormal, 0, 512);

void StartSbusTask(void const *argument)
{
    (void)argument;

    for (;;) {
        Sbus_Task();
        osDelay(SBUS_PERIOD_MS);
    }
}

void StartCtrlTask(void const *argument)
{
    (void)argument;

    MotorCtrl_Init();

    while (!g_rc.online) {
        osDelay(10);
    }

    for (;;) {
        MotorCtrl_Task();
        osDelay(CTRL_PERIOD_MS);
    }
}

void App_FreertosInit(void)
{
    sbusTaskHandle = osThreadCreate(osThread(sbusTask), NULL);
    ctrlTaskHandle = osThreadCreate(osThread(ctrlTask), NULL);
}