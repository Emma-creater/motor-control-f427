#include "can_motor.h"      /* 本模块头文件 */
#include "can.h"            /* CubeMX 生成的 hcan1 句柄 */
#include <string.h>

static CAN_TxHeaderTypeDef s_tx_header;   /* 发送报文头（ID/长度等） */
static uint8_t             s_tx_data[8];  /* 发送数据缓冲 */
static uint32_t            s_tx_mailbox;  /* 邮箱编号（HAL回填，我们不用） */

/* ===== 初始化：过滤器 + 启动 + 中断 ===== */
void CanMotor_Init(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;  /* ID+掩码模式 */
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;  /* 32位过滤器 */
    filter.FilterIdHigh         = (C620_RX_ID_BASE << 5); /* 期望ID（左移5位对齐） */
    filter.FilterIdLow          = 0x0000;                 /* 低16位不用 */
    filter.FilterMaskIdHigh     = (0x7FC << 5);           /* ★掩码：放行0x201~0x204 */
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;           /* 收进FIFO0 */
    filter.FilterBank           = 0;                      /* 用0号过滤器 */
    filter.FilterActivation     = ENABLE;                 /* 使能 */
    filter.SlaveStartFilterBank = 14;                     /* F4需填（从机过滤器起始组） */
    HAL_CAN_ConfigFilter(&amp;hcan1, &amp;filter);                /* 下发给硬件 */

    HAL_CAN_Start(&amp;hcan1);                                /* 启动CAN外设 */
    HAL_CAN_ActivateNotification(&amp;hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); /* 开收中断 */

    s_tx_header.IDE   = CAN_ID_STD;     /* 标准帧（11位ID） */
    s_tx_header.RTR   = CAN_RTR_DATA;   /* 数据帧 */
    s_tx_header.DLC   = 8;              /* 8字节 */
    s_tx_header.StdId = C620_TX_ID;     /* 控制帧ID=0x200 */
}

/* ===== 发控制电流 ===== */
void CanMotor_SendCurrent(int16_t current)
{
    int16_t cur[4] = {0, 0, 0, 0};      /* 4个电机槽位，默认0 */
    cur[MOTOR_ID - 1] = current;        /* 只填自己那格（ID1→cur[0]） */

    s_tx_data[0] = (uint8_t)(cur[0] >> 8);    /* 大端：高字节在前 */
    s_tx_data[1] = (uint8_t)(cur[0] &amp; 0xFF);  /* 低字节 */
    s_tx_data[2] = (uint8_t)(cur[1] >> 8);    /* ID2（本工程填0） */
    s_tx_data[3] = (uint8_t)(cur[1] &amp; 0xFF);
    s_tx_data[4] = (uint8_t)(cur[2] >> 8);    /* ID3 */
    s_tx_data[5] = (uint8_t)(cur[2] &amp; 0xFF);
    s_tx_data[6] = (uint8_t)(cur[3] >> 8);    /* ID4 */
    s_tx_data[7] = (uint8_t)(cur[3] &amp; 0xFF);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&amp;hcan1) > 0) {  /* 有空邮箱才发，防阻塞 */
        HAL_CAN_AddTxMessage(&amp;hcan1, &amp;s_tx_header, s_tx_data, &amp;s_tx_mailbox);
    }
}

/* ===== 解析反馈（CAN中断里调用） ===== */
void CanMotor_OnRxFrame(uint32_t std_id, uint8_t *data, uint8_t len)
{
    if (len != 8) return;                                 /* 长度不对，丢 */
    if (std_id != (C620_RX_ID_BASE + MOTOR_ID)) return;   /* 不是本电机的帧，丢 */

    uint16_t ecd  = (uint16_t)((data[0] << 8) | data[1]); /* 机械角0~8191，大端拼 */
    int16_t  rpm  = (int16_t)((data[2] << 8) | data[3]);  /* 转子转速rpm，大端拼 */
    int16_t  cur  = (int16_t)((data[4] << 8) | data[5]);  /* 实际电流，大端拼 */
    uint8_t  temp = data[6];                              /* 温度 */

    /* ===== 多圈角度累加：把"单圈读数"修成"连续角度" ===== */
    static float last_angle_deg = 0.0f;   /* 累计角（static：跨调用保持） */
    static int32_t last_ecd_i   = -1;     /* 上次编码器值，-1=未初始化 */
    if (last_ecd_i < 0) last_ecd_i = ecd; /* 首次进来先记基准 */

    int32_t d = (int32_t)ecd - last_ecd_i;  /* 本次增量 */
    if (d >  4096) d -= 8192;               /* ★跨界修正：8190→2 其实在正转 */
    if (d < -4096) d += 8192;               /* ★跨界修正：2→8190 其实在反转 */
    last_ecd_i = ecd;                       /* 更新基准 */

    /* 增量→角度：÷8192×360得转子角，再÷减速比得输出轴角 */
    last_angle_deg += (float)d * (360.0f / 8192.0f) / MOTOR_GEAR_RATIO;

    g_motor.ecd         = ecd;            /* 存原始编码器值 */
    g_motor.speed_rpm   = rpm;            /* 存转速 */
    g_motor.current_raw = cur;            /* 存电流 */
    g_motor.temperature = temp;           /* 存温度 */
    g_motor.angle_deg   = last_angle_deg; /* ★位置环反馈 */
    g_motor.speed_dps   = (float)rpm * (360.0f / 60.0f) / MOTOR_GEAR_RATIO;
    /* ↑ rpm→度/秒：×360÷60=×6，再÷减速比得到输出轴角速度 ★速度环反馈 */
    g_motor.last_update_tick = HAL_GetTick();  /* 时间戳（判掉线） */
    g_motor.online = 1;                        /* 标记电机在线 */
}
