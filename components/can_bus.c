#include "can_bus.h"
#include "can.h"          // hcan 句柄（CubeMX 生成）
#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

//条件编译定义接收 ID
#ifdef BOARD_GIMBAL
#define CAN_RX_ID  0x201   // 云台收反馈帧
#else
#define CAN_RX_ID  0x101   // 底盘收控制帧
#endif
//如果这里不定义，下面 can_bus_init 里 CAN_RX_ID 会编译报错


void can_bus_init(void)
{
    /* CAN 过滤器：只放行本板关心的帧，其余硬件直接丢弃 */
    CAN_FilterTypeDef filter = {0};
    filter.FilterIdHigh      = (uint16_t)(CAN_RX_ID << 5);  // 标准帧 ID 存高 11 位
    filter.FilterIdLow       = 0;
    filter.FilterMaskIdHigh  = 0x7FF << 5;                  // 11 位全部精确匹配
    filter.FilterMaskIdLow   = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank        = 0;
    filter.FilterMode        = CAN_FILTERMODE_IDMASK;
    filter.FilterScale       = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation  = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &filter);

    /* 启动 CAN + 激活 FIFO0 接收中断（收到帧会进回调） */
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void can_bus_send(uint16_t id, uint8_t *data)
{
    CAN_TxHeaderTypeDef hdr = {0};//好比寄一封信，这就是定义信的结构体
    hdr.StdId = id;//好比收件地址，过底盘过滤器用
    hdr.IDE = CAN_ID_STD;//好比信封规格，两种帧格式，标准帧够用就不用扩展帧
    hdr.RTR = CAN_RTR_DATA;//好比是信的类型，数据帧类型
    hdr.DLC   = 8;//好比是信的页数，8字节
    uint32_t mailbox;//好比回执单号，API签名要求有这个参数
    HAL_CAN_AddTxMessage(&hcan, &hdr, data, &mailbox);//这步好比投信出去
}


/* CAN 接收中断回调（中断上下文！只搬数据，不做解析） */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t rx_data[8];

    /* 从 FIFO0 取出整帧（8 字节） */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, rx_data);

    /* 整帧入队（FromISR 版本：中断里专用，不阻塞） */
    BaseType_t need_yield = pdFALSE;
    xQueueSendFromISR(can_rx_queue, rx_data, &need_yield);

    /* 如果刚才打断的任务优先级低于等着收队首的任务，立即切换 */
    portYIELD_FROM_ISR(need_yield);
}
