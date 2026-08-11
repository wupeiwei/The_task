#include "can_bus.h"
#include "can.h"          // hcan 句柄（CubeMX 生成）
#include "can_protocol.h"
#include "state.h"        // 共享状态实例
#include <stdint.h>

//条件编译定义接收 ID
#ifdef BOARD_GIMBAL
#define CAN_RX_ID  0x201   // 云台收反馈帧
#else
#define CAN_RX_ID  0x101   // 底盘收控制帧
#endif

uint32_t tx_fail_cnt = 0;   // 发送失败计数

void can_bus_init(void)
{
    /* CAN 过滤器 */
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

    /* 启动 CAN + 激活 FIFO0 接收中断 */
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void can_bus_send(uint16_t id, uint8_t *data)
{
    CAN_TxHeaderTypeDef hdr = {0};
    hdr.StdId = id;
    hdr.IDE = CAN_ID_STD;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC   = 8;
    uint32_t mailbox;

    if (HAL_CAN_AddTxMessage(&hcan, &hdr, data, &mailbox) != HAL_OK)
    {
        tx_fail_cnt++;     // 发送失败计数诊断用
    }
}


// CAN 接收中断回调直解
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, rx_data);

    //中断直解：解析 + 写共享变量 + 序号/心跳检查
#ifdef BOARD_GIMBAL
    feedback_frame_t ff;
    if (parse_feedback_frame(rx_data, &ff))
    {
        if (ff.version != 1) { /* 版本不符，整帧丢弃 */ }
        else if (ff.number == can_link.last_rx_seq) { can_link.dup_cnt++; }
        else {
            motor_ctrl.motor_current_speed = ff.motor_current_speed;
            motor_ctrl.motor_echo = ff.motor_target_speed_echo;
            chassis.chassis_motor_fault = (ff.chassis_state & 0x04) ? 1 : 0;
            if ((uint8_t)(ff.number - can_link.last_rx_seq) > 1) can_link.drop_cnt++;
            can_link.last_rx_seq = ff.number;
            can_link.last_rx_tick = HAL_GetTick();
        }
    }
#else
    control_frame_t cf;
    if (parse_control_frame(rx_data, &cf))
    {
        if (cf.version != 1) { /* 版本不符，整帧丢弃 */ }
        else if (cf.number == can_link.last_rx_seq) { can_link.dup_cnt++; }
        else {
            motor_ctrl.motor_target_speed = cf.motor_target_speed;
            gimbal.servo_target_speed = cf.servo_target_speed;
            gimbal.servo_online = (cf.gimbal_state & 0x01) ? 1 : 0;
            if ((uint8_t)(cf.number - can_link.last_rx_seq) > 1) can_link.drop_cnt++;
            can_link.last_rx_seq = cf.number;
            can_link.last_rx_tick = HAL_GetTick();
        }
    }
#endif
}
