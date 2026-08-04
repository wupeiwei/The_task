//
// Created by salen on 2026/8/4.
//


#ifndef TASK_CAN_PROTOCOL_H
#define TASK_CAN_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>

uint8_t crc8(const uint8_t *data, size_t len);

#define CAN_CTRL_FRAME_ID 0x101
typedef struct
{
    int16_t motor_target_speed;   //0-1字段
    int16_t servo_target_speed;   //2-3字段
    uint8_t number;   //4字段
    uint8_t gimbal_state;   //5字段
    uint8_t version;   //6字段
}control_frame_t;
void pack_control_frame(const control_frame_t *cf, uint8_t *tx_data);
uint8_t parse_control_frame(const uint8_t *rx_data, control_frame_t *cf);


#define CAN_FB_FRAME_ID 0x201
typedef struct
{
    int16_t motor_target_speed_echo;   //0-1字段
    int16_t motor_current_speed;  //2-3字段
    uint8_t number;   //4字段
    uint8_t chassis_state;   //5字段
    uint8_t version;   //6字段
}feedback_frame_t;
void pack_feedback_frame(const feedback_frame_t *ff, uint8_t *tx_data);
uint8_t parse_feedback_frame(const uint8_t *rx_data, feedback_frame_t *ff);
#endif //TASK_CAN_PROTOCOL_H
