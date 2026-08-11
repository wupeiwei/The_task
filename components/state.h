//
// Created by salen on 2026/8/11.
//

#ifndef TASK_STATE_H
#define TASK_STATE_H

#include <stdint.h>

/* 链路健康（板间通信） */
typedef struct {
    volatile uint32_t last_rx_tick;   // 心跳时间戳
    volatile uint8_t  can_comm_ok;    // 板间通信状态
    volatile uint8_t  last_rx_seq;    // 上次收到帧序号
    volatile uint32_t dup_cnt;        // 重复帧计数
    volatile uint32_t drop_cnt;       // 丢帧计数
} can_link_state_t;

/* 电机控制面（指令 + 反馈 + 回显） */
typedef struct {
    volatile int16_t motor_target_speed;   // 目标
    volatile int16_t motor_current_speed;  // 实测
    volatile int16_t motor_echo;           // 指令回显
} motor_control_t;

/* 底盘状态上报 */
typedef struct {
    volatile uint8_t motor_online;          // 底盘电机在线
    volatile uint8_t motor_fault;           // 底盘电机异常
    volatile uint8_t chassis_motor_fault;   // 远端电机异常
} chassis_state_t;

/* 云台状态面 */
typedef struct {
    volatile int16_t servo_target_speed;   // 舵机目标转速
    volatile uint8_t servo_online;         // 舵机在线
} gimbal_state_t;

/* 实例声明 */
extern can_link_state_t can_link;
extern motor_control_t  motor_ctrl;
extern chassis_state_t  chassis;
extern gimbal_state_t   gimbal;

#endif //TASK_STATE_H
