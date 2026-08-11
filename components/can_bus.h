//
// Created by salen on 2026/8/4.
//

#ifndef TASK_CAN_BUS_H
#define TASK_CAN_BUS_H
#include <stdint.h>

//启用过滤器+激活中断
void can_bus_init(void);
//通用发送函数，id+已打包8字节
void can_bus_send(uint16_t id, uint8_t *data);

#endif //TASK_CAN_BUS_H
