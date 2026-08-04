#include "can_protocol.h"
#include <stdint.h>
#include <stddef.h>

// Created by salen on 2026/8/4.
//crc校验函数的编写，数据的指纹
uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    uint8_t i,bit;
    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x01)//只取crc的最低位，其余位清0，结果只会是0或1，用作if条件判断
            {
                crc = (crc >> 1) ^ 0x07;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

void pack_control_frame(const control_frame_t *cf, uint8_t *tx_data)
{
    //int16字段：小段拆两字节（低字节在前）
    tx_data[0] =(uint8_t)(cf->motor_target_speed & 0xFF);
    tx_data[1] = (uint8_t)(cf->motor_target_speed>>8);
    tx_data[2] = (uint8_t)(cf->servo_target_speed & 0xFF);
    tx_data[3] = (uint8_t)(cf->servo_target_speed>>8);
    //uint8字段：直接放
    tx_data[4] = cf->number;
    tx_data[5] = cf->gimbal_state;
    tx_data[6] = cf->version;
    //最后算CRC填7字段
    tx_data[7] = crc8(tx_data, 7);
}

void pack_feedback_frame(const feedback_frame_t *ff, uint8_t *tx_data)
{
    //依旧int16字段
    tx_data[0] =(uint8_t)(ff->motor_target_speed_echo & 0xFF);
    tx_data[1] = (uint8_t)(ff->motor_target_speed_echo>>8);
    tx_data[2] = (uint8_t)(ff->motor_current_speed & 0xFF);
    tx_data[3] = (uint8_t)(ff->motor_current_speed>>8);
    //4-6依旧直接放
    tx_data[4] = ff->number;
    tx_data[5] = ff->chassis_state;
    tx_data[6] = ff->version;
    //依旧校验位
    tx_data[7] = crc8(tx_data, 7);
}

//接收解码函数的顺序有点注意，先验CRC，不对就不解码
uint8_t parse_control_frame(const uint8_t *rx_data, control_frame_t *cf)
{
    if (crc8(rx_data, 7) != rx_data[7])
    {
        return 0;
    }

    //小段组合int16，低字节|高字节<<8，强转int16_t保留符号
    cf->motor_target_speed = (int16_t)(rx_data[0] | (rx_data[1] << 8));
    cf->servo_target_speed = (int16_t)(rx_data[2] | (rx_data[3] << 8));
    cf->number = rx_data[4];
    cf->gimbal_state = rx_data[5];
    cf->version = rx_data[6];
    return 1;
}

uint8_t parse_feedback_frame(const uint8_t *rx_data, feedback_frame_t *ff)
{
    if (crc8(rx_data, 7) != rx_data[7])
    {
        return 0;
    }

    //依旧。和控制的解码函数一样
    ff->motor_target_speed_echo = (int16_t)(rx_data[0] | (rx_data[1] << 8));
    ff->motor_current_speed = (int16_t)(rx_data[2] | (rx_data[3] << 8));
    ff->number = rx_data[4];
    ff->chassis_state = rx_data[5];
    ff->version = rx_data[6];
    return 1;
}