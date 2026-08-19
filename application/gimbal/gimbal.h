//
// Created by salen on 2026/8/10.
//

#ifndef TASK_GIMBAL_H
#define TASK_GIMBAL_H

/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
void StartInputTask(void const * argument);
void StartCanSendTask(void const * argument);
void StartHealthTask(void const * argument);
void StartLedTask(void const * argument);

#endif // TASK_GIMBAL_H
