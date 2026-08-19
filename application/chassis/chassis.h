//
// Created by salen on 2026/8/10.
//

#ifndef TASK_CHASSIS_H
#define TASK_CHASSIS_H

/*********************************************************************************************************
*                                              对外允许调用文件
*********************************************************************************************************/
void StartMotorTask(void const * argument);
void StartCanSendTask(void const * argument);
void StartHealthTask(void const * argument);
void StartLedTask(void const * argument);
void StartDisplayTask(void const * argument);

#endif // TASK_CHASSIS_H
