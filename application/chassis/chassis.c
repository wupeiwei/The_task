//
// Created by salen on 2026/8/10.
//

/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "chassis.h"

#include "cmsis_os.h"
#include "main.h"
#include "tim.h"
#include "can_protocol.h"
#include "can_bus.h"
#include "state.h"
#include "PID.h"
#include "oled.h"
#include <stdio.h>

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
can_link_state_t can_link = {0};
motor_control_t  motor_ctrl = {0};
chassis_state_t  chassis = {0};
gimbal_state_t   gimbal = {0};

static int16_t target = 0;   // 控制目标
static uint16_t last_cnt = 0;
static uint32_t last_tick = 0;
static uint16_t acc_cnt = 0;
static uint32_t acc_tick = 0;
static int16_t rpm = 0;    // 测速结果
static int16_t diff = 0;   // 窗口计数差
static pid_t motor_pid;

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void Motor_Speed_Update(void);
static void Motor_Statu_Update(void);
static void Motor_PID_Calculate(void);
static void Motor_Output(void);
static void Motor_Fault_Check(void);
static void Can_Feedback_Pack(uint8_t *tx_data);
static void str_pad(char *buf, uint8_t width);






/*********************************************************************************************************
*                                              任务与初始化
*********************************************************************************************************/

/** 底盘电机控制任务。 */
void StartMotorTask(void const * argument)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    pid_init(&motor_pid, 0.6f, 0.1f, 0.0f, 3000.0f);

    last_tick = HAL_GetTick();
    acc_tick = HAL_GetTick();

    /* Infinite loop */
    for (;;)
    {
        Motor_Speed_Update();
        Motor_Statu_Update();
        Motor_PID_Calculate();
        Motor_Output();
        Motor_Fault_Check();
        osDelay(10);
    }
}

/** 底盘反馈帧发送任务。 */
void StartCanSendTask(void const * argument)
{
    uint8_t tx_data[8];

    /* Infinite loop */
    for (;;)
    {
        Can_Feedback_Pack(tx_data);
        can_bus_send(CAN_FB_FRAME_ID, tx_data);
        osDelay(10);
    }
}

/** 板间通信健康检查任务。 */
void StartHealthTask(void const * argument)
{
    /* Infinite loop */
    for (;;)
    {
        if (HAL_GetTick() - can_link.last_rx_tick > 100)  // 100ms 没收到有效帧
        {
            can_link.can_comm_ok = 0;                     // 板间通信异常
            motor_ctrl.motor_target_speed = 0;
        }
        else
            can_link.can_comm_ok = 1;                     // 正常

        osDelay(20);
    }
}

/** 故障指示灯任务。 */
void StartLedTask(void const * argument)
{
    uint8_t duty = 0;        // 当前亮度 0~20
    int8_t  dir = 1;         // 呼吸方向
    uint8_t pwm_cnt = 0;     // 20ms 周期内的位置

    /* Infinite loop */
    for (;;)
    {
#ifdef BOARD_GIMBAL
        uint8_t fault = !can_link.can_comm_ok;   // 云台：CAN 通信异常
#else
        uint8_t fault = chassis.motor_fault;    // 底盘电机异常
#endif
        if (fault)
        {
            /* 软件 PWM：亮 duty/20，灭 (20-duty)/20 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (pwm_cnt < duty) ? GPIO_PIN_RESET : GPIO_PIN_SET);
            pwm_cnt++;
            if (pwm_cnt >= 20)            // 一个 PWM 周期结束
            {
                pwm_cnt = 0;
                duty += dir;                // 亮度调一级
                if (duty >= 20 || duty == 0) dir = -dir;   // 到头反向
            }
        }
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // 正常：灭
        }
        osDelay(1);                     // 1ms 节拍（PWM 周期 20ms，20 级亮度）
    }
}

/** OLED 状态显示任务。 */
void StartDisplayTask(void const * argument)
{
    osDelay(1000);
    oled_init();
    oled_clear();
    char buf[22];

    /* Infinite loop */
    for (;;)
    {
        // 行0：舵机目标转速 + 在线
        sprintf(buf, "SERVO:%5dRPM %s", gimbal.servo_target_speed, gimbal.servo_online ? "ON " : "OFF");
        str_pad(buf, 21);
        oled_show_string(0, 0, buf);

        // 行2：电机目标/实际转速
        sprintf(buf, "MOTOR:%4d/%4d", motor_ctrl.motor_target_speed, motor_ctrl.motor_current_speed);
        str_pad(buf, 21);
        oled_show_string(2, 0, buf);

        // 行4：电机在线 + 异常
        sprintf(buf, "MST:%s FUT:%s", chassis.motor_online ? "ON " : "OFF", chassis.motor_fault ? "YES" : "NO ");
        str_pad(buf, 21);
        oled_show_string(4, 0, buf);

        // 行6：板间通信
        sprintf(buf, "CAN:%s ", can_link.can_comm_ok ? "OK " : "ERR");
        str_pad(buf, 21);
        oled_show_string(6, 0, buf);
        osDelay(100);                    // 100ms 刷新（人眼够用，省 CPU）
    }
}

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/

/** 根据编码器计数更新电机实际转速。 */
static void Motor_Speed_Update(void)
{
    // 测速
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
    diff = (int16_t)(cnt - last_cnt);      // int16 相减，溢出自动回绕
    last_cnt = cnt;
    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_tick;   // 无符号差值
    last_tick = now;

    if (dt_ms == 0) dt_ms = 1;          // 防除零

    // 换算
    rpm = diff * 60000 / (44 * (int32_t)dt_ms);

    // 反馈实际转速
    motor_ctrl.motor_current_speed = rpm;
}

/** 根据通信状态更新当前控制目标。 */
static void Motor_Statu_Update(void)
{
    if (can_link.can_comm_ok)
    {
        target = motor_ctrl.motor_target_speed;
    }
    else
    {
        target = 0;
        pid_reset(&motor_pid);
    }
}

/** 计算电机速度环 PID 输出。 */
static void Motor_PID_Calculate(void)
{
    // motor_pid.u = 300.0f;  // 测试用
    pid_calc(&motor_pid, (float)target, (float)rpm);
}

/** 将 PID 输出转换为 PWM 和方向信号。 */
static void Motor_Output(void)
{
    int32_t pwm = (int32_t)(motor_pid.u * 3599 / 1000);
    if (pwm < 0) pwm = -pwm;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);

    if (motor_pid.u >= 0)
    { // 正转/停：AIN1=1 AIN2=0
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }
    else
    { // 反转：AIN1=0 AIN2=1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
}

/** 检查堵转状态并更新电机在线标志。 */
static void Motor_Fault_Check(void)
{
    // 堵转检测：500ms 计数域累计（粒度 2.7 RPM/计数，低速不误报）
    acc_cnt += (uint16_t)(diff < 0 ? -diff : diff);
    if (HAL_GetTick() - acc_tick >= 500)
    {
        if (target != 0 && acc_cnt < 3)
        {
            chassis.motor_fault = 1;
        }
        else if (target != 0 && acc_cnt >= 3)
        {
            chassis.motor_fault = 0;                           // 目标归零自动恢复
        }

        if (target != 0 && acc_cnt >= 3)  chassis.motor_online = 1;   // 有指令且动了  在线
        else                              chassis.motor_online = 0;   // 堵转  不在线

        acc_cnt = 0;                                   // 窗口复位
        acc_tick = HAL_GetTick();
    }
}

/** 组装底盘反馈帧。 */
static void Can_Feedback_Pack(uint8_t *tx_data)
{
    feedback_frame_t ff = {0};             // 局部：每周期重新填（原来是任务局部，挪进来）
    static uint8_t tx_seq = 0;             // 序号计数器
    ff.motor_target_speed_echo = motor_ctrl.motor_target_speed;
    ff.motor_current_speed = motor_ctrl.motor_current_speed;
    ff.version = 1;
    ff.chassis_state = 0;
    ff.chassis_state |= chassis.motor_online ? 0x01 : 0x00;
    ff.chassis_state |= can_link.can_comm_ok ? 0x02 : 0x00;
    ff.chassis_state |= chassis.motor_fault ? 0x04 : 0x00;
    ff.number = tx_seq++;
    pack_feedback_frame(&ff, tx_data);
}

/** 将显示行补齐到固定宽度。 */
static void str_pad(char *buf, uint8_t width)
{
    uint8_t len = 0;
    while (len < width && buf[len] != '\0') len++;
    while (len < width) buf[len++] = ' ';
    buf[len] = '\0';
}
