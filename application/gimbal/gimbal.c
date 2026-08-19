//
// Created by salen on 2026/8/10.
//
/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "gimbal.h"

#include "cmsis_os.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "can_protocol.h"
#include "can_bus.h"
#include "state.h"

//外部变量
extern DMA_HandleTypeDef hdma_adc1;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
can_link_state_t can_link = {0};
motor_control_t  motor_ctrl = {0};
chassis_state_t  chassis = {0};
gimbal_state_t   gimbal = {0};
static uint16_t center_y = 2048;
static uint16_t center_x = 2048;
volatile uint16_t adc_buf[2] = {2048, 2048};   // 板级私有：DMA 持续写入的摇杆原始值（CH0/CH1）

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void Input_Init(void);
static void Input_read_and_map(void);
static void servo_output(void);
static void Can_control_Pack(uint8_t *tx_data);

/*********************************************************************************************************
*                                              任务与初始化
*********************************************************************************************************/

/** 云台输入任务：读取摇杆、更新目标并输出舵机 PWM。 */
void StartInputTask(void const * argument)
{
    Input_Init();
    gimbal.servo_online = 1;       //舵机在线

    for (;;)
    {
        Input_read_and_map();
        servo_output();
        osDelay(10);
    }
}

/** 云台控制帧发送任务。 */
void StartCanSendTask(void const * argument)
{
    uint8_t tx_data[8];

    for (;;)
    {
        Can_control_Pack(tx_data);
        can_bus_send(CAN_CTRL_FRAME_ID, tx_data);
        osDelay(5);                         // 5ms 周期
    }
}

/** 板间通信健康检查任务。 */
void StartHealthTask(void const * argument)
{
    for (;;)
    {
        if (HAL_GetTick() - can_link.last_rx_tick > 100)  // 100ms 没收到有效帧
            can_link.can_comm_ok = 0;                     //  板间通信异常
        else
            can_link.can_comm_ok = 1;                     //  正常
        osDelay(20);
    }
}

/** 故障指示灯任务。 */
void StartLedTask(void const * argument)
{
    uint8_t duty = 0;        // 当前亮度 0~20
    int8_t  dir = 1;         // 呼吸方向（亮→灭 / 灭→亮）
    uint8_t pwm_cnt = 0;     // 20ms 周期内的位置

    for (;;)
    {
#ifdef BOARD_GIMBAL
        uint8_t fault = !can_link.can_comm_ok;   // 云台：CAN 通信异常
#else
        uint8_t fault = chassis.motor_fault;    // 底盘：电机异常
#endif
        if (fault)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (pwm_cnt < duty) ? GPIO_PIN_RESET : GPIO_PIN_SET);
            pwm_cnt++;
            if (pwm_cnt >= 20)
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
        osDelay(1);
    }
}






/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/** 启动 ADC DMA，并记录摇杆中位值。 */
static void Input_Init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);   // 启动 DMA 采样
    osDelay(5);
    center_y = adc_buf[0];
    center_x = adc_buf[1];
    __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);   // DMA 静默搬运，任务轮询读
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // 启动舵机 PWM50Hz
}

/** 读取摇杆原始值并映射为电机、舵机目标速度。 */
static void Input_read_and_map(void)
{
    // 读原始值：CH0=Y轴（电机），CH1=X轴（舵机）
    int32_t y_raw = adc_buf[0];
    int32_t x_raw = adc_buf[1];

    y_raw -= center_y;
    if (y_raw > -20 && y_raw < 20) y_raw = 0;            // 死区
    motor_ctrl.motor_target_speed = (int16_t)(y_raw * 6000 / 2048); // 映射

    x_raw -= center_x;
    if (x_raw > -20 && x_raw < 20) x_raw = 0;
    gimbal.servo_target_speed = (int16_t)(x_raw * 2000 / 2048); // 舵机目标转速
}

/** 输出舵机 PWM 比较值。 */
static void servo_output(void)
{
    int32_t pulse = 1500 + gimbal.servo_target_speed / 2;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pulse);
}

/** 组装云台控制帧。 */
static void Can_control_Pack(uint8_t *tx_data)
{
    control_frame_t cf = {0};
    static uint8_t tx_seq = 0;
    cf.motor_target_speed = motor_ctrl.motor_target_speed;        //  摇杆指令
    cf.servo_target_speed = gimbal.servo_target_speed;        //  舵机指令（同上）
    cf.version = 1; //协议版本
    cf.gimbal_state = 0;
    cf.gimbal_state |= gimbal.servo_online ? 0x01 : 0x00;   // 位0：舵机在线
    cf.gimbal_state |= can_link.can_comm_ok ? 0x02 : 0x00;    // 位1：板间通信
    cf.number = tx_seq++;                   // 先填序号（自然回绕）
    //pack 成 8 字节
    pack_control_frame(&cf, tx_data);
}
