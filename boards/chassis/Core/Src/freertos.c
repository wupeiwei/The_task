/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "can_protocol.h"
#include "tim.h"     // TIM3->CNT、htim1、HAL_TIM_PWM_Start
#include "gpio.h"    // PB12/PB13 方向脚
#include "can_bus.h" // can_bus_send 声明 + can_rx_queue extern
#include "stdio.h"    // sprintf
#include "oled.h"     // OLED 驱动
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t can_rx_queue;   // 定义本体
//共享变量
volatile int16_t motor_target_speed = 0;   // 电机目标速度（底盘：recv写，motor读）
volatile int16_t servo_target_speed = 0;   // 舵机目标转速（底盘：recv写，显示读）
volatile int16_t motor_current_speed = 0;  // 电机实际转速（云台：recv写）
volatile uint32_t last_rx_tick = 0;        // 最近收到有效帧的时间（心跳用）
volatile uint8_t can_comm_ok = 0;          //状态字节和LED
volatile uint8_t servo_online = 0;   // 舵机在线（云台）
volatile uint8_t motor_online = 0;   // 电机在线（底盘）
volatile uint8_t motor_fault  = 0;   // 电机异常（底盘）
volatile uint8_t  last_rx_seq = 0;        // 上次收到的帧序号
volatile uint32_t dup_cnt = 0;            // 重复帧计数
volatile uint32_t drop_cnt = 0;           // 丢帧计数
/* USER CODE END Variables */
osThreadId motor_taskHandle;
osThreadId can_recv_taskHandle;
osThreadId can_send_taskHandle;
osThreadId health_taskHandle;
osThreadId led_taskHandle;
osThreadId display_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void const * argument);
void StartCanRecvTask(void const * argument);
void StartCanSendTask(void const * argument);
void StartHealthTask(void const * argument);
void StartLedTask(void const * argument);
void StartDisplayTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  can_rx_queue = xQueueCreate(8, sizeof(uint8_t[8]));
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of motor_task */
  osThreadDef(motor_task, StartMotorTask, osPriorityHigh, 0, 256);
  motor_taskHandle = osThreadCreate(osThread(motor_task), NULL);

  /* definition and creation of can_recv_task */
  osThreadDef(can_recv_task, StartCanRecvTask, osPriorityAboveNormal, 0, 256);
  can_recv_taskHandle = osThreadCreate(osThread(can_recv_task), NULL);

  /* definition and creation of can_send_task */
  osThreadDef(can_send_task, StartCanSendTask, osPriorityNormal, 0, 128);
  can_send_taskHandle = osThreadCreate(osThread(can_send_task), NULL);

  /* definition and creation of health_task */
  osThreadDef(health_task, StartHealthTask, osPriorityLow, 0, 128);
  health_taskHandle = osThreadCreate(osThread(health_task), NULL);

  /* definition and creation of led_task */
  osThreadDef(led_task, StartLedTask, osPriorityLow, 0, 128);
  led_taskHandle = osThreadCreate(osThread(led_task), NULL);

  /* definition and creation of display_task */
  osThreadDef(display_task, StartDisplayTask, osPriorityLow, 0, 256);
  display_taskHandle = osThreadCreate(osThread(display_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the motor_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);       // 启动 PWM 输出（PA8 开始出波形）
  motor_online = 1;
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动编码器计数

  float kp = 0.0f, ki = 0.0f, kd = 0.0f;           //
  float e1 = 0.0f, e2 = 0.0f;                      // 历史误差（增量式要存两个）
  float u  = 0.0f;

  uint16_t last_cnt = 0;
  uint32_t last_tick = HAL_GetTick();   //  基准时间
  uint16_t acc_cnt = 0;                     // 500ms 累计计数（堵转判定用）
  uint32_t acc_tick = HAL_GetTick();        // 累计窗口起点

  /* Infinite loop */
  for(;;)
  {
    //测速：10ms 窗口计数差 → RPM（44 计数/圈）
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t diff = (int16_t)(cnt - last_cnt);      // int16 相减，溢出自动回绕
    last_cnt = cnt;

    uint32_t now = HAL_GetTick();
    uint32_t dt_ms = now - last_tick;   // 无符号差值
    last_tick = now;
    if (dt_ms == 0) dt_ms = 1;          // 防除零

    // 3. 换算：真实窗口
    int32_t rpm = diff * 60000 / (44 * (int32_t)dt_ms);


    //PID 目标：失联时按 0 处理（即使 health_task 还没跑到，本周期就停）
    int16_t target = can_comm_ok ? motor_target_speed : 0;

    //增量式 PID
    float err = (float)(target - rpm);     // 用 target 替换原 motor_target_speed
    u += kp * (err - e1) + ki * err + kd * (err - 2*e1 + e2);
    e2 = e1;
    e1 = err;

    //限幅 ±1000 → 占空比 → 方向
    if (u > 1000.0f)  u = 1000.0f;
    if (u < -1000.0f) u = -1000.0f;

    int32_t pwm = (int32_t)(u * 3599 / 1000);      // u=1000 → 3599（100%）
    if (pwm < 0) pwm = -pwm;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);

    if (u >= 0) {                                  // 正转/停：AIN1=1 AIN2=0
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else {                                       // 反转：AIN1=0 AIN2=1
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }

    //反馈：实际转速给 can_send_task 发反馈帧 
    motor_current_speed = (int16_t)rpm;

    //堵转检测：500ms 计数域累计（粒度 2.7 RPM/计数，低速不误报）
    acc_cnt += (uint16_t)(diff < 0 ? -diff : diff);
    if (HAL_GetTick() - acc_tick >= 500)
    {
      if (target != 0 && acc_cnt < 3)
        motor_fault = 1;                           // 堵转：有指令但几乎没动
      else if (target == 0)
        motor_fault = 0;                           // 目标归零自动恢复
      acc_cnt = 0;                                   //  窗口复位
      acc_tick = HAL_GetTick();
    }

    if (target != 0 && acc_cnt >= 3)  motor_online = 1;   // 有指令且动了  在线
    else if (motor_fault)             motor_online = 0;   // 堵转  不在线
    // target==0：保持原状

    osDelay(10);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartCanRecvTask */
/**
* @brief Function implementing the can_recv_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRecvTask */
void StartCanRecvTask(void const * argument)
{
  /* USER CODE BEGIN StartCanRecvTask */
  uint8_t rx_data[8];//队列里取出的原始帧
//条件编译的编写，实则和定义变量是很相像的
#ifdef BOARD_GIMBAL
  feedback_frame_t ff;//云台：解析反馈帧
#else
  control_frame_t cf;//底盘：解析控制帧
#endif

  /* Infinite loop */
  for(;;)//任务主循环永不退出
  {
    if (xQueueReceive(can_rx_queue, rx_data, portMAX_DELAY) == pdPASS)//阻塞等待队列，有帧醒来
    {
#ifdef BOARD_GIMBAL
      if (parse_feedback_frame(rx_data, &ff))// 解析：CRC 不过 parse 返回 0，直接丢
      {
        if (ff.version != 1) { /* 版本不符，整帧丢弃 */ }
        else if (ff.number == last_rx_seq) { dup_cnt++; }  // 重复帧：不刷新心跳
        else {
          motor_current_speed = ff.motor_current_speed;  // 存共享变量
          motor_echo = ff.motor_target_speed_echo;              // 回显（诊断对比用）
          chassis_motor_fault = (ff.chassis_state & 0x04) ? 1 : 0;  // 远端电机异常
          if ((uint8_t)(ff.number - last_rx_seq) > 1) drop_cnt++;  // 跳变：丢帧计数
          last_rx_seq = ff.number;                              // 记录本次序号
          last_rx_tick = HAL_GetTick(); // 心跳的时间戳，用来算超时
        }
      }
#else
      if (parse_control_frame(rx_data, &cf))//依旧是解析校验crc
      {
        if (cf.version != 1) { /* 版本不符，整帧丢弃 */ }
        else if (cf.number == last_rx_seq) { dup_cnt++; }  // 重复帧：不刷新心跳
        else {
          motor_target_speed = cf.motor_target_speed;
          servo_target_speed = cf.servo_target_speed;
          servo_online = (cf.gimbal_state & 0x01) ? 1 : 0;   // 位0：舵机在线
          if ((uint8_t)(cf.number - last_rx_seq) > 1) drop_cnt++;  // 跳变：丢帧计数
          last_rx_seq = cf.number;                              // 记录本次序号
          last_rx_tick = HAL_GetTick();//依旧同上
        }
      }
#endif
    }
  }
  /* USER CODE END StartCanRecvTask */
}

/* USER CODE BEGIN Header_StartCanSendTask */
/**
* @brief Function implementing the can_send_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanSendTask */
void StartCanSendTask(void const * argument)
{
  /* USER CODE BEGIN StartCanSendTask */
  uint8_t tx_data[8];
  feedback_frame_t ff = {0};
  static uint8_t tx_seq = 0;              // 序号计数器：函数内 static，只初始化一次

  /* Infinite loop */
  for(;;)
  {
    ff.motor_target_speed_echo = motor_target_speed;
    ff.motor_current_speed = motor_current_speed;
    ff.version = 1;                       // 协议版本号
    ff.chassis_state = 0;
    ff.chassis_state |= motor_online ? 0x01 : 0x00;  // 位0：电机在线
    ff.chassis_state |= can_comm_ok ? 0x02 : 0x00;   // 位1：板间通信
    ff.chassis_state |= motor_fault ? 0x04 : 0x00;   // 位2：电机异常
    ff.number = tx_seq++;                 // ① 先填序号（自然回绕）

    pack_feedback_frame(&ff, tx_data);

    can_bus_send(CAN_FB_FRAME_ID, tx_data);



    osDelay(10);
  }
  /* USER CODE END StartCanSendTask */
}

/* USER CODE BEGIN Header_StartHealthTask */
/**
* @brief Function implementing the health_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHealthTask */
void StartHealthTask(void const * argument)
{
  /* USER CODE BEGIN StartHealthTask */
  /* Infinite loop */
  for(;;)
  {
    if (HAL_GetTick() - last_rx_tick > 100)  // 100ms 没收到有效帧
    {
      can_comm_ok = 0;                     // 板间通信异常
      motor_target_speed = 0;
    }
    else
      can_comm_ok = 1;                     // 正常
    
    osDelay(20);
  }
  /* USER CODE END StartHealthTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  uint8_t duty = 0;        // 当前亮度 0~20
  int8_t  dir = 1;         // 呼吸方向（亮→灭 / 灭→亮）
  uint8_t pwm_cnt = 0;     // 20ms 周期内的位置
  /* Infinite loop */
  for(;;)
  {
#ifdef BOARD_GIMBAL
    uint8_t fault = !can_comm_ok;   // 云台：CAN 通信异常
#else
    uint8_t fault = motor_fault;    // 底盘：电机异常
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
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the display_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  oled_init();
  oled_clear();
  char buf[22];
  /* Infinite loop */
  for(;;)
  {
    //行0：舵机目标转速 + 在线
    sprintf(buf, "SERVO:%5dRPM %s", servo_target_speed, servo_online ? "ON " : "OFF");
    oled_show_string(0, 0, buf);
    //行2：电机目标/实际转速
    sprintf(buf, "MOTOR:%4d/%4d", motor_target_speed, motor_current_speed);
    oled_show_string(2, 0, buf);
    //行4：电机在线 + 异常
    sprintf(buf, "MST:%s FLT:%s", motor_online ? "ON " : "OFF", motor_fault ? "YES" : "NO ");
    oled_show_string(4, 0, buf);
    //行6：板间通信 + 电机任务栈水位（高水位监测：余量越小越危险）
    uint16_t stk = uxTaskGetStackHighWaterMark(motor_taskHandle);
    sprintf(buf, "CAN:%s STK:%3u", can_comm_ok ? "OK " : "ERR", (unsigned)stk);
    oled_show_string(6, 0, buf);
    osDelay(100);                    // 100ms 刷新（人眼够用，省 CPU）
  }
  /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* 栈溢出钩子：系统已不可信，关中断停机并点亮 LED（低电平亮）指示 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   // LED 常亮
    for(;;);
}

/* 内存分配失败钩子：heap 耗尽（heap_4 无碎片回收），同样停机指示 */
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    for(;;);
}

/* USER CODE END Application */

