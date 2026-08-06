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

  /* Infinite loop */
  for(;;)
  {
    //测速：10ms 窗口计数差 → RPM（44 计数/圈）
    uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
    int16_t diff = (int16_t)(cnt - last_cnt);      // int16 相减，溢出自动回绕
    last_cnt = cnt;
    int32_t rpm = diff * 6000 / 44;                // diff/44圈 ÷ 0.01s × 60

    //增量式 PID
    float err = (float)(motor_target_speed - rpm); // 目标 − 实际
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
        motor_current_speed = ff.motor_current_speed;  // 存共享变量
        last_rx_tick = HAL_GetTick(); // 心跳的时间戳，用来算超时
      }
#else
      if (parse_control_frame(rx_data, &cf))//依旧是解析校验crc
      {
        motor_target_speed = cf.motor_target_speed;
        servo_target_speed = cf.servo_target_speed;
        last_rx_tick = HAL_GetTick();//依旧同上
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
      can_comm_ok = 0;                     // 板间通信异常
    else
      can_comm_ok = 1;                     // 正常

    /* 电机异常：目标≠0 且实际≈0 持续 500ms（20ms×25次） */
    static uint8_t motor_fault_cnt;
    if (motor_target_speed != 0 && (motor_current_speed > -30 && motor_current_speed < 30))
      motor_fault_cnt++;
    else
      motor_fault_cnt = 0;
    if (motor_fault_cnt > 25) motor_fault = 1;
    else if (motor_target_speed == 0) motor_fault = 0;   // 目标归零自动恢复
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

