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
#include "adc.h"          // hadc1 句柄声明在这（adc.c 里定义）
#include "can_bus.h"      // can_bus_send 声明 + can_rx_queue extern
#include "tim.h"          // htim1、HAL_TIM_PWM_Start
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
volatile uint16_t adc_buf[2];   // DMA 持续写入的摇杆原始值（CH0/CH1）
volatile uint8_t servo_online = 0;   // 舵机在线（云台）
volatile uint8_t motor_online = 0;   // 电机在线（底盘）
volatile uint8_t motor_fault  = 0;   // 电机异常（底盘）
/* USER CODE END Variables */
osThreadId can_recv_taskHandle;
osThreadId input_taskHandle;
osThreadId can_send_taskHandle;
osThreadId health_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCanRecvTask(void const * argument);
void StartInputTask(void const * argument);
void StartCanSendTask(void const * argument);
void StartHealthTask(void const * argument);

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
  /* definition and creation of can_recv_task */
  osThreadDef(can_recv_task, StartCanRecvTask, osPriorityAboveNormal, 0, 256);
  can_recv_taskHandle = osThreadCreate(osThread(can_recv_task), NULL);

  /* definition and creation of input_task */
  osThreadDef(input_task, StartInputTask, osPriorityNormal, 0, 128);
  input_taskHandle = osThreadCreate(osThread(input_task), NULL);

  /* definition and creation of can_send_task */
  osThreadDef(can_send_task, StartCanSendTask, osPriorityBelowNormal, 0, 128);
  can_send_taskHandle = osThreadCreate(osThread(can_send_task), NULL);

  /* definition and creation of health_task */
  osThreadDef(health_task, StartHealthTask, osPriorityLow, 0, 128);
  health_taskHandle = osThreadCreate(osThread(health_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartCanRecvTask */
/**
  * @brief  Function implementing the can_recv_task thread.
  * @param  argument: Not used
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
  for(;;)
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

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the input_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void const * argument)
{
  /* USER CODE BEGIN StartInputTask */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);   // 启动 DMA 采样（只启动这一次）
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // 启动舵机 PWM（50Hz）
  servo_online = 1;       //舵机在线
  /* Infinite loop */
  for(;;)
  {
    // 读原始值：CH0=Y轴（电机），CH1=X轴（舵机）——按实际接线调
    int32_t y_raw = adc_buf[0];
    int32_t x_raw = adc_buf[1];

    //摇杆回中 = 2048，减去得到偏差（-2048~+2048）设置回中为0点
    y_raw -= 2048;
    if (y_raw > -50 && y_raw < 50) y_raw = 0;            // 死区：防摇杆抖动漂移
    motor_target_speed = (int16_t)(y_raw * 1000 / 2048); // 映射 ±1000 RPM

    x_raw -= 2048;
    if (x_raw > -50 && x_raw < 50) x_raw = 0;
    servo_target_speed = (int16_t)(x_raw * 1000 / 2048); // 舵机目标转速
    int32_t pulse = 1500 + servo_target_speed / 2;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pulse);
    osDelay(10);
  }
  /* USER CODE END StartInputTask */
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
  control_frame_t cf = {0};

  /* Infinite loop */
  for(;;)
  {
    /* 1. 读共享变量，填协议结构体 */
    cf.motor_target_speed = motor_target_speed;        //  摇杆指令
    cf.servo_target_speed = servo_target_speed;        //  舵机指令（同上）
    cf.version = 1; //协议版本
    cf.gimbal_state = 0;
    cf.gimbal_state |= servo_online ? 0x01 : 0x00;   // 位0：舵机在线
    cf.gimbal_state |= can_comm_ok ? 0x02 : 0x00;    // 位1：板间通信
    /* 2. pack 成 8 字节 */
    pack_control_frame(&cf, tx_data);
    /* 3. 发出去 */
    can_bus_send(CAN_CTRL_FRAME_ID, tx_data);
    osDelay(5);                         // 5ms 周期（协议定的）
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
      can_comm_ok = 0;                     //  板间通信异常
    else
      can_comm_ok = 1;                     //  正常
    osDelay(20);
  }
  /* USER CODE END StartHealthTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

