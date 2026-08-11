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
#include "adc.h"
#include "can_bus.h"
#include "state.h"   // 共享状态实例（成员级 volatile，单成员原子访问）
#include "tim.h"
#include "dma.h"
extern DMA_HandleTypeDef hdma_adc1;
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
//共享状态实例（类型定义在 state.h，成员级 volatile）
can_link_state_t can_link = {0};
motor_control_t  motor_ctrl = {0};
chassis_state_t  chassis = {0};
gimbal_state_t   gimbal = {0};
volatile uint16_t adc_buf[2] = {2048, 2048};   // 板级私有：DMA 持续写入的摇杆原始值（CH0/CH1）
/* USER CODE END Variables */
osThreadId input_taskHandle;
osThreadId can_send_taskHandle;
osThreadId health_taskHandle;
osThreadId led_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void Input_Init(void);
static void Input_read_and_map(void);
static void servo_output(void);
static void Can_control_Pack(uint8_t *tx_data);
/* USER CODE END FunctionPrototypes */

void StartInputTask(void const * argument);
void StartCanSendTask(void const * argument);
void StartHealthTask(void const * argument);
void StartLedTask(void const * argument);


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
  /* definition and creation of input_task */
  osThreadDef(input_task, StartInputTask, osPriorityNormal, 0, 128);
  input_taskHandle = osThreadCreate(osThread(input_task), NULL);

  /* definition and creation of can_send_task */
  osThreadDef(can_send_task, StartCanSendTask, osPriorityBelowNormal, 0, 128);
  can_send_taskHandle = osThreadCreate(osThread(can_send_task), NULL);

  /* definition and creation of health_task */
  osThreadDef(health_task, StartHealthTask, osPriorityLow, 0, 128);
  health_taskHandle = osThreadCreate(osThread(health_task), NULL);

  /* definition and creation of led_task */
  osThreadDef(led_task, StartLedTask, osPriorityLow, 0, 128);
  led_taskHandle = osThreadCreate(osThread(led_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  Input_Init();
  gimbal.servo_online = 1;       //舵机在线
  /* Infinite loop */
  for(;;)
  {
    Input_read_and_map();
    servo_output();
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
  /* Infinite loop */
  for(;;)
  {
    Can_control_Pack(tx_data);
    can_bus_send(CAN_CTRL_FRAME_ID, tx_data);
    osDelay(5);                         // 5ms 周期
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
    if (HAL_GetTick() - can_link.last_rx_tick > 100)  // 100ms 没收到有效帧
      can_link.can_comm_ok = 0;                     //  板间通信异常
    else
      can_link.can_comm_ok = 1;                     //  正常
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
    uint8_t fault = !can_link.can_comm_ok;   // 云台：CAN 通信异常
#else
    uint8_t fault = chassis.motor_fault;    // 底盘：电机异常
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

//栈溢出钩子
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);   // LED 常亮
    for(;;);
}
//内存分配失败钩子
void vApplicationMallocFailedHook(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    for(;;);
}

static void Input_Init(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 2);   // 启动 DMA 采样（只启动这一次）
  osDelay(5);
  __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);   // DMA 静默搬运，任务轮询读
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // 启动舵机 PWM50Hz
}
static void Input_read_and_map(void)
{
  // 读原始值：CH0=Y轴（电机），CH1=X轴（舵机）——按实际接线调
  int32_t y_raw = adc_buf[0];
  int32_t x_raw = adc_buf[1];

  //摇杆回中 = 2048，减去得到偏差（-2048~+2048）设置回中为0点
  y_raw -= 2048;
  if (y_raw > -50 && y_raw < 50) y_raw = 0;            // 死区：防摇杆抖动漂移
  motor_ctrl.motor_target_speed = (int16_t)(y_raw * 1000 / 2048); // 映射 ±1000 RPM

  x_raw -= 2048;
  if (x_raw > -50 && x_raw < 50) x_raw = 0;
  gimbal.servo_target_speed = (int16_t)(x_raw * 1000 / 2048); // 舵机目标转速
}
static void servo_output(void)
{
  int32_t pulse = 1500 + gimbal.servo_target_speed / 2;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pulse);
}
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


/* USER CODE END Application */

