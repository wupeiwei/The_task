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
#include "tim.h"
#include "gpio.h"
#include "can_bus.h"
#include "state.h"   // 共享状态实例
#include "stdio.h"
#include "oled.h"
#include "PID.h"
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
//共享状态实例
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

/* USER CODE END Variables */
osThreadId motor_taskHandle;
osThreadId can_send_taskHandle;
osThreadId health_taskHandle;
osThreadId led_taskHandle;
osThreadId display_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void Motor_Speed_Update(void);
static void Motor_Statu_Update(void);
static void Motor_PID_Calculate(void);
static void Motor_Output(void);
static void Motor_Fault_Check(void);
static void Can_Feedback_Pack(uint8_t *tx_data);

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void const * argument);
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
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  pid_init(&motor_pid, 0.0f, 0.0f, 0.0f, 1000.0f);

  last_tick = HAL_GetTick();
  acc_tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
    Motor_Speed_Update();
    Motor_Statu_Update();
    Motor_PID_Calculate();
    Motor_Output();
    Motor_Fault_Check();
    osDelay(10);
  }
  /* USER CODE END StartMotorTask */
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
  uint8_t tx_data[8];                     // 打包/发送的桥梁缓冲

  /* Infinite loop */
  for(;;)
  {
    Can_Feedback_Pack(tx_data);
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
    if (HAL_GetTick() - can_link.last_rx_tick > 100)  // 100ms 没收到有效帧
    {
      can_link.can_comm_ok = 0;                     // 板间通信异常
      motor_ctrl.motor_target_speed = 0;
    }
    else
      can_link.can_comm_ok = 1;                     // 正常
    
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
  int8_t  dir = 1;         // 呼吸方向
  uint8_t pwm_cnt = 0;     // 20ms 周期内的位置
  /* Infinite loop */
  for(;;)
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
    sprintf(buf, "SERVO:%5dRPM %s", gimbal.servo_target_speed, gimbal.servo_online ? "ON " : "OFF");
    oled_show_string(0, 0, buf);
    //行2：电机目标/实际转速
    sprintf(buf, "MOTOR:%4d/%4d", motor_ctrl.motor_target_speed, motor_ctrl.motor_current_speed);
    oled_show_string(2, 0, buf);
    //行4：电机在线 + 异常
    sprintf(buf, "MST:%s FLT:%s", chassis.motor_online ? "ON " : "OFF", chassis.motor_fault ? "YES" : "NO ");
    oled_show_string(4, 0, buf);
    //行6：板间通信 + 电机任务栈水位（高水位监测：余量越小越危险）
    uint16_t stk = uxTaskGetStackHighWaterMark(motor_taskHandle);
    sprintf(buf, "CAN:%s STK:%3u", can_link.can_comm_ok ? "OK " : "ERR", (unsigned)stk);
    oled_show_string(6, 0, buf);
    osDelay(100);                    // 100ms 刷新（人眼够用，省 CPU）
  }
  /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// 栈溢出钩子
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


static void Motor_Speed_Update(void)
{
  //测速
  uint16_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
  diff = (int16_t)(cnt - last_cnt);      // int16 相减，溢出自动回绕
  last_cnt = cnt;
  uint32_t now = HAL_GetTick();
  uint32_t dt_ms = now - last_tick;   // 无符号差值
  last_tick = now;

  if (dt_ms == 0) dt_ms = 1;          // 防除零

  // 换算
  rpm = diff * 60000 / (44 * (int32_t)dt_ms);

  //反馈实际转速
  motor_ctrl.motor_current_speed = rpm;
}
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
static void Motor_PID_Calculate(void)
{
  pid_calc(&motor_pid, (float)target, (float)rpm);
}
static void Motor_Output(void)
{
    int32_t pwm = (int32_t)(motor_pid.u * 3599 / 1000);
    if (pwm < 0) pwm = -pwm;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm);

    if (motor_pid.u >= 0)
    { // 正转/停：AIN1=1 AIN2=0
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    } else
    { // 反转：AIN1=0 AIN2=1
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
    }
}
static void Motor_Fault_Check(void)
{
  //堵转检测：500ms 计数域累计（粒度 2.7 RPM/计数，低速不误报）
  acc_cnt += (uint16_t)(diff < 0 ? -diff : diff);
  if (HAL_GetTick() - acc_tick >= 500)
  {
    if (target != 0 && acc_cnt < 3)
    {
      chassis.motor_fault = 1;                           // 堵转：有指令但几乎没动
    }
    else if (target == 0)
    {
       chassis.motor_fault = 0;                           // 目标归零自动恢复
    }

    if (target != 0 && acc_cnt >= 3)  chassis.motor_online = 1;   // 有指令且动了  在线
    else                              chassis.motor_online = 0;   // 堵转  不在线

    acc_cnt = 0;                                   //  窗口复位
    acc_tick = HAL_GetTick();
  }

}
static void Can_Feedback_Pack(uint8_t *tx_data)
{
  feedback_frame_t ff = {0};             // 局部：每周期重新填（原来是任务局部，挪进来）
  static uint8_t tx_seq = 0;             // 序号计数器：函数内 static，只初始化一次
  ff.motor_target_speed_echo = motor_ctrl.motor_target_speed;
  ff.motor_current_speed = motor_ctrl.motor_current_speed;
  ff.version = 1;
  ff.chassis_state = 0;
  ff.chassis_state |= chassis.motor_online ? 0x01 : 0x00;
  ff.chassis_state |= can_link.can_comm_ok ? 0x02 : 0x00;
  ff.chassis_state |= chassis.motor_fault ? 0x04 : 0x00;
  ff.number = tx_seq++;                  // tx_seq 保持函数内 static（Can_Feedback_Pack 里）
  pack_feedback_frame(&ff, tx_data);
}


/* USER CODE END Application */

