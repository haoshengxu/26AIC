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
#include "vofa_debug.h"
#include "vision_command.h"
#include "usart.h"
#include "show.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
volatile UBaseType_t g_vofa_stack_free_words;
volatile UBaseType_t g_display_stack_free_words;
const char * volatile g_rtos_assert_file;
volatile unsigned long g_rtos_assert_line;
/* USER CODE END Variables */
/* Definitions for vofaTask */
osThreadId_t vofaTaskHandle;
uint32_t vofaTaskBuffer[ 768 ];
osStaticThreadDef_t vofaTaskControlBlock;
const osThreadAttr_t vofaTask_attributes = {
  .name = "vofaTask",
  .cb_mem = &vofaTaskControlBlock,
  .cb_size = sizeof(vofaTaskControlBlock),
  .stack_mem = &vofaTaskBuffer[0],
  .stack_size = sizeof(vofaTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
uint32_t displayTaskBuffer[ 768 ];
osStaticThreadDef_t displayTaskControlBlock;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .cb_mem = &displayTaskControlBlock,
  .cb_size = sizeof(displayTaskControlBlock),
  .stack_mem = &displayTaskBuffer[0],
  .stack_size = sizeof(displayTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartVofaTask(void *argument);
void StartDisplayTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  AppRTOS_AssertFailed((const char *)pcTaskName, 0U);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
  AppRTOS_AssertFailed("RTOS heap", 0U);
}
/* USER CODE END 5 */

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
  /* creation of vofaTask */
  vofaTaskHandle = osThreadNew(StartVofaTask, NULL, &vofaTask_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (vofaTaskHandle == NULL || displayTaskHandle == NULL) Error_Handler();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartVofaTask */
/**
  * @brief  Function implementing the vofaTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartVofaTask */
void StartVofaTask(void *argument)
{
  /* USER CODE BEGIN StartVofaTask */
  (void)argument;
  for (;;) {
    {
      static uint8_t lane_frame[9], lane_pending, lane_restore;
      static uint32_t lane_sent_ms;
      if(lane_restore && (uint32_t)(HAL_GetTick()-lane_sent_ms)>=2500U) {
        Vision_RequestLane(0U);lane_restore=0;
      }
      if(!lane_pending) lane_pending=Vision_TakeLaneCommand(lane_frame);
      if(lane_pending && HAL_UART_Transmit(&huart1,lane_frame,9,2)==HAL_OK) {
        lane_restore=(lane_frame[6]!=0U);lane_sent_ms=HAL_GetTick();
        lane_pending=0;
      }
    }
    Vofa_Task();
    g_vofa_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
    osDelay(2);
  }
  /* USER CODE END StartVofaTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  (void)argument;
  for (;;) {
    show_task();
    g_display_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
    osDelay(10);
  }
  /* USER CODE END StartDisplayTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void AppRTOS_AssertFailed(const char *file, unsigned long line)
{
  g_rtos_assert_file = file;
  g_rtos_assert_line = line;
  Error_Handler();
}
/* USER CODE END Application */

