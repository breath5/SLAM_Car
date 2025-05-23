/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

extern "C" {
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
}
#include "wheel_motor_app.h"
#include "uart_app.h"

#include <iostream>

extern TIM_HandleTypeDef        htim7;
extern TaskHandle_t chassis_task_handle;
 static uint8_t TIM7_Interrupt_Count = 0;
static uint8_t TIM7_Interrupt_ChassisCount = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
// void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void MX_TIM7_Init(void);  // 启动定时器
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
   MX_TIM7_Init();  // 启动定时器
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();   // 初始化GPIO时钟和电机的引脚
  MX_DMA_Init();

  HAL_TIM_Base_MspInit(&htim1);
  MX_TIM1_Init();

  MX_TIM2_Init();
  HAL_TIM_Base_MspInit(&htim2);
  MX_TIM3_Init();
  HAL_TIM_Encoder_MspInit(&htim3);
  MX_TIM4_Init();
  HAL_TIM_Encoder_MspInit(&htim4);
  MX_TIM5_Init();
  HAL_TIM_Encoder_MspInit(&htim5);

  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  HAL_UART_MspInit(&huart1);
  HAL_UART_MspInit(&huart2);

  // /* 添加优先级冲突检查（FreeRTOS最高可管理优先级为5） */
  // printf("configMAX_SYSCALL_INTERRUPT_PRIORITY: %d\n", configMAX_SYSCALL_INTERRUPT_PRIORITY);
  // if (configMAX_SYSCALL_INTERRUPT_PRIORITY > 5) {
  //   printf("TIM7_IRQn: %d\n", TIM7_IRQn);
  //   Error_Handler();
  // }
  // /* 确保中断优先级不高于 FreeRTOS 最高可管理优先级 */
  // configASSERT(0 > configMAX_SYSCALL_INTERRUPT_PRIORITY);

  /* USER CODE BEGIN 2 */

  UARTApp_Init();
  /* USER CODE END 2 */
  FourWheelMotorApp();

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  // MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  HAL_Delay(1000);
    // FourWheelMotorApp();
    // uart.sendLine("UART通信初始化完成!");

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */  
/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM7_Init(void)
{
  /* 1. 基本参数配置 */
  htim7.Instance               = TIM7;
  htim7.Init.Prescaler         = 84 - 1;              // 84MHz/84 = 1MHz
  htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim7.Init.Period            = 1000 - 1;             // 1MHz/1000 = 1kHz → 1ms
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }

  /* 2. NVIC 中断配置 */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* TIM7 中断抢占优先级设为 0（最紧急），可根据系统需求调整 */
  // 在TIM7中断配置处添加检查
  HAL_NVIC_SetPriority(TIM7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  /* 3. 启动带中断的基础定时器 */
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  printf("[NVIC Priority Group] NVIC_PRIORITYGROUP_4\r\n"
         "[IRQ Priorities] TIM7:5(p=5,s=0) | USART1:7 | USART2:6\r\n"
         "[IRQ Priorities] DMA:5 | TIM2/3/4/5:1\r\n");
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
  // 在定时器7中断回调函数中更新每个编码器定时器的计数值
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*
      * 1) 如果你想用 HAL 的延时函数：
      *    在此调用 HAL_IncTick() 以驱动 HAL_Delay()
      */

  if (htim->Instance == TIM7) {   // 定时器7中断, 1ms
    HAL_IncTick();
    // 调用编码器的中断更新函数
    TIM7_Interrupt_Count++;
    // 调用底盘控制的中断更新函数
    TIM7_Interrupt_ChassisCount++;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (TIM7_Interrupt_Count >= 10) {  // 10ms 更新编码器计数
      // 执行定时器7的回调函数
      TIM7_Interrupt_Count = 0;
      FourWheelInterruptCountReset();
    }
    if (TIM7_Interrupt_ChassisCount >= 10) {  //10ms 要与底盘控制的周期一致 0.01s
      TIM7_Interrupt_ChassisCount = 0;
      // 发送任务通知到底盘控制任务
      if (chassis_task_handle != NULL) {
        xTaskNotifyFromISR(chassis_task_handle,
                         1,  // 通知值
                         eSetValueWithOverwrite,
                         &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }

  }

  /*
      * 2) 如果你用 CMSIS-RTOS v2 (FreeRTOS) 作为底层：
      *    在此调用 osSystickHandler() 以进给 RTOS 时基
      *    （一般在 freertos.c 的 SysTick_Handler 中已调用，无需重复）
      */
  // osSystickHandler();
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    __disable_irq();
    printf("System Error! Check task handles and priorities\n");
    while (1) {
        // 添加LED闪烁等可视指示
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

