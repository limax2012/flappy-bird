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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "bird.h"
#include "oled.h"
#include "pipe_queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define JUMP_BUTTON_PORT GPIOA
#define JUMP_BUTTON_PIN GPIO_PIN_8
#define DEBOUNCE_DELAY 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

/* Definitions for JumpButtonTask */
osThreadId_t JumpButtonTaskHandle;
const osThreadAttr_t JumpButtonTask_attributes = { .name = "JumpButtonTask",
    .stack_size = 128 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for GameTask */
osThreadId_t GameTaskHandle;
const osThreadAttr_t GameTask_attributes = { .name = "GameTask", .stack_size =
    256 * 4, .priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for RenderTask */
osThreadId_t RenderTaskHandle;
const osThreadAttr_t RenderTask_attributes = { .name = "RenderTask",
    .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for BirdPosMutex */
osMutexId_t BirdPosMutexHandle;
const osMutexAttr_t BirdPosMutex_attributes = { .name = "BirdPosMutex" };
/* Definitions for PipeQueueMutex */
osMutexId_t PipeQueueMutexHandle;
const osMutexAttr_t PipeQueueMutex_attributes = { .name = "PipeQueueMutex" };
/* Definitions for JumpSemaphore */
osSemaphoreId_t JumpSemaphoreHandle;
const osSemaphoreAttr_t JumpSemaphore_attributes = { .name = "JumpSemaphore" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
void StartJumpButtonTask(void *argument);
void StartGameTask(void *argument);
void StartRenderTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void reset_game(void) {
  bird_reset_pos();
  bird_reset_vel();
  pq_clear();
  pq_enqueue();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  oled_init(&hi2c2);
  oled_flush_fb(&hi2c2);
  pq_init();
  pq_enqueue();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of BirdPosMutex */
  BirdPosMutexHandle = osMutexNew(&BirdPosMutex_attributes);

  /* creation of PipeQueueMutex */
  PipeQueueMutexHandle = osMutexNew(&PipeQueueMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of JumpSemaphore */
  JumpSemaphoreHandle = osSemaphoreNew(1, 0, &JumpSemaphore_attributes);

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
  /* creation of JumpButtonTask */
  JumpButtonTaskHandle = osThreadNew(StartJumpButtonTask, NULL,
      &JumpButtonTask_attributes);

  /* creation of GameTask */
  GameTaskHandle = osThreadNew(StartGameTask, NULL, &GameTask_attributes);

  /* creation of RenderTask */
  RenderTaskHandle = osThreadNew(StartRenderTask, NULL, &RenderTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == JUMP_BUTTON_PIN) {
    osSemaphoreRelease(JumpSemaphoreHandle);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartJumpButtonTask */
/**
 * @brief  Function implementing the JumpButtonTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartJumpButtonTask */
void StartJumpButtonTask(void *argument) {
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for (;;) {
    if (osSemaphoreAcquire(JumpSemaphoreHandle, osWaitForever) == osOK) {
      if (HAL_GPIO_ReadPin(JUMP_BUTTON_PORT, JUMP_BUTTON_PIN)
          == GPIO_PIN_RESET) {

        bird_reset_vel();

        while (HAL_GPIO_ReadPin(JUMP_BUTTON_PORT, JUMP_BUTTON_PIN)
            == GPIO_PIN_RESET) {

          osDelay(1);
        }

        osDelay(DEBOUNCE_DELAY);
      }
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGameTask */
/**
 * @brief  Function implementing the GameTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGameTask */
void StartGameTask(void *argument) {
  /* USER CODE BEGIN StartGameTask */
  TickType_t last_wake = xTaskGetTickCount();

  /* Infinite loop */
  for (;;) {
    if ((osMutexAcquire(BirdPosMutexHandle, osWaitForever) == osOK)
        && (osMutexAcquire(PipeQueueMutexHandle, osWaitForever) == osOK)) {

      bird_update();
      pq_update();

      if (pq_collision(BIRD_POS_X, bird_get_y(), BIRD_W, BIRD_H)
          || (bird_get_y() + BIRD_H > OLED_HEIGHT - 1)) {

        reset_game();
      }

      osMutexRelease(PipeQueueMutexHandle);
      osMutexRelease(BirdPosMutexHandle);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
  }
  /* USER CODE END StartGameTask */
}

/* USER CODE BEGIN Header_StartRenderTask */
/**
 * @brief Function implementing the RenderTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartRenderTask */
void StartRenderTask(void *argument) {
  /* USER CODE BEGIN StartRenderTask */
  TickType_t last_wake = xTaskGetTickCount();

  /* Infinite loop */
  for (;;) {
    fb_clear();
    fb_draw_floor();

    if (osMutexAcquire(BirdPosMutexHandle, osWaitForever) == osOK) {
      bird_draw();
      osMutexRelease(BirdPosMutexHandle);
    }

    if (osMutexAcquire(PipeQueueMutexHandle, osWaitForever) == osOK) {
      pq_draw();
      osMutexRelease(PipeQueueMutexHandle);
    }

    oled_flush_fb(&hi2c2);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(35));
  }
  /* USER CODE END StartRenderTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
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
