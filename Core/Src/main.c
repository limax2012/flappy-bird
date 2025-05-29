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
#include "math.h"
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SSD1306_I2C_ADDR 0x3C
#define BUTTON_PIN GPIO_PIN_11
#define DEBOUNCE_DELAY 50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

/* Definitions for GameTask */
osThreadId_t GameTaskHandle;
const osThreadAttr_t GameTask_attributes = { .name = "GameTask", .stack_size =
    256 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for InputTask */
osThreadId_t InputTaskHandle;
const osThreadAttr_t InputTask_attributes = { .name = "InputTask", .stack_size =
    128 * 4, .priority = (osPriority_t) osPriorityAboveNormal, };
/* Definitions for RenderTask */
osThreadId_t RenderTaskHandle;
const osThreadAttr_t RenderTask_attributes = { .name = "RenderTask",
    .stack_size = 512 * 4, .priority = (osPriority_t) osPriorityBelowNormal, };
/* Definitions for ButtonSemaphore */
osSemaphoreId_t ButtonSemaphoreHandle;
const osSemaphoreAttr_t ButtonSemaphore_attributes =
    { .name = "ButtonSemaphore" };
/* USER CODE BEGIN PV */

const int BIRD_W = 4, BIRD_H = 4;
const float ACCEL = 0.075f, JUMP_VEL = -2.0f;
const int MAX_POS = 127 - BIRD_W;

float vel = JUMP_VEL, pos = 64.0f;
int last_pos = -1000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
void StartGameTask(void *argument);
void StartInputTask(void *argument);
void StartRenderTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void OLED_Init() {
  uint8_t cmds[] = { 0x00,  // Control byte for commands
      0xAE,        // Display OFF
      0xD5, 0x80,  // Set display clock divide ratio/oscillator frequency
      0xA8, 0x3F,  // Set multiplex ratio (height - 1)
      0xD3, 0x00,  // Set display offset
      0x40,        // Set start line to 0
      0x8D, 0x14,  // Enable charge pump
      0x20, 0x00,  // Set memory addressing mode to horizontal
      0xA1,        // Set segment re-map (mirror horizontally)
      0xC8,        // Set COM output scan direction (remapped mode)
      0xDA, 0x12,  // Set COM pins hardware configuration
      0x81, 0x7F,  // Set contrast control
      0xD9, 0xF1,  // Set pre-charge period
      0xDB, 0x40,  // Set VCOMH deselect level
      0xA4,        // Resume display from RAM content
      0xA6,        // Set normal display (not inverted)
      0xAF         // Display ON
      };

  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, cmds, sizeof(cmds),
      100);

  HAL_Delay(100);
}

void OLED_Clear() {
  uint8_t horizontal_mode[] = { 0x00, 0x20, 0x00 };
  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, horizontal_mode, 3,
      100);

  for (uint8_t page = 0; page < 8; page++) {
    uint8_t set_page_col[] = { 0x00, 0xB0 | page, 0x00, 0x10 };
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, set_page_col, 4,
        100);

    uint8_t clear[129] = { 0x40 };
    memset(clear + 1, 0x00, 128);
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, clear, sizeof(clear),
        100);
  }
}

void OLED_Draw_Floor() {
  uint8_t vertical_mode[] = { 0x00, 0x20, 0x01 };
  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, vertical_mode, 3, 100);

  uint8_t set_column[] = { 0x00, 0x21, 127, 127 };
  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, set_column, 4, 100);

  uint8_t set_page[] = { 0x00, 0x22, 0x00, 0x07 };
  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, set_page, 4, 100);

  uint8_t data[] = { 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, data, 9, 100);

  uint8_t horizontal_mode[] = { 0x00, 0x20, 0x00 };
  HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, horizontal_mode, 3,
      100);
}

void Update_Kinematics() {
  pos += (vel += ACCEL);
  if (pos > MAX_POS)
    pos = MAX_POS;
}

void OLED_Update_Bird(float new_pos) {
  int new_top = (int) floorf(new_pos);
  if (new_top == last_pos)
    return;

  int old_top = last_pos;
  int min_y = (new_top < old_top) ? new_top : old_top;
  int max_y =
      ((new_top + BIRD_W) > (old_top + BIRD_W)) ?
          (new_top + BIRD_W) : (old_top + BIRD_W);

  for (int y = min_y; y < max_y; y++) {
    bool was = (y >= old_top && y < old_top + BIRD_W);
    bool now = (y >= new_top && y < new_top + BIRD_W);
    if (was == now || y < 0 || y > 127)
      continue;

    uint8_t cmds[] = { 0x00, 0xB7, y & 0x0F, 0x10 + (y >> 4) };
    uint8_t data[] = { 0x40, now ? ((1 << BIRD_H) - 1) : 0x00 };

    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, cmds, 4, 100);
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR << 1, data, 2, 100);
  }

  last_pos = new_top;
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  OLED_Init();
  OLED_Clear();
  OLED_Draw_Floor();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ButtonSemaphore */
  ButtonSemaphoreHandle = osSemaphoreNew(1, 1, &ButtonSemaphore_attributes);

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
  /* creation of GameTask */
  GameTaskHandle = osThreadNew(StartGameTask, NULL, &GameTask_attributes);

  /* creation of InputTask */
  InputTaskHandle = osThreadNew(StartInputTask, NULL, &InputTask_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == BUTTON_PIN) {
    osSemaphoreRelease(ButtonSemaphoreHandle);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartGameTask */
/**
 * @brief  Function implementing the GameTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGameTask */
void StartGameTask(void *argument) {
  /* USER CODE BEGIN 5 */
  TickType_t last_wake = xTaskGetTickCount();

  /* Infinite loop */
  for (;;) {
    Update_Kinematics();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
 * @brief Function implementing the InputTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument) {
  /* USER CODE BEGIN StartInputTask */
  bool input_armed = true;

  /* Infinite loop */
  for (;;) {
    if (osSemaphoreAcquire(ButtonSemaphoreHandle, osWaitForever) == osOK) {
      if (input_armed
          && HAL_GPIO_ReadPin(GPIOB, BUTTON_PIN) == GPIO_PIN_RESET) {
        vel = JUMP_VEL;
        input_armed = false;

        while (HAL_GPIO_ReadPin(GPIOB, BUTTON_PIN) == GPIO_PIN_RESET) {
          osDelay(1);
        }

        osDelay(DEBOUNCE_DELAY);

        input_armed = true;
      }
    }
  }
  /* USER CODE END StartInputTask */
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
    OLED_Update_Bird(pos);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
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
