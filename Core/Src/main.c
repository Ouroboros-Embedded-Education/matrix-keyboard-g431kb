/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matrix_keyboard.h"
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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

static mk_gpio_t mkColumns[] = {
	{
		.GPIO = (uint32_t)KEY_C1_GPIO_Port,
		.pin = (uint32_t)KEY_C1_Pin
	},
	{
		.GPIO = (uint32_t)KEY_C2_GPIO_Port,
		.pin = (uint32_t)KEY_C2_Pin
	},
	{
		.GPIO = (uint32_t)KEY_C3_GPIO_Port,
		.pin = (uint32_t)KEY_C3_Pin
	},
	{
		.GPIO = (uint32_t)KEY_C4_GPIO_Port,
		.pin = (uint32_t)KEY_C4_Pin
	}
};
static mk_gpio_t mkRows[] = {
	{
		.GPIO = (uint32_t)KEY_R1_GPIO_Port,
		.pin = (uint32_t)KEY_R1_Pin
	},
	{
		.GPIO = (uint32_t)KEY_R2_GPIO_Port,
		.pin = (uint32_t)KEY_R2_Pin
	},
	{
		.GPIO = (uint32_t)KEY_R3_GPIO_Port,
		.pin = (uint32_t)KEY_R3_Pin
	},
	{
		.GPIO = (uint32_t)KEY_R4_GPIO_Port,
		.pin = (uint32_t)KEY_R4_Pin
	}
};

mk_t Key4x4;

uint32_t PCol, PRow;
uint8_t KeyFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// In this example, the Timer 4 was configured to trigger the interrupt
// every 10 ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == TIM2){
		mk_DoOperation(&Key4x4, NULL);
	}
}

// The callback of API
void mk_callback(mk_t *Mk, mk_event_e ev, uint32_t PressedCol, uint32_t PressedRow){
    if (ev == MK_EVENT_PRESSED){
        // load the PressedCol and PressedRow on you global variables and
        // inform the main that a PRESSED event was occoured
    	PCol = PressedCol;
    	PRow = PressedRow;
    }
    else if (ev == MK_EVENT_RELEASED){
        // inform the main that the key was released
    	PCol = 0;
    	PRow = 0;
    }
    KeyFlag = 1;
}


/*
 * Initialize Modules
 */

void _init_keypad(){
	Key4x4.actLevel = MK_ACTIVE_LEVEL_LOW;
	Key4x4.eventMask = MK_EVENT_ALL;
	Key4x4.nCols = 4;
	Key4x4.nRows = 4;
	Key4x4.gCols = mkColumns;
	Key4x4.gRows = mkRows;

	if (mk_init(&Key4x4) != MK_STATUS_OK){
		Error_Handler();
	}

}

void __init_display(){
	Lcd.columns = 16;
	Lcd.rows = 2;
	Lcd.font = LCD_FONT_5X8;
	Lcd.interface = LCD_INTERFACE_4BIT;
	Lcd.gpios[LCD_RS].GPIO = (uint32_t)LCD_RS_GPIO_Port;
	Lcd.gpios[LCD_RS].pin = LCD_RS_Pin;
	Lcd.gpios[LCD_E].GPIO = (uint32_t)LCD_E_GPIO_Port;
	Lcd.gpios[LCD_E].pin = LCD_E_Pin;
	Lcd.gpios[LCD_D4].GPIO = (uint32_t)LCD_D4_GPIO_Port;
	Lcd.gpios[LCD_D4].pin = LCD_D4_Pin;
	Lcd.gpios[LCD_D5].GPIO = (uint32_t)LCD_D5_GPIO_Port;
	Lcd.gpios[LCD_D5].pin = LCD_D5_Pin;
	Lcd.gpios[LCD_D6].GPIO = (uint32_t)LCD_D6_GPIO_Port;
	Lcd.gpios[LCD_D6].pin = LCD_D6_Pin;
	Lcd.gpios[LCD_D7].GPIO = (uint32_t)LCD_D7_GPIO_Port;
	Lcd.gpios[LCD_D7].pin = LCD_D7_Pin;

	lcd_init(&Lcd);

	lcd_clear_all(&Lcd);

	lcd_send_string(&Lcd, ">> STM32G431KB");
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  _init_keypad();
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 149;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, KEY_C1_Pin|KEY_C2_Pin|KEY_C4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KEY_C3_GPIO_Port, KEY_C3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_D5_Pin|LCD_D4_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RS_Pin|LCD_E_Pin|LCD_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : KEY_C1_Pin KEY_C2_Pin KEY_C4_Pin */
  GPIO_InitStruct.Pin = KEY_C1_Pin|KEY_C2_Pin|KEY_C4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_R1_Pin KEY_R3_Pin KEY_R4_Pin */
  GPIO_InitStruct.Pin = KEY_R1_Pin|KEY_R3_Pin|KEY_R4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_C3_Pin */
  GPIO_InitStruct.Pin = KEY_C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(KEY_C3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D5_Pin LCD_D4_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_D5_Pin|LCD_D4_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_R2_Pin */
  GPIO_InitStruct.Pin = KEY_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_R2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_D6_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
