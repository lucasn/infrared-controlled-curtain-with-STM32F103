/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dwt_stm32_delay.h"
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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define IN1_PIN GPIO_PIN_6
#define IN1_PORT GPIOB
#define IN2_PIN GPIO_PIN_5
#define IN2_PORT GPIOB
#define IN3_PIN GPIO_PIN_4
#define IN3_PORT GPIOB
#define IN4_PIN GPIO_PIN_3
#define IN4_PORT GPIOB


char fullDriveTable[4][4] = {{1, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {1, 0, 0, 1}};

size_t tableSize = 4;

void stepFullDrive(int steps, uint16_t delay){
	for (int s = 0; s < steps; s++){
		for (int i = 0; i < tableSize; i++){
			HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, fullDriveTable[i][0]);
			HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, fullDriveTable[i][1]);
			HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, fullDriveTable[i][2]);
			HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, fullDriveTable[i][3]);
			DWT_Delay_us(delay);
		}
	}
}

void stepFullDriveReverse(int steps, uint16_t delay){
	for (int s = 0; s < steps; s++){
			for (int i = tableSize - 1; i >= 0; i--){
				HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, fullDriveTable[i][0]);
				HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, fullDriveTable[i][1]);
				HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, fullDriveTable[i][2]);
				HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, fullDriveTable[i][3]);
				DWT_Delay_us(delay);
			}
		}
}


uint8_t count = 0;

uint32_t receiveData(void){
	uint32_t code = 0;

	int timeout = 50000;

	__HAL_TIM_SET_COUNTER(&htim1, 0);

	// waiting for the HIGH signal
	while(!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) && __HAL_TIM_GET_COUNTER(&htim1) < timeout);

	__HAL_TIM_SET_COUNTER(&htim1, 0);

	// waiting for the LOW signal
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) && __HAL_TIM_GET_COUNTER(&htim1) < timeout);

	// receiving the data,
	// 562.5us interval -> 0, 1.6ms interval -> 1
	for(int i = 0; i < 32; i++){
		count = 0;
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		// waiting for the HIGH signal
		while(!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) && __HAL_TIM_GET_COUNTER(&htim1) < timeout);

		__HAL_TIM_SET_COUNTER(&htim1, 0);
		// count the interval
		while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) && __HAL_TIM_GET_COUNTER(&htim1) < timeout){
			count++;
			DWT_Delay_us(100);
		}

		// check which bit should be assigned
		if(count > 12){
			// write 1
			code |= (1UL << (31-i));
		} else {
			//write 0
			code &= ~(1UL << (31-i));
		}
	}
	return code;
}

int checkForRepeat(int time){
	DWT_Delay_us(time);

	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
		if (__HAL_TIM_GET_COUNTER(&htim1) > time){
			return 0;
		}
	}
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
		if (__HAL_TIM_GET_COUNTER(&htim1) > 12000){
			return 0;
		}
	}
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
		if (__HAL_TIM_GET_COUNTER(&htim1) > 3000){
			return 0;
		}
	}
	return 1;
}


int convertCode(uint32_t code){
	switch (code){
		case (0xFF30CF):
			return 1;
			break;

		case (0xFF18E7):
			return 2;
			break;

		default :
			return 0;
			break;
	}
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim1);
  DWT_Delay_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int buttonCounterClockWise, buttonClockWise;
  uint32_t data;
  int convertedData = 0;
  int shouldRepeat;
  while (1)
  {

	// infra-red control logic
	if (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)){
		data = receiveData();

		convertedData = convertCode(data);

		if (convertedData == 1){
			shouldRepeat = checkForRepeat(40000);
			while (shouldRepeat){
				stepFullDrive(5, 3000);
				shouldRepeat = checkForRepeat(30000);
			}

			convertedData = 0;
			HAL_Delay(100);
		}
		else if (convertedData == 2){
			shouldRepeat = checkForRepeat(40000);
			while (shouldRepeat){
				stepFullDriveReverse(5, 3000);
				shouldRepeat = checkForRepeat(30000);
			}
			convertedData = 0;
			HAL_Delay(100);
		}
	}

	// button control logic
	buttonCounterClockWise = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	buttonClockWise = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	while(!buttonCounterClockWise){
	  stepFullDrive(1, 3500);
	  buttonCounterClockWise = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	}

	while(!buttonClockWise){
		stepFullDriveReverse(1, 3500);
		buttonClockWise = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	}

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

