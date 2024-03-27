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
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	// Begin clocks for peripherals
	RCC->AHBENR |= (1 << 17); // Enable GPIOA clock
	RCC->AHBENR |= (1 << 19); // Enable GPIOC clock
	RCC->APB2ENR |= (1 << 9); // Enable ADC clock
	RCC->APB1ENR |= (1 << 29); // Enable DAC clock
	
	// LED Config
	GPIOC->MODER |= (0x55 << 12); // 01 01 01 01 << 12 red, blue, orange, green general purpose output
	GPIOC->OTYPER &= ~(0xF << 6); // ~(1 1 1 1) << 6 Push-pull output
	GPIOC->OSPEEDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 low speed
	GPIOC->PUPDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 no pull-up pull-down;
	
	// Use PA0 as analog input ADC_IN0
	GPIOA->MODER |= (0x3 << 0); // Set PA0 mode to  11 - analog input mode
	GPIOA->PUPDR &= ~(0x3 << 0); // Set PA0 PUPDR to 00 - no pull-up/pull-down
	
	// Use PA4 for DAC_OUT1
	GPIOA->MODER |= (0x3 << 8); // Set PA4 mode to  11 - analog input mode
	GPIOA->PUPDR &= ~(0x3 << 8); // Set PA4 PUPDR to 00 - no pull-up/pull-down
	
	// Configure ADC options:
	ADC1->CFGR1 |= (0x2 << 3); // Set bit resolution to 10 - 8 bits
	ADC1->CFGR1 |= (0x1 << 13); // Set single/continuous conversion mode to 1 - continuous conversion;
	ADC1->CFGR1 &= ~(0x3 << 10); // Set external trigger/polarity selection to 00 - hardware trigger detection disabled
	
	// Configure DAC options:
	DAC1->CR |= (0x1 << 2); // Set TEN1 to enable triggering
	DAC1->CR |= (0x7 << 3); // Set TSEL to 111 - software triggering;
	
	// Select ADC channel for conversion
	ADC1->CHSELR |= (0x1 << 0); // Select channel ADC_IN0
	
	// Calibrate ADC
	ADC1->CR |= (0x1 << 31); // Set ADCAL
	while((ADC1->CR & (0x1 << 31)) != 0)
	{
		// ADCAL is 1, Do nothing while waiting for calibration
	}
	
	// Enable ADC
	ADC1->CR |= (0x1 << 0); // Set ADEN
	
	// Enable DAC
	DAC1->CR |= (0x1 << 0); // Set EN1
	
	while((ADC1->ISR & (0x1 << 0)) == 0)
	{
		// ADRDY is 0, wait for ready
	}
	
	// Start ADC
	ADC1->CR |= (0x1 << 2); // Set ADSTART
	
	// Initialize useful variables
	uint16_t adcOutput = 0;
	uint16_t adcRes = 0xFF;
	uint16_t thresh1 = adcRes / 4;
	uint16_t thresh2 = (adcRes / 4) * 2;
	uint16_t thresh3 = (adcRes / 4) * 3;
	
	const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
	uint8_t iter = 0;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
		// Wait for end of conversion
		while((ADC1->ISR & (0x1 << 2)) == 0)
		{
			// EOC is 0, wait for conversion to end
		}
		
		adcOutput = ADC1->DR; // Read DR (should reset EOC)
		
		// Have data, set LEDs
		if((adcOutput <= thresh1) & (adcOutput >= 0))
		{
			// Within lower quartile, set starting LED as red, go CCW increasing
			GPIOC->ODR |= (1 << 6); // Set red
			GPIOC->ODR &= ~(1 << 8); // Came from orange, clear it.
		} else if ((adcOutput <= thresh2) & (adcOutput > thresh1))
		{
			// Within second quartile, set Orange, clear blue and red
			GPIOC->ODR |= (1 << 8); // Set orange
			GPIOC->ODR &= ~(1 << 6); // Came from red, clear it.
			GPIOC->ODR &= ~(1 << 7); // Came from blue, clear it.
		} else if ((adcOutput <= thresh3) & (adcOutput > thresh2))
		{
			// Within third quartile, set blue, clear orange and green
			GPIOC->ODR |= (1 << 7); // Set blue
			GPIOC->ODR &= ~(1 << 8); // Came from orange, clear it.
			GPIOC->ODR &= ~(1 << 9); // Came from green, clear it.
		} else
		{
			// Within upper quartile, set green, clear blue
			GPIOC->ODR |= (1 << 9); // Set green
			GPIOC->ODR &= ~(1 << 7); // Came from blue, clear it.
		}
		
		if (iter < 32)
		{
			DAC1->DHR8R1 &= ~(0xFF << 0);
			DAC1->DHR8R1 |= sine_table[iter];
			DAC1->SWTRIGR |= (0x1 << 0); // Set SWTRIG1 to 1 to send DAC
		} else
		{
			iter = 0;
			DAC1->DHR8R1 &= ~(0xFF << 0);
			DAC1->DHR8R1 |= sine_table[iter];
			DAC1->SWTRIGR |= (0x1 << 0); // Set SWTRIG1 to 1 to send DAC
		}
		iter += 1;
		HAL_Delay(1);
		

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
