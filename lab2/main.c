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
volatile uint32_t extiCtr = 0;
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
	// Enable clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= (1 << 0); // Enable SYSCFG and COMP clock
	
	// Configure GPIOC and GPIOA
	GPIOC->MODER |= (0x55 << 12); // 01 01 01 01 << 12 Gen purpose output
	GPIOC->OTYPER &= ~(0xF << 6); // ~(1 1 1 1) << 6 Push-pull output
	GPIOC->OSPEEDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 low speed
	GPIOC->PUPDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 no pull-up pull-down;
	
	GPIOA->MODER &= ~(0x3 << 0); // ~(11) << 0 Input mode
	GPIOA->OSPEEDR &= ~(0x3 << 0); // ~(11) << 0 low speed
	GPIOA->PUPDR |= (1 << 1); // 10 pull-down resistor
	
	// EXTI CONFIG
	EXTI->IMR |= (1 << 0); // Unmask interrupt request from line 0
	EXTI->RTSR |= (1 << 0); // 1 RT0 rising trigger enabled
	
	// SYSCFG CONFIG
	SYSCFG->EXTICR[0] &= ~(0xF << 0); // ~(1111) << 0 PA0 selected and linked to EXTI0
	
	// NVIC Enable IRQ
	NVIC_EnableIRQ(5); // 5 corresponds to EXTI0 and EXIT1 lines
	NVIC_SetPriority(5, 1); // Set EXTI0 and EXTI1 lines to [1] high priority
	
	// System clock priority, must be initialized for systick to not be overwritten
	NVIC_SetPriority(-1, 0); // Set systick to priority 0 to have highest priority, EXTI priority or lower to starve
	
	// Set Green LED high
	GPIOC->ODR |= (1 << 9); // PC9 (green LED) HI
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		
		// Toggle the red LED with 500 ms delay
		HAL_Delay(500);
		GPIOC->ODR |= (1 << 6); // ON
		HAL_Delay(500);
		GPIOC->ODR &= ~(1 << 6); // OFF
  }
  /* USER CODE END 3 */
}

void EXTI0_1_IRQHandler(void)
{
	GPIOC->ODR ^= (1 << 9); // Toggle green
	GPIOC->ODR ^= (1 << 8); // Toggle orange
	
	while(extiCtr < 1500000)
	{
		extiCtr += 1;
	}
	
	// Toggle once more
	GPIOC->ODR ^= (1 << 9); // Toggle green
	GPIOC->ODR ^= (1 << 8); // Toggle orange
	extiCtr = 0;
	EXTI->PR |= (1 << 0); // Set bit 0 (clear action) of pending register; handler is done.
	
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
