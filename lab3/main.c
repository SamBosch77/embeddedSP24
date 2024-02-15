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
	SystemClock_Config();
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
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // GPIOC
	RCC->APB1ENR |= 0x3; // (1 1 enable TIM3, enable TIM2)
	
	// Configure GPIOC
	GPIOC->MODER |= (0x5A << 12); // 01 01 10 10 << 12 Green + Orange general out, Red and blue alt func
	GPIOC->OTYPER &= ~(0xF << 6); // ~(1 1 1 1) << 6 Push-pull output
	GPIOC->OSPEEDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 low speed
	GPIOC->PUPDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 no pull-up pull-down;
	
	// Configure TIM2 UEV frequency
	// Target f = 4 Hz = 250 ms period. Divide 8 MHz by 8000 to get clock freq of 1 kHz = 1 ms period, set ARR to 250
	// PSC = 8000 - 1 = 7999
	// ARR = 250
	TIM2->PSC = 7999;//0x1F3F; // Divide 8 MHz clock by 8000 to get 1 kHz clock
	TIM2->ARR = 250;//0xFA; // UEV after 250 1 ms counts
	
	// Configure TIM3 UEV frequency
	// We desire larger PSC, since it creates more UEVs within a desired period, giving greater PWM resolution.
	TIM3->PSC = 3;//0x1F3F; // Divide 8 MHz clock by to get 2 MHz clock
	TIM3->ARR = 2500; // 2500 counts at 8 MHz gives 1.25 ms period -> 800 Hz ideally
	
	// Configure CCMR1 to output PWM
	TIM3->CCMR1 &= ~(0x3 << 8); // Clear bits 8 and 9 (00) to select channel 2 as output CCS2
	TIM3->CCMR1 &= ~(0x3 << 0); // Clear bits 0 and 1 (00) to select channel 1 as output CCS1
	TIM3->CCMR1 |= (0x7 << 4); // Write 111 to OC1M to select PWM Mode 2
	TIM3->CCMR1 |= (0x6 << 12); // Write 110 to OC2M to select PWM Mode 1
	TIM3->CCMR1 |= (0x1 << 3); // Set OC1PE output compare preload enable
	TIM3->CCMR1 |= (0x1 << 11); // Set OC2PE output compare preload enable
	TIM3->CCER |= (0x1 << 0); // Set CCE1 output enable for channel 1
	TIM3->CCER |= (0x1 << 4); // Set CCE2 output enable for channel 2
	
	// Set Channel PWM
	TIM3->CCR1 = (20 * 2500) / 100; // (20 / 100) * ARR channel 1
	TIM3->CCR2 = (20 * 2500) / 100; // (20 / 100) * ARR channel 2
	
	// Select alternate functions for red and blue LEDs
	GPIOC->AFR[0] &= (0xF << 24); // 0000 (AF TIM3_CH1) on bits 24-27 -> PC6
	GPIOC->AFR[0] &= (0xF << 28); // 0000 (AF TIM3_CH2) on bits 28-31 -> PC7
	
	// Enable TIM2 UEV to interrupt
	TIM2->DIER |= 0x1; // 1 << 0 update enable; ***
	
	// Set up TIM2 NVIC enable
	NVIC_EnableIRQ(TIM2_IRQn);
	
  /* USER CODE END 2 */

	// Start TIM2 timer
	TIM2->CR1 &= ~(1 << 1); // Clear bit 1 -> enable UDIS
	TIM2->CR1 |= 0x1; // (...10 -> ...11) start timer 2 
	
	// Enable TIM3
	TIM3->CR1 |= (0x1 << 0); // Enable TIM3 CEN at bit 0
	
	// Initialize LEDs, green HI, orange LO
	GPIOC->ODR |= (1 << 9);
	GPIOC->ODR &= ~(1 << 8);
	
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

// TIM2 Interrupt request handler
void TIM2_IRQHandler(void)
{
	GPIOC->ODR ^= (1 << 9); // Toggle green
	GPIOC->ODR ^= (1 << 8); // Toggle orange
	TIM2->SR &= ~(1 << 0); // Clear status flag
}

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
