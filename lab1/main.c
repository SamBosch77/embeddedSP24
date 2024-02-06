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

	HAL_Init(); // Reset of all peripherals, init the Flash and Systick 
	SystemClock_Config(); //Configure the system clock 
//	/* This example uses HAL library calls to control the GPIOC peripheral. 
//	You’ll be redoing this code with hardware register access. */ 
//	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in theRCC 
//	// Set up aconfiguration struct to passtothe initialization function 
//	GPIO_InitTypeDef initStr ={GPIO_PIN_8 |GPIO_PIN_9, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
//	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8 & PC9 
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);//Start PC8 high 
//	while (1) 
//	{ 
//		HAL_Delay(200);// Delay 200ms 
//		// Toggle the output state of both PC8and PC9 
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9); 
//	}
	
	// Init RCC for GPIOC and GPIOA (DO THIS FIRST IF NOT USING HAL)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Sourced from stm32f072xb.h
	RCC->AHBENR |= (1 << 19);
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= (1 << 17);
	
	// Clear MODERA and MODERC - reset states
	GPIOC->MODER &= 0x0; // clear GPIOC
	GPIOC->OTYPER &= 0x0;
	GPIOC->OSPEEDR &= 0x0;
	GPIOC->PUPDR &= 0x0;
	GPIOA->MODER &= 0x0; // clear GPIOA
	
	// Set pins 12 and 14 (PC6[01], PC7[01], PC8[01], PC9[01], PA0[00]) - general purpose i/o
	// GPIOA PA0 is already input
	GPIOA->MODER &= ~((1 << 0) | (1 << 1)); 
	GPIOC->MODER |= (0x55 << 12); // ...010101010000...
	
	// Pin details:
	// PC6,7,8,9 push-pull
	
	GPIOC->OTYPER &= ~(0x0F << 6); // Make all push-pull
	GPIOC->OSPEEDR &= ~(0x0FF << 12); // Set all to LOW speed
	GPIOC->PUPDR &= ~(0x0FF << 12); // Set all to no pull-up or pull-down
	
	//PA0 settings
	GPIOA->OSPEEDR &= ~(1 << 0); // Set all to LOW speed
	GPIOA->PUPDR |= ((1 << 1) | ~(1 << 0) ); // Set to pull-down
	
	// Init debouncer and toggle code
	uint32_t debouncer = 0;
	uint32_t toggled = 1;
	uint32_t isBlue = 0;
	uint32_t demoMode = 0;  // 0 for LED flashing, 1 for button toggle part
	
	// Init 1 LED on, 1 LED off
	GPIOC->ODR |= (1 << 6); // RED LED (PC6) HI
	GPIOC->ODR &= ~(1 << 7); // BLU LED (PC7) LO
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		if(demoMode == 0)
		{
			HAL_Delay(200); // Delay 200 ms
			GPIOC->ODR |= (1 << 7); // BLU LED (PC7) HI
			GPIOC->ODR &= ~(1 << 6); // RED LED (PC6) LO
			
			HAL_Delay(200); // Delay 200 ms
			GPIOC->ODR |= (1 << 6); // RED LED (PC6) HI
			GPIOC->ODR &= ~(1 << 7); // BLU LED (PC7) LO
		} else
		{
			
			debouncer = (debouncer << 1);
			
			if(GPIOA->IDR & 0x1) // read from button
			{
				debouncer |= 1;  // set lowest bit if button is high
			}
			
			if((debouncer == 0xFFFFFFFF) & (toggled != 1)) //steady high
			{
				toggled = 1;
				// Toggle the LED
				if(isBlue == 1)
				{
					GPIOC->ODR |= (1 << 6); // Set red HI
					GPIOC->ODR &= ~(1 << 7); // Set blu LO
					isBlue = 0;
				} else
				{
					GPIOC->ODR &= ~(1 << 6); // Set red LO
					GPIOC->ODR |= (1 << 7); // Set blu HI
					isBlue = 1;
				}
			}
			
			if(debouncer == 0x0)
			{
				toggled = 0;
			}
		}
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

