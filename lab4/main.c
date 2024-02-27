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
volatile char newData;
volatile uint32_t ndFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Define character transmission function
void transmitChar(char input)
{
	uint32_t txeMask = 1 << 7;
	while ((USART3->ISR & txeMask) == 0)
	{
		// Do nothing while transmit register is not empty
	}
	// Transmit register is now empty, write to register
	USART3->TDR = input;
	
}

char readChar()
{
	char val;
	uint32_t rxneMask = 1 << 5;
	while((USART3->ISR & rxneMask) == 0)
	{
		// Don't do anything while recieve register is empty
	}
	val = USART3->RDR;
	return val;
}

void transmitStr(char *input)
{
	uint32_t i = 0;
	while(input[i] != '\0') 
	{
		transmitChar(input[i]);
		i = i + 1;
	}
}

void USART3_4_IRQHandler()
{
	newData = (USART3->RDR & 0xFF); // Only take needed bits, not reserved ones
	ndFlag = 1;
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
  /* USER CODE BEGIN 2 */
	
	// Initialize clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= (0x1 << 18); // (1) enable USART3 clock
	
	// Startup peripheral configuration
	GPIOB->MODER |= (0xA << 20); // (10 10) Use PB10 TX and PB11 RX in alt func mode
	USART3->BRR = 69; // OVER8 = 0, so 69 -> 115942 baud rate -> 0.64% error

	GPIOC->MODER |= (0x55 << 12); // 01 01 01 01 << 12 red, blue, orange, green general purpose output
	GPIOC->OTYPER &= ~(0xF << 6); // ~(1 1 1 1) << 6 Push-pull output
	GPIOC->OSPEEDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 low speed
	GPIOC->PUPDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 no pull-up pull-down;
	
	// Select alternate functions
	GPIOB->AFR[1] |= (0x44 << 8); // (0100 0100) select USART3_TX for PB10, USART3_RX for PB11
	
	
	// Enable USART (do this last)
	USART3->CR1 |= (0x1 << 2); // Enable reciever
	USART3->CR1 |= (0x1 << 3); // Enable transmitter
	USART3->CR1 |= (0x1 << 5); // Enable USART RXNE interrupts
	USART3->CR1 |= (0x1); // Set UE to enable USART

	NVIC_EnableIRQ(USART3_4_IRQn); // 29
	NVIC_SetPriority(USART3_4_IRQn, 1); // 29

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
//		// Check-off 1 code:
//	  // Call transmit and delay
//		char recieveKey = readChar();
//		
//		// Check the char
//		switch(recieveKey)
//		{
//			case 'r':
//				GPIOC->ODR ^= (0x1 << 6);
//				break;
//			case 'b':
//				GPIOC->ODR ^= (0x1 << 7);
//				break;
//			case 'o':
//				GPIOC->ODR ^= (0x1 << 8);
//				break;
//			case 'g':
//				GPIOC->ODR ^= (0x1 << 9);
//				break;
//			default:
//				transmitStr("ERROR: Key input does not map to valid LED");
//				break;
//		}
		
		char colorInput;
		char stateInput;
		
		uint32_t ledPos = 0; 
		
		transmitStr("Command?\n\r");
		while(ndFlag == 0)
		{
//			// Do nothing while no data
//			if(ndFlag == 1)
//			{
//				break;
//			}
		}
		// ndFlag is 1
		colorInput = newData;
		ndFlag = 0;
		
		switch(colorInput)
		{
			case 'r':
				ledPos = 6;
				break;
			case 'b':
				ledPos = 7;
				break;
			case 'o':
				ledPos = 8;
				break;
			case 'g':
				ledPos = 9;
				break;
			default:
				transmitStr("ERROR: Key input does not map to valid LED, please try again.\n\r");
				continue; // Go to next iteration; start over
		}
		transmitStr("Selected LED: ");
		transmitChar(colorInput);
		transmitChar('\n');
		transmitChar('\r');
		
		while(ndFlag == 0)
		{
			// Do nothing while no data
		}
		// ndFlag is 1
		stateInput = newData;
		ndFlag = 0;
		
		// Check for validity
		switch(stateInput)
		{
			case '0':
				break;
			case '1':
				break;
			case '2':
				break;
			default:
				transmitStr("ERROR: Key input does not map to valid state, please try again.\n\r");
				continue; // Go to next iteration; start over
		}
		transmitStr("Selected state: ");
		transmitChar(stateInput);
		transmitChar('\n');
		transmitChar('\r');
		
		transmitStr("Command recognized: ");
		transmitChar(colorInput);
		transmitChar(stateInput);
		transmitChar('\n');
		transmitChar('\r');
		
		// Set the actual LEDs post recognization
		switch(stateInput)
		{
			case '0':
				GPIOC->ODR &= ~(1 << ledPos);
				break;
			case '1':
				GPIOC->ODR |= (1 << ledPos);
				break;
			case '2':
				GPIOC->ODR ^= (1 << ledPos);
				break;
			default:
				transmitStr("WTF? How did we get here?\n\r");
				continue; // Go to next iteration; start over
		}
		
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

