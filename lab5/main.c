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

int I2CRegRW(uint8_t* arrayBuff, uint8_t slvADR, uint8_t targADR, uint8_t nBytes, uint8_t rw)
{
	// Function for reading/writing (rw = 1/0) a register from a slave over I2C (gyro in this case)
	// returns 0 upon successful transfer, 1 otherwise.
	// If rw = 0, indicating a write operation, arrayBuff must contain nBytes to be sent to slave
	// if rw = 1, indicating read operation, arrayBuff must have nByte indices that will be written to by slave.
	
	// I2C register prep
	// Clear
	I2C2->CR2 &= ~(0xFF << 1); // clear slave address
	I2C2->CR2 &= ~(0xFF << 16); // clear nBytes
	I2C2->CR2 &= ~(1 << 10); // clear RW
	
	I2C2->CR2 |= (slvADR << 1); // 7-bit addressing, set LSB to bit 1
	if(rw == 1)
	{
		// Read operation, write slaveADR then RESET and read nBytes
		I2C2->CR2 |= (1 << 16); // send 1 byte, register location
	} 
	else
	{
		// Write operation, set nBytes here
		I2C2->CR2 |= ((1+nBytes) << 16); // send n bytes to slave (+1 for the sub address)
	}
	I2C2->CR2 &= ~(1 << 10); // write register location first (clear bit 10)
	
	// Start Transaction
	I2C2->CR2 |= (1 << 13); // START TRANSACTION
	
	// Wait for TXIS or NACKF
	while(((I2C2->ISR & (1 << 1)) == 0) & ((I2C2->ISR & (1 << 4)) == 0))
	{
		// Either TXIS (bit 1) or NACKF (bit 4) was set
		if(I2C2->ISR & (1 << 4))
		{
			// NACKF was set, exit everything
			return 1; // ERROR	
		}
		else if(I2C2->ISR & (1 << 1))
		{
			// TXIS was set
			break;
		}
		else
		{
			// Do nothing while waiting
		}
	}
	
	I2C2->TXDR = targADR; // Request read/write from/to this register

	if(rw == 1)
	{
		// READ TRANSFER COMPLETE - Reset for READ
		// Wait for TC transfer complete
		while((I2C2->ISR & (1 << 6)) == 0)
		{
			// Do nothing while waiting
		}
		
		// READ operation selected
		// I2C register prep
		// Clear
		I2C2->CR2 &= ~(0xFF << 1); // clear slave address
		I2C2->CR2 &= ~(0xFF << 16); // clear nBytes
		I2C2->CR2 &= ~(1 << 10); // clear RW
		// Write
		I2C2->CR2 |= (slvADR << 1); // 7-bit addressing, set LSB to bit 1
		I2C2->CR2 |= (nBytes << 16); // recieve nBytes
		I2C2->CR2 |= (1 << 10); // request read/write
		
		// Start Transaction - RESTART condition
		I2C2->CR2 |= (1 << 13); // START TRANSACTION
	}
	
	for(int i = 0; i < nBytes; i++)
	{
		if(rw == 1)
		{
			// READING
			// Wait for RXNE or NACKF
			while(((I2C2->ISR & (1 << 2)) == 0) & ((I2C2->ISR & (1 << 4)) == 0))
			{
				// Either RXNE (bit 2) or NACKF (bit 4) was set
				if(I2C2->ISR & (1 << 4))
				{
					// NACKF was set, exit everything
					return 1; // ERROR	
				}
				else if(I2C2->ISR & (1 << 2))
				{
					// RXNE was set
					break;
				}
				else
				{
					// Do nothing while waiting
				}
			}
			
			arrayBuff[i] = I2C2->RXDR; // read data transmitted and store it
		}
		else
		{
			// WRITING
			// Wait for TXIS or NACKF
			while(((I2C2->ISR & (1 << 1)) == 0) & ((I2C2->ISR & (1 << 4)) == 0))
			{
				// Either TXIS (bit 0) or NACKF (bit 4) was set
				if(I2C2->ISR & (1 << 4))
				{
					// NACKF was set, exit everything
					return 1; // ERROR	
				}
				else if(I2C2->ISR & (1 << 1))
				{
					// TXIS was set
					break;
				}
				else
				{
					// Do nothing while waiting
				}
			}
			I2C2->TXDR = arrayBuff[i]; // write data to periph
		}

	}
	// Wait for TC transfer complete
	while((I2C2->ISR & (1 << 6)) == 0)
	{
		// Do nothing while waiting
	}
	
	// STOP condition - release the bus!
	I2C2->CR2 |= (1 << 14); // Set STOP bit
	
	return 0; // Successful transfer
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
	
	// Clock Config
	RCC->AHBENR |= ((1 << 18) | (1 << 19)); // Enable GPIOB (bit 18) and GPIOC (bit 19) clocks
	RCC->APB1ENR |= (1 << 22); // Enable I2C2 clock
	
	// I/O Config
	// PB11 - SDA (connected to PB15, but PB15 left as input mode)
	GPIOB->MODER |= (2 << 22); // Set PB11 mode to 10 - alt func mode
	GPIOB->OTYPER |= (1 << 11); // Set output type to 1 - open-drain
	GPIOB->AFR[1] |= (0x1 << 12); // Select PB11 AF1 (0001) - I2C_SDA
	// PB13 - SCL
	GPIOB->MODER |= (2 << 26); // Set PB13 mode to 10 - alt func mode
	GPIOB->OTYPER |= (1 << 13); // Set output type to 1 - open-drain
	GPIOB->AFR[1] |= (0x5 << 20); // Select PB11 AF5 (0101) - I2C2_SCL
	// PB14 - I/O
	GPIOB->MODER |= (1 << 28); // Set PB14 mode to 01 - gen output
	GPIOB->OTYPER &= ~(1 << 14); // Set output type to 0 - push-pull
	GPIOB->ODR |= (1 << 14); // Initialize pin to 1
	// PC0 - I/O
	GPIOC->MODER |= (1 << 0); // Set PC0 mode to 01 - gen output
	GPIOC->OTYPER &= ~(1 << 0); // Set output type to 0 - push-pull
	GPIOC->ODR |= (1 << 0); // Initialize pin to 1
	// LED Config
	GPIOC->MODER |= (0x55 << 12); // 01 01 01 01 << 12 red, blue, orange, green general purpose output
	GPIOC->OTYPER &= ~(0xF << 6); // ~(1 1 1 1) << 6 Push-pull output
	GPIOC->OSPEEDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 low speed
	GPIOC->PUPDR &= ~(0xFF << 12); // ~(11 11 11 11) << 12 no pull-up pull-down;
	
	// I2C2 Config - 100 kHz standard mode
	I2C2->TIMINGR |= (1 << 28); // PRESC 0001
	I2C2->TIMINGR |= (0x13 << 0); // SCLL 0x13
	I2C2->TIMINGR |= (0xF << 8); // SCLH 0xF
	I2C2->TIMINGR |= (0x2 << 16); // SDADEL 0x2
	I2C2->TIMINGR |= (0x4 << 20); // SCLDEL 0x4
	
	// Init variables
	uint16_t hasRun = 0;
	
	// Peripheral Enables (DO THIS LAST)
	I2C2->CR1 |= (1 << 0); // Enable I2C2, config now locked
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		// PART 1 CHECK-OFF
//		if(hasRun == 0) //hasRun == 0
//		{
//			// Init params
//			uint8_t slvADR = 0x69; // Slave address
//			uint8_t targADR = 0x0F; // WHO_AM_I in i3g4250d
//			uint8_t nBytes = 1; // Number of bytes to read/write
//			uint8_t arrayBuff[1];
//			
//			int I2CSuccess = I2CRegRW(arrayBuff, slvADR, targADR, nBytes, 1); // Read
//			
//			if(I2CSuccess == 0)
//			{
//				// Successful transfer!
//				// Check if RXDR matches what we expect (WHO_AM_I == 0xD3 for gyroscope)
//				if(arrayBuff[0] & 0xD3)
//				{
//					GPIOC->ODR |= (1 << 7); // Set BLUE LED, indicating successful I2C transaction!
//				}
//				hasRun = 1;
//				continue;
//			}
//			else
//			{
//				// Unsuccessful transfer...
//				hasRun = 1;
//				continue;
//			}
//			
//		}
		
		// CHECK-OFF PART 2
		
		// Initialize Gyro
		// CTRL_REG1 - 0000 1011 - 0x0B
		// DR = 00 BW = 00 - ODR 100 Hz, Cutoff 12.5
		// PD = 1 - Power on
		// Zen = 0 - Z axis disabled
		// Yen = 1 - Y axis enabled
		// Xen = 1 - X axis enabled
		uint8_t slvADR = 0x69; // Gyro address
		uint8_t targADR;
		
		// Configure CTRL_REG1
		uint8_t cfgBuff[1];
		cfgBuff[0] = 0x0B; // CTRL_REG1 word 0x0B
		targADR = 0x20; // CTRL_REG1 
		int I2CSuccess = I2CRegRW(cfgBuff, slvADR, targADR,1,0); // write 1 byte
		if(I2CSuccess == 1)
		{
			// I2C failed, retry
			continue;
		}
		HAL_Delay(1);
		
		// Read X 0x28(L) and 0x29(H)
		uint8_t xBuff[2];
		targADR = 0x28;
		targADR |= (1 << 7); // Set bit 7 on sub address, allowing register auto-increment
		I2CSuccess = I2CRegRW(xBuff, slvADR, targADR, 2, 1); // Read two bytes
		if(I2CSuccess == 1)
		{
			// I2C failed, retry
			continue;
		}
		HAL_Delay(1);
				
		// Read Y 0x2A(L) and 0x2B(H)
		uint8_t yBuff[2];
		targADR = 0x2A;
		targADR |= (1 << 7); // Set bit 7 on sub address, allowing register auto-increment
		I2CSuccess = I2CRegRW(yBuff, slvADR, targADR, 2, 1); // Read two bytes
		if(I2CSuccess == 1)
		{
			// I2C failed, retry
			continue;
		}
		
		// X and Y angular rate successfully measured.
		// Combine X
		int16_t xRot = ((int16_t) xBuff[1]) << 8;
		xRot |= ((int16_t) xBuff[0]);
		
		// Combine Y
		int16_t yRot = ((int16_t) yBuff[1]) << 8;
		yRot |= ((int16_t) yBuff[0]);
		
		// Thresholding
		int upBnd = 125;
		int lowBnd = -125;
		
		// X LEDs
		if(xRot < lowBnd)
		{
			GPIOC->ODR |= (1 << 8); // Set Orange HI
			GPIOC->ODR &= ~(1 << 9); // Set Green LO
		}
		else if (xRot > upBnd)
		{
			GPIOC->ODR |= (1 << 9); // Set Green HI
			GPIOC->ODR &= ~(1 << 8); // Set Orange LO
		}
		else
		{
			GPIOC->ODR &= ~(1 << 8); // Set Orange LO
			GPIOC->ODR &= ~(1 << 9); // Set Green LO
		}
		
		// Y LEDs
		// X LEDs
		if(yRot > upBnd)
		{
			GPIOC->ODR |= (1 << 6); // Set Red HI
			GPIOC->ODR &= ~(1 << 7); // Set Blue LO
		}
		else if (yRot < lowBnd)
		{
			GPIOC->ODR |= (1 << 7); // Set Blue HI
			GPIOC->ODR &= ~(1 << 6); // Set Red LO
		}
		else
		{
			GPIOC->ODR &= ~(1 << 6); // Set Red LO
			GPIOC->ODR &= ~(1 << 7); // Set Blue LO
		}
		
		// Do this once every 100 milliseconds
		HAL_Delay(98);
		
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
