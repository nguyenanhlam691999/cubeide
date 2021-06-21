/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
	uint16_t CR1;
	uint16_t CR2;
	uint16_t OAR1;
	uint16_t OAR2;
	uint16_t DR;
	uint16_t SR1;
	uint16_t SR2;
	uint16_t CCR;
	uint16_t TRISE;
} I2C;

typedef struct {
	uint32_t CR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t APB2RSTR;
	uint32_t APB1RSTR;
	uint32_t AHBENR;
	uint32_t APB2ENR;
	uint32_t APB1ENR;
	uint32_t BDCR;
	uint32_t CSR;
} RCC_RG;

typedef struct {
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;
	uint32_t BSRR;
	uint32_t BRR;
	uint32_t LCKR;
} GPIO;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
RCC_RG *_RCC = 0x40021000;
I2C *I2C_2 = 0x40005800;
GPIO *GPIO_B = 0x40010C00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void generate_start_bit() {
	// Create a start bit
	I2C_2->CR1 |= (1 << 8);
	// Wait for SB set to 1
	while((I2C_2->SR1 & 1) == 0);
}

void set_DR_register(uint8_t value) {
	I2C_2->DR = value;
}

void wait_ADDR() {
	// Wait for ADDR set to 1
	while(((I2C_2->SR1 >> 1) & 1) == 0);
}

void wait_DR_empty() {
	// Wait for TxE set to 1
	while(((I2C_2->SR1 >> 7) & 1) == 0);
}

void wait_DR_not_empty() {
	// Wait for RxNE set to 1
	while(((I2C_2->SR1 >> 6) & 1) == 0);
}

void access_eeprom_data(char mode[5], uint8_t RW_address, uint8_t *msg) {
	generate_start_bit();
	set_DR_register((0x50 << 1) | 0);
	wait_ADDR();
	set_DR_register(RW_address);
	wait_DR_empty();
	if(mode == 'write')
	{
		set_DR_register(*msg);
		wait_DR_empty();
	}
	else if(mode == 'read')
	{
		generate_start_bit();
		set_DR_register((0x50 << 1) | 1);
		wait_DR_not_empty();
		*msg = I2C_2->DR;
	}
	// Generate a stop bit
	I2C_2->CR1 |= (1 << 9);
}
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
//  MX_GPIO_Init();
//  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  _RCC->APB1ENR |= (1 << 22);		// Enable clock for I2C2
  _RCC->APB2ENR |= (1 << 3);		// Enable clock for GPIOB

  GPIO_B->CRH |= (0xFF << 8);		// Set PB10, PB11 output mode AF open-drain max 50MHz

  I2C_2->CR1 &= ~(1 << 0);			// Disable I2C Peripheral
  while((((I2C_2->SR2) >> 1) & 1) == 1)
  {
	  I2C_2->CR1 |= (1 << 15);
	  I2C_2->CR1 &= ~(1 << 15);
  }
  I2C_2->CR2 &= ~(0b111111 << 0);	// Reset peripheral clock
  I2C_2->CR2 |= (0b101101 << 0);	// Set peripheral clock to 45MHz
  I2C_2->CCR |= (225 << 0);			// Set SCL clock
  I2C_2->CR1 |= (1 << 0);			// Enable I2C Peripheral

  uint8_t write_msg = 1;
  access_eeprom_data('write', 0x10, &write_msg);
  uint8_t read_data;
  access_eeprom_data('read', 0x1, &read_data);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
