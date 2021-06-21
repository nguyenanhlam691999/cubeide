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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
typedef struct {
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;

} GPIO_t;
typedef struct {
	uint32_t CR1;
	uint32_t CR2;
	uint32_t OAR1;
	uint32_t OAR2;
	uint32_t DR;
	uint32_t SR1;
	uint32_t SR2;
	uint32_t CCR;
	uint32_t TRIS;
} I2C_t;
GPIO_t *my_gpioB = 0x40010c00;
I2C_t *my_i2c = 0x40005800;
uint32_t data = 0;

void i2c_init() {
	// CLEAR BIT PORT B
	my_gpioB->CRH &= ~(0b1111 << 8) | (0b1111 << 12);
	// SET GPIO PIN B10 B11 Alternate function open drain
	my_gpioB->CRH |= (0b1111 << 8) | (0b1111 << 12);
	// DISABLE PERIPHERAL PORT B
	my_i2c->CR1 &= ~(0b1);
	while (((my_i2c->SR2 >> 1) & 1) == 1) {
		my_i2c->CR1 |= (1 << 15);
		my_i2c->CR1 &= ~(1 << 15);
	}
	// SET SCL CLOCK
	my_i2c->CCR = 0x28;
	// SET I2C PERIPHERAL CLOCK
	my_i2c->CR2 |= 8;
	// ENABLE I2C
	my_i2c->CR1 |= (0b1);
	// ENABLE ACK
	//my_i2c->CR1 |= (1 << 10);
}
void i2c_write() {
	int temp;
	// START
	my_i2c->CR1 |= (1 << 8);
	// WAIT SB SET
	while ((my_i2c->SR1 & 1) != 1)
		;
	// SEND 7 BIT ADDRESS
	my_i2c->DR = (0b1010000 << 1) | 0;
	// WAIT ADDR SET
	while (((my_i2c->SR1 >> 1) & 1) != 1)
		;
	// CLEAR ADDR
	temp = my_i2c->SR1;
	temp = my_i2c->SR2;
	// WAIT TX iS SET
	while (((my_i2c->SR1 >> 7) & 1) != 1)
		;
	// WIRTE ADDRESS VARIABLE
	my_i2c->DR = 0x08;
	// WAIT DATA SEND
	while (((my_i2c->SR1 >> 7) & 1) != 1)
		;
	// WRITE VALUE
	my_i2c->DR = 1;
	// WAIT DATA SEND
	while (((my_i2c->SR1 >> 7) & 1) != 1)
		;
	// STOP
	my_i2c->CR1 |= (0b1 << 9);
}
void i2c_read() {
	int temp;
	// START
	my_i2c->CR1 |= (1 << 8);
	// WAIT SB SET
	while ((my_i2c->SR1 & 1) != 1)
		;
	// SEND 7 BIT ADDRESS
	my_i2c->DR = (0b1010000 << 1) | 0;
	// WAIT ADDR SET
	while (((my_i2c->SR1 >> 1) & 1) != 1)
		;
	// CLEAR ADDR
	temp = my_i2c->SR2;
	// WAIT TX iS SET
	while (((my_i2c->SR1 >> 7) & 1) != 1)
		;
	// WIRTE ADDRESS VARIABLE
	my_i2c->DR = 0x08;
	// WAIT DATA SEND
	while (((my_i2c->SR1 >> 7) & 1) != 1)
		;
	// STOP
	my_i2c->CR1 |= (0b1 << 9);

	// START
	my_i2c->CR1 |= (1 << 8);
	// WAIT SB SET
	while ((my_i2c->SR1 & 1) != 1)
		;

	//SEND 7 BIT ADDRESS
	my_i2c->DR = (0b1010000 << 1) | 1;

	// WAIT ADDR SET
	while (((my_i2c->SR1 >> 1) & 1) != 1)
		;
	// CLEAR ADDR
	temp = my_i2c->SR2;

	while ((((my_i2c->SR1 >> 6) & 1) != 1))
		;
	// READ DATA
	data = my_i2c->DR;
	// STOP
	my_i2c->CR1 |= (0b1 << 9);

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_I2C2_CLK_ENABLE();
	i2c_init();
	i2c_write();
	i2c_read();
	/* USER CODE END 2 */

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
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
