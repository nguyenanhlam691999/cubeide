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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;

} GPIO_t;
typedef struct {
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
} EXTI_t;
void delay(uint32_t time) {
	uint32_t timeout = time * 8000;
	for (int i = 0; i < timeout; i++) {
		__asm("NOP");
	}
}
/*void EXTI15_10_IRQHandler() {
	uint32_t *EXTI_PR = 0x40010414;
	*EXTI_PR |= (1 << 11);
	GPIO_t *PORTB0 = 0x40010C00;
	PORTB0->CRL &= ~(0xffffffff);
	PORTB0->CRL |= (0b00110011);

	PORTB0->ODR &= ~(0b1);
	delay(10);
	PORTB0->ODR |= (0b1);
	delay(10);

}*/
void my_interrupt()
{
	    uint32_t *EXTI_PR = 0x40010414;
		*EXTI_PR |= (1 << 11);
		GPIO_t *PORTB0 = 0x40010C00;
		PORTB0->CRL &= ~(0xffffffff);
		PORTB0->CRL |= (0b00110011);

		PORTB0->ODR &= ~(0b1);
		delay(10);
		PORTB0->ODR |= (0b1);
		delay(10);
}
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
	/* USER CODE BEGIN 2 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
//	EXTI15_10_IRQHandler();
	uint32_t*VTOR=0xE000ED08;
		*VTOR=0x20000000;
		memcpy(0x20000000 , 0x00000000, 304);
// set port pa11
	GPIO_t *PORTA11 = 0x40010800;
	PORTA11->CRH &= ~(0xffffffff);
	PORTA11->CRH |= (0b1000 << 12);
	PORTA11->ODR |= (0b1 << 11);

// set led output b0
	GPIO_t *PORTB0 = 0x40010C00;
	PORTB0->CRL &= ~(0xffffffff);
	PORTB0->CRL |= (0b00110011);
	PORTB0->ODR |= (0xffff);
	PORTB0->ODR |= (0b11 << 0);

//  set pin interrupt
	uint32_t *AFIO_EXTICR3 = 0x40010010;
	*AFIO_EXTICR3 &= ~(0b1111 << 12);
/// set interrupt mode
	EXTI_t *PORTA11_interrupt = 0x40010400;
	PORTA11_interrupt->IMR |= (1 << 11);
	PORTA11_interrupt->FTSR |= (1 << 11);
// ENABLE NVIC_ISER0
	uint32_t *NVIC_ISER1 = 0xE000E104;
	*NVIC_ISER1 |= (1 << 8);
// COPY

//set
	uint32_t *EXTI15_10_IRQHandler=0x200000E0;
	*EXTI15_10_IRQHandler=my_interrupt;
					/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		PORTB0->ODR &= ~(0b1 << 1);
		HAL_Delay(1000);
		PORTB0->ODR |= (0b1 << 1);
		HAL_Delay(1000);

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
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
