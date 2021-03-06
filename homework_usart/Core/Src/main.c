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

/*typedef struct {
 uint8_t mantissa :4;
 uint32_t fraction :12;
 } BRR_t;*/
#define Mantissa 4
#define Fraction 0
typedef struct {
	uint32_t CRL;
	uint32_t CRH;
	uint32_t IDR;
	uint32_t ODR;

} GPIO_t;
typedef struct {
	uint32_t SR;
	uint32_t DR;
	uint32_t BRR;
	uint32_t CR1;
	uint32_t CR2;
	uint32_t CR3;
	uint32_t GTPR;

} myUSART_t;
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

} RCC_t;

typedef struct {

	uint32_t CCR;
	uint32_t CNDTR;
	uint32_t CPAR;
	uint32_t CMAR;
} DMA_t;
myUSART_t *myUSART1 = 0x40013800;
GPIO_t *PINA = 0x40010800;
RCC_t *myRCC = 0x40021000;
DMA_t *myDMA = 0x40020058;
char dataUsart[128] = { 0 };
char data = 0;
void my_dmaInit() {

	myDMA->CPAR = 0x40013804;
	myDMA->CMAR = dataUsart;
	myDMA->CNDTR = sizeof(dataUsart);
	// set memory size and peripheral size 8 bit
	myDMA->CCR &= ~(0b1111 << 8);
	// set circle mode enable memory increase
	myDMA->CCR |= (0b1 << 5) | (0b1 << 7);
	//enable dma
	myDMA->CCR |= (0b1 << 0);
	// enable usart dma
	myUSART1->CR3 |= (0b1 << 6);
}
;
void my_usartInit() {
	// SET TX A9 alternative OUTPUT PUSH PULL  RX A10 INPUT FLOATING
	PINA->CRH &= ~0xffff;
	PINA->CRH |= (0b1011 << 4) | (0b0100 << 8);
	// SET USART BAUD RATE
	myUSART1->BRR = (52 << Mantissa) | (1 << Fraction);
	// SET DATA LENGTH 8 BIT
	myUSART1->CR1 &= ~(0b1 << 12);
	// ENABLE USART , RECEIVER , TRANSMITTER
	myUSART1->CR1 |= (0b1 << 13) | (0b1 << 3) | (0b1 << 2);

}

void my_usartSend(char data) {
	//  SET UP DATA SEND
	myUSART1->DR = data;
	// WAITE UNTIL DATA SEND
	while (((myUSART1->SR >> 6) & 1) != 1)
		;
}
void my_usartInit_interrupt() {
	// ENABLE INTERRUPT FOR USART1 RXNEIE
	myUSART1->CR1 |= (1 << 5);
	//  ENABLE VECTOR INTTERUPT USART1
	uint32_t *NVIC_ISER0 = 0xE000E104;
	*NVIC_ISER0 |= (1 << 5);

}
char my_usartGet() {

	// WAIT UNTIL DATA READY TO READ
	while (((myUSART1->SR >> 5) & 1) != 1)
		;
	// GET DATA
	return myUSART1->DR;
}
void USART1_IRQHandler() {
	// SET PENDING INTERRUPT VECTOR USART1
	/*uint32_t *NVIC_ISPR0 = 0XE000E204;
	 *NVIC_ISPR0 |= (0b1 << 5);*/
	// GET DATA
	data = my_usartGet();
	/*uint32_t *NVIC_ICPR0 = 0XE000E208;
	 *NVIC_ISPR0 &=~ (0b1 << 5);*/

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
	// ENABLE CLOCK PORT A AND USART1
	//HAL_Delay(50);
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
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	//myRCC->APB2ENR |= (1 << 14) | (1 << 2);
	my_usartInit();
	my_usartInit_interrupt();
	my_dmaInit();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		my_usartSend(1);
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
