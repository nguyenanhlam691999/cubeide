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
RCC_t *MY_RCC = 0x40021000;
GPIO_t *MY_GPIO_B = 0x40010c00;
GPIO_t *MY_GPIO_A = 0x40010800;
uint32_t *TIMER1_SR = (uint32_t*) 0x40012C10;
uint32_t *AFIO_EVCR = 0x40010000;
uint32_t *TIMER1_CCR = 0x40012c34;
int counter = 0;
void MY_RCC_INIT(char *mode) {
	if (strcmp(mode, "PLL") == 0) {
		MY_RCC->CFGR &= ~(0xf << 18);
		MY_RCC->CFGR |= (0b0110 << 18);
		MY_RCC->CFGR |= 0b10;
		MY_RCC->CFGR |= (0b0100 << 24);
		MY_RCC->CR |= (1 << 24);
	}
	if (strcmp(mode, "HSI") == 0) {
		MY_RCC->CFGR |= (0b00);
		MY_RCC->CR |= (1 << 0);
	}
}

void MY_TIMER1_INIT() {
	// 1000 times in 1 millis
	// TIMER1 : 40012C00
	uint32_t *TIMER1_PSC = (uint32_t*) 0x40012C28;
	uint32_t *TIMER1_SR = (uint32_t*) 0x40012C10;
	uint32_t *TIMER_ARR = (uint32_t*) 0x40012C2C;
	uint32_t *TIMER1_EGR = 0x40012c14;
	uint32_t *TIMER1_DIER = 0x40012c0c;
	uint32_t *TIMER1_CCMR1 = 0x40012c18;
	uint32_t *TIMER1_CCER = 0x40012c20;
	uint32_t *TIMER1_BDTR = 0x40012c44;
	uint32_t *TIMER1_CR1 = 0x40012c00;
	// SET PRE-SCALER PSC 0x28
	*TIMER1_PSC = 0;
	// SET AUTO-RELOAD ARR 0x2C
	*TIMER_ARR &= ~(0xffff);
	*TIMER_ARR |= 0xffff;
	// SET update generration
	*TIMER1_EGR |= (0b1);

	// Set TIMx_CCRx

	// Set Output Compare mode
	*TIMER1_CCMR1 &= ~(0b111 << 4);
	*TIMER1_CCMR1 |= (0b110 << 4);
	// Enable pre-load
	*TIMER1_CCMR1 |= (0b1 << 2);
	// Enable output fast mode
	*TIMER1_CCMR1 |= (0b1 << 3);
	// Enable Capture/Compare output
	*TIMER1_CCER |= (0b1);
	*TIMER1_BDTR |= (0b1 << 15)|(0b1<<13);
	// the auto-reload preload register
	*TIMER1_CR1 |= (0b1 << 7);
	// ENABLE TIMER1 BY SET CEN ON CR1
	*TIMER1_CR1 |= 0b1;
	*TIMER1_CCR = 10000;


}
void GPIO_PWM_OUTPUT() {
	// Alternate function push-pull
	// GPIO A8
	MY_GPIO_A->CRH &= ~(0b1111);
	MY_GPIO_A->CRH |= (0b1010);


}

void LED_BLINK() {
// SET GPIO OUTPUT
	MY_GPIO_B->CRL &= ~(0xf);
	MY_GPIO_B->CRL |= (0b001100110011);
	MY_GPIO_B->ODR |= 0xffff;
	for (int var = 0; (var < 3); ++var) {
		MY_GPIO_B->ODR &= ~(1 << var);
		//HAL_Delay(100);
		MY_DELAY(100);
		//SET LED OFF
		MY_GPIO_B->ODR |= (1 << var);
		//HAL_Delay(100);
		MY_DELAY(100);
	}
}
void TIM1_UP_IRQHandler() {
	uint32_t *TIMER1_SR = (uint32_t*) 0x40012C10;
	counter++;
	*TIMER1_SR &= ~0b1;
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
	//SystemClock_Config();
	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */
	//MY_RCC_INIT("HSI");
	// ENABLE CLOCK TIMER1
	//MY_RCC->APB2ENR |= (0b1 << 11);
	//MY_RCC->APB2ENR |= (0b1 << 3) | (0b1 << 2) | (0b1);
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM1_CLK_ENABLE();
	GPIO_PWM_OUTPUT();
	MY_TIMER1_INIT();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		LED_BLINK();





		/* USER CODE END 3 */
	}
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

		if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0)
				!= HAL_OK) {
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
