#include<stdio.h>
#include <string.h>
#include<stdint.h>
#include"my_define.h"

RCC_t *MY_RCC = 0x40021000;
GPIO_t *MY_GPIO_B = 0x40010c00;
void Custom_SysTick_Init() {
	uint32_t *SYST_CSR = (uint32_t*) (0xE000E010);
	uint32_t *SYST_RVR = (uint32_t*) (0xE000E014);
	*SYST_CSR = (*SYST_CSR & ~(uint32_t) 0x0007) | 7;
	*SYST_RVR = 8000;
}
void Custom_Delay(uint32_t milisec) {
	uint32_t *SYST_CSR = (uint32_t*) (0xE000E010);
	*SYST_CSR = (*SYST_CSR & ~(uint32_t) 0x0001) | 1;
	uint32_t cnt = 0;
	while (cnt < milisec) {
		if (((*SYST_CSR & (uint32_t) 0x10000) >> 16) == 1)
			cnt++;
	}
	*SYST_CSR = (*SYST_CSR & ~(uint32_t) 0x0001);
}
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
	// SET PRE-SCALER PSC 0x28
	uint32_t *TIMER1_PSC = (uint32_t*) 0x40012C28;
	*TIMER1_PSC = 7;
	// SET AUTO-RELOAD ARR 0x2C
	uint32_t *TIMER_ARR = (uint32_t*) 0x40012C2C;
	*TIMER_ARR &= ~(0xffff);
	*TIMER_ARR = 1000;

}

void MY_DELAY(uint32_t millis) {
	int counter = 0;
	// counter value
	uint32_t *TIMER1_CNT = (uint32_t*) 0x40012C24;
	// ENABLE TIMER1 BY SET CEN ON CR1
	uint32_t *TIMER1_CR1 = (uint32_t*) 0x40012C00;
	*TIMER1_CR1 |= 0b1;
	// RESERT SR
	uint32_t *TIMER1_SR = (uint32_t*) 0x40012C10;
	*TIMER1_SR &= ~(0xfff);
	// WAIT UIF IS SET
	while ((*TIMER1_SR & 1) != 1)
		;

	while (counter < millis) {

		if (*TIMER1_CNT == 1000) {
			counter++;
		}
	}

}


void LED_BLINK() {
// SET GPIO OUTPUT
MY_GPIO_B->CRL &= ~(0xf);
MY_GPIO_B->CRL |= (0b001100110011);
Custom_Delay(500);
MY_GPIO_B->ODR |= 0xffff;
Custom_Delay(500);
char chieu = 0;
for (int var = 0; (var < 3) && (chieu == 0); ++var) {
	MY_GPIO_B->ODR &= ~(1 << var);
	MY_DELAY(500);
	//SET LED OFF
	MY_GPIO_B->ODR |= (1 << var);
	MY_DELAY(500);
	if (var == 2) {
		chieu = 1;
	}
}
for (int varr = 2; (chieu == 1); --varr) {
	MY_GPIO_B->ODR &= ~(1 << varr);
	MY_DELAY(500);
	//SET LED OFF
	MY_GPIO_B->ODR |= (1 << varr);
	MY_DELAY(500);
	if (varr == 0) {
		chieu = 0;
	}

}

}
int main() {
Custom_SysTick_Init();
MY_RCC_INIT("HSI");
// ENABLE CLOCK TIMER1
MY_RCC->APB2ENR |= (0b1 << 11);
MY_RCC->APB2ENR |= (1 << 3);
MY_TIMER1_INIT();
while (1) {
	LED_BLINK();

}
}
