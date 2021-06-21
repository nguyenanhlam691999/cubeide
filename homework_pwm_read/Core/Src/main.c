#include <stdint.h>
float b=0;
void MY_CLOCK_INTER_INIT() {
	// CHOOSE SOURCE CLOCK INTERNAL
	uint32_t *MY_RCC_CFGR = (uint32_t*) 0x40021004;
	*MY_RCC_CFGR |= (0b00);
	uint32_t *MY_RCC_CR = (uint32_t*) 0x40021000;
	*MY_RCC_CR |= (1 << 0);
	// ENABLE CLOCK FOR PORT A , TIMER1
	uint32_t *MY_RCC_APB2ENR = 0x40021018;
	*MY_RCC_APB2ENR |= (0b1 << 11);
	*MY_RCC_APB2ENR |= (0b1 << 3) | (0b1 << 2) | (0b1);
}
void MY_GPIO_INIT() {
	// Input floating

	// GPIO A8
	uint32_t *GPIOA_CRH = (uint32_t*) 0x40010804;
	*GPIOA_CRH = 0B0100;
}
void MY_TIMER1_PWM_INPU_INIT() {
	/*For example, user can measure the period (in TIMx_CCR1 register) and the duty cycle (in
	 TIMx_CCR2 register) of the PWM applied on TI1 using the following procedure (depending
	 on CK_INT frequency and prescaler value):
	 • Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1
	 register (TI1 selected).
	 • Select the active polarity for TI1FP1 (used both for capture in TIMx_CCR1 and counter
	 clear): write the CC1P bit to ‘0’ (active on rising edge).
	 • Select the active input for TIMx_CCR2: write the CC2S bits to 10 in the TIMx_CCMR1
	 register (TI1 selected).
	 • Select the active polarity for TI1FP2 (used for capture in TIMx_CCR2): write the CC2P
	 bit to ‘1’ (active on falling edge).
	 • Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register
	 (TI1FP1 selected).
	 • Configure the slave mode controller in reset mode: write the SMS bits to 100 in the
	 TIMx_SMCR register.
	 • Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register*/
	// SET PSC AND ARR FOR TIMER 1
	uint32_t *TIMER1_PSC = (uint32_t*) 0x40012c28;
	uint32_t *TIMER1_ARR = (uint32_t*) 0x40012c2c;
	*TIMER1_PSC = 8000;
	*TIMER1_ARR = 0XFFFF;
	/* Select the active input for TIMx_CCR1: write the CC1S bits to 01 in the TIMx_CCMR1
	 register (TI1 selected).*/
	uint32_t *TIMER1_CCMR1 = (uint32_t*) 0x40012c18;
	*TIMER1_CCMR1 |= (0b01);
	/*• Select the valid trigger input: write the TS bits to 101 in the TIMx_SMCR register
	 (TI1FP1 selected).
	 • Configure the slave mode controller in reset mode: write the SMS bits to 100 in the
	 TIMx_SMCR register.
	 • Enable the captures: write the CC1E and CC2E bits to ‘1’ in the TIMx_CCER register*/
	uint32_t *TIMER1_SMCR = (uint32_t) 0x40012c08;
	uint32_t *TIMER1_CCER = (uint32_t) 0x40012c20;
	*TIMER1_SMCR |= (0B100) | (0B101 << 4);
	*TIMER1_CCER |= (0B1);
	uint32_t *TIMER1_CR1 = (uint32_t) 0x40012c00;
	*TIMER1_CR1 |= 0B1;

}
int main() {
	MY_CLOCK_INTER_INIT();
	MY_GPIO_INIT();
	MY_TIMER1_PWM_INPU_INIT();
	uint32_t * TIMER1_CCR1 = 0x40012c34;

	while (1) {
		b = *TIMER1_CCR1 * 0.001;



	}
}

