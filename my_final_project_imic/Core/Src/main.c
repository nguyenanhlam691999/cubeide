#include<stdint.h>
#include<string.h>

char condition_update = 0;
void MY_RCC_CLOCK_INIT();
void MY_GPIO_USART1_INIT();
void MY_USART1_INIT();
void MY_DMA_INIT();
void MY_EFROM_INIT();
void MY_GPIO_INTERRUPT_INIT();
void MY_INTERRUPT_NVIC_INIT();
void MY_COPY_NVIC()__attribute__((section(".run_in_ram")));
void MY_Flash_Erase(uint32_t address, uint8_t numSector)__attribute__((section(".run_in_ram")));
void MY_Flash_Write(uint32_t address, uint16_t data2Write)__attribute__((section(".run_in_ram")));
void MY_UPDATE()__attribute__((section(".run_in_ram")));

void EXTI15_10_IRQHandler() {
	uint32_t *EXTI_PR = 0x40010414;
	*EXTI_PR |= (1 << 11);
	if (condition_update == 1) {
		// disable sys tick
		uint32_t *STK = (uint32_t*) 0xe000e010;
		*STK &= ~(0B1 << 1);
		MY_UPDATE();
		// reset mcu
		uint32_t *AIRCR = (uint32_t*) 0xE000ED0C;
		*AIRCR = (0x5FA << 16) | (1 << 2);
	}

		condition_update = 1;


}
char buffer[5120] = { 0 };
int main() {
	MY_RCC_CLOCK_INIT();
	MY_GPIO_INTERRUPT_INIT();
	MY_INTERRUPT_NVIC_INIT();
	MY_GPIO_USART1_INIT();
	MY_USART1_INIT();
	MY_DMA_INIT();
	MY_EFROM_INIT();
	MY_COPY_NVIC();
	while (1) {

	}
	return 0;
}
void MY_COPY_NVIC() {
	uint32_t *VTOR = (uint32_t*) 0xE000ED08;
	*VTOR = 0x20000000;
	memcpy(0x20000000, 0x08000000, 304);
}
void MY_RCC_CLOCK_INIT() {
	// SELECT CLOCK SOURCE HSI
	uint32_t *MY_RCC_CR = (uint32_t*) 0x40021000;
	uint32_t *MY_RCC_CFGR = (uint32_t*) 0x40021004;
	*MY_RCC_CFGR |= (0B00) | (0B100 << 24);
	*MY_RCC_CR |= 0B1;
	// ENABLE CLOCK FOR PORT A , DMA , USART1 .AFIO
	uint32_t *MY_RCC_APB2ENR = (uint32_t*) 0x40021018;
	*MY_RCC_APB2ENR |= (0B1 << 2) | (0B1 << 14) | (0B1);
	uint32_t *MY_RCC_AHBENR = (uint32_t*) 0x40021014;
	*MY_RCC_AHBENR |= (0B1) | (0B1 << 4);
}
void MY_GPIO_USART1_INIT() {
	// A9 TX OUTPUT ALTER PUSH-PULL
	// A10 RX INPUT FLOATING
	uint32_t *MY_GPIO_CRH = (uint32_t*) 0x40010804;
	*MY_GPIO_CRH &= ~(0XFF << 4);
	*MY_GPIO_CRH |= (0B1011 << 4) | (0B0100 << 8);
}
void MY_USART1_INIT() {
	//  Baud rate register
	uint32_t *MY_USART1_BRR = (uint32_t*) 0x40013808;
	*MY_USART1_BRR = (52 << 4) | 1;
	// ENABLE TRANS AND RECEV
	uint32_t *MY_USART1_CR1 = (uint32_t*) 0x4001380c;
	*MY_USART1_CR1 |= (0B11 << 2);
	// ENABLE DMA
	uint32_t *MY_USART1_CR3 = (uint32_t*) 0x40013814;
	*MY_USART1_CR3 |= (0B1 << 6);
	// EANBLE USART
	*MY_USART1_CR1 |= (0B1 << 13);
}
void MY_DMA_INIT() {

	// DMA ADDRESS
	uint32_t *MY_DMA_CPAR5 = (uint32_t*) 0x40020060;
	*MY_DMA_CPAR5 = 0x40013804;
	uint32_t *MY_DMA_CMAR5 = (uint32_t*) 0x40020064;
	*MY_DMA_CMAR5 = buffer;
	// DMA NUMBER OF DATA
	uint32_t *MY_DMA_CNDTR5 = (uint32_t*) 0x4002005c;
	*MY_DMA_CNDTR5 = sizeof(buffer);
	// DMA CHANNEL 4
	uint32_t *MY_DMA_CCR5 = (uint32_t*) 0x40020058;
	*MY_DMA_CCR5 |= (0B1 << 5) | (0B1 << 7);
	*MY_DMA_CCR5 |= 0b1;

}
void MY_EFROM_INIT() {
	uint32_t *MY_FLASH_KEYR = (uint32_t*) 0x40022004;
	*MY_FLASH_KEYR = 0x45670123;
	*MY_FLASH_KEYR = 0xCDEF89AB;

}
void MY_INTERRUPT_NVIC_INIT() {
	//Rising trigger selection register (EXTI_RTSR)
	uint32_t *MY_EXTI_RTSR = (uint32_t*) 0x40010408;
	*MY_EXTI_RTSR |= (0B1 << 11);
	uint32_t *IMR = (uint32_t*) 0x40010400;
	*IMR |= (1 << 11);
	//Pending register (EXTI_PR)
	uint32_t *MY_EXTI_PR = (uint32_t*) 0x40010414;
	//* MY_EXTI_PR |= (0B1<<11);
	// ENABLE NVIC_ISER1
	uint32_t *NVIC_ISER1 = (uint32_t*) 0xe000e104;
	*NVIC_ISER1 |= (0B1 << 8);
	// Interrupt Set-pending Registers
	/*uint32_t * MY_NVIC_ISPR1 = (uint32_t *)0xe000e204;
	 * MY_NVIC_ISPR1 |= (0B1<<8);
	 */

}
void MY_Flash_Erase(uint32_t address, uint8_t numSector) {
	uint32_t *FLASH_SR = (uint32_t*) 0x4002200C;
	uint32_t *FLASH_CR = (uint32_t*) 0x40022010;
	uint32_t *FLASH_AR = (uint32_t*) 0x40022014;
	*FLASH_CR |= 1 << 1;
	*FLASH_CR &= ~(0B1);
	for (int i = 0; i < numSector; i++) {
		*FLASH_AR = address + 0x400 * i;
		*FLASH_CR |= 1 << 6;		//Start
		while (((*FLASH_SR >> 5) & 1) != 1)
			;
		*FLASH_SR |= 1 << 5;
	}
	*FLASH_CR &= ~(0B1 << 1);
	*FLASH_CR |= 0B1;
}
void MY_Flash_Write(uint32_t address, uint16_t data2Write) {
	uint32_t *FLASH_CR = (uint32_t*) 0x40022010;
	uint32_t *FLASH_SR = (uint32_t*) 0x4002200C;
	volatile uint16_t *Data;
	*FLASH_CR |= 1;
	Data = address;
	*Data = data2Write;
	*FLASH_CR &= ~1u;
}

void MY_UPDATE() {
	uint32_t *AIRCR = (uint32_t*) 0xE000ED0C;
	MY_Flash_Erase(0x08000000, 5);
	for (int i = 0; i < sizeof(buffer); i += 2) {
		MY_Flash_Write(0x08000000 + i, *(uint16_t*) (buffer + i));
	}
	*AIRCR = (0x5FA << 16) | (1 << 2);
}
void MY_GPIO_INTERRUPT_INIT() {
	uint32_t *MY_PORTA11_CRH = (uint32_t*) 0x40010804;
	*MY_PORTA11_CRH &= ~(0xf << 12);
	*MY_PORTA11_CRH |= (0B1000 << 12);
	uint32_t *ODR = 0x4001080c;
	*ODR |= (1 << 11);
	uint32_t *MY_PORTA11_EXTICR3 = (uint32_t*) 0x40010010;
	*MY_PORTA11_EXTICR3 &= ~(0xf << 12);

}
