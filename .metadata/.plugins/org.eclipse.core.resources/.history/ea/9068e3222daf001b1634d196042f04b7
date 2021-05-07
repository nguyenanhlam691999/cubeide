/*
 * main.c
 *
 *  Created on: Oct 6, 2020
 *      Author: maitrongnhan
 */
#include <stdint.h>
#include <string.h>

void Custom_Clock_Init();
void Custom_SysTick_Init();
void Custom_Delay(uint32_t milisec);
void Custom_Uart_Init();
void Custom_Uart_Write_Char(uint8_t data);
void Custom_Uart_Write_String(uint8_t* str);
void Custom_Uart_DMA_Enable();
void Custom_Uart_Interrupt_Enable();
uint8_t Custom_Uart_Read_Char();
uint8_t data_recv[128] = {0};
uint8_t test = 0;
int main()
{
	Custom_Clock_Init();
	Custom_SysTick_Init();
	Custom_Uart_Init();
	//Custom_Uart_DMA_Enable();
	Custom_Uart_Interrupt_Enable();
	Custom_Uart_Write_String((uint8_t*)"hello world\r\n");
	while(1)
	{


	}
	return 0;
}

void Custom_Clock_Init()
{
	uint32_t* RCC_CR = (uint32_t*)0x40021000;
	*RCC_CR = (*RCC_CR & ~(uint32_t)(1))|1;

	uint32_t* RCC_CFGR = (uint32_t*)(0x40021000 + 0x04);
	*RCC_CFGR = (*RCC_CFGR & ~(uint32_t)3);

	uint32_t* RCC_APB2ENR = (uint32_t*)(0x40021000 + 0x18);
	uint32_t* RCC_AHBENR = (uint32_t*)(0x40021014);
	*RCC_APB2ENR = (*RCC_APB2ENR & ~(uint32_t)0x08) | 0x08;
	*RCC_APB2ENR = (*RCC_APB2ENR & ~(uint32_t)0x04) | 0x04;
	*RCC_APB2ENR |= 1<<2;		//enable clock for PORTA
	*RCC_APB2ENR |= 1<<14;		//enable clock for UART1
	*RCC_AHBENR |= 1;			//enable clock for DMA1
}

void Custom_SysTick_Init()
{
	uint32_t* SYST_CSR = (uint32_t*)(0xE000E010);
	uint32_t* SYST_RVR = (uint32_t*)(0xE000E014);
	*SYST_CSR = (*SYST_CSR & ~(uint32_t)0x0007) | 7;
	*SYST_RVR = 8000;
}

void Custom_Delay(uint32_t milisec)
{
	uint32_t* SYST_CSR = (uint32_t*)(0xE000E010);
	*SYST_CSR = (*SYST_CSR & ~(uint32_t)0x0001) | 1;
	uint32_t cnt = 0;
	while(cnt < milisec)
	{
		if(((*SYST_CSR & (uint32_t)0x10000) >> 16) == 1)
			cnt++;
	}
	*SYST_CSR = (*SYST_CSR & ~(uint32_t)0x0001);
}

void Custom_Uart_Init()
{
	uint32_t *GPIOA_CRH = (uint32_t*)(0x40010804);
	uint32_t *USART_BRR = (uint32_t*)(0x40013808);
	uint32_t *USART_CR1 = (uint32_t*)(0x4001380C);

	*GPIOA_CRH &= 0xFFFFF00F;
	*GPIOA_CRH |= (3 << 6) | (3 << 4);	//set Tx as Alternate function output open-dair
	*GPIOA_CRH |= (1 << 10);			//set Rx as Floating input (reset state)
	/*Set baud rate for UART1
	 * UART_BRR = Fuart  / (16 * BaudRate)
	 * 9600 ==> UART_BRR = 8000000 / (16 * 9600) = 52.08 ~ 52.1
	 */


	*USART_BRR = 52<<4 | 1; //Set BaudRate in 9600 bit/sec
	*USART_CR1 |= 1<<3;		//Enable transmitter
	*USART_CR1 |= 1<<2; 	//Enable receiver
	*USART_CR1 |= 1<<13;	//Enable UART
}

void Custom_Uart_Write_Char(uint8_t data)
{
	uint32_t *USART_DR = (uint32_t*)(0x40013804);
	uint32_t *USART_SR = (uint32_t*)(0x40013800);

	while(((*USART_SR>>7)&1) !=1);			//wait TXE (Transmit data register empty)
	*USART_DR = data & 0x000000ff;			//write data to USART_DR
	while(((*USART_SR>>6)&1) !=1);			//wait TC (Transmission complete)
	*USART_SR &= ~(uint32_t)(1<<6);
}

uint8_t Custom_Uart_Read_Char()
{
	uint32_t *USART_DR = (uint32_t*)(0x40013804);
	uint32_t *USART_SR = (uint32_t*)(0x40013800);
	uint8_t result = 0;


	while(((*USART_SR>>5)&1) !=1);			//wait RXNE flag is set (Read data register not empty)
	result = (uint8_t)(*USART_DR);			//Read data to USART_DR
	return result;
}

void Custom_Uart_Write_String(uint8_t* str)
{
	uint32_t *USART_DR = (uint32_t*)(0x40013804);
	uint32_t *USART_SR = (uint32_t*)(0x40013800);


	uint8_t index = 0;
	while(str[index] != 0)
	{
		while(((*USART_SR>>7)&1) !=1);			//wait TXE (Transmit data register empty)
		*USART_DR = str[index] & 0x000000ff;	//write data to USART_DR
		while(((*USART_SR>>6)&1) !=1);			//wait TC (Transmission complete)
		*USART_SR &= ~(uint32_t)(1<<6);
		index++;
	}
}

void Custom_Uart_DMA_Enable()
{
	//Use DMA channel 5 to receive data

	uint32_t *DMA_CCR5 = (uint32_t*)(0x40020058);
	uint32_t *DMA_CNDTR5 = (uint32_t*)(0x4002005C);
	uint32_t *DMA_CPAR5 = (uint32_t*)(0x40020060);
	uint32_t *DMA_CMAR5 = (uint32_t*)(0x40020064);
	uint32_t *USART_CR3 = (uint32_t*)(0x40013814);
	*USART_CR3 |= 1<<6; 						//DMA enable receiver

	// ***note: Set all before enable DMA channel 5
	*DMA_CNDTR5 = sizeof(data_recv);			//number of data register
	*DMA_CMAR5 = (uint32_t)data_recv;			//memory address register
	*DMA_CPAR5 = (0x40013804); 					//peripheral address register

	/*
	 * Enable channel 5
	 * Enable Circular mode
	 * Enable Memory increment mode
	 */
	*DMA_CCR5 = (1<<0)|(1<<5)|(1<<7);
}

void Custom_Uart_Interrupt_Enable()
{
	uint32_t *USART_CR1 = (uint32_t*)(0x4001380C);
	uint32_t *NVIC_ISER1 = (uint32_t*)(0xE000E104);

	*USART_CR1 |= 1<<5;			//RXNE interrupt enable
	*NVIC_ISER1 |= 1<<(37-32);	//Enable interrupt at position 37 (UART1 global)

}

void USART1_IRQHandler()
{
	test = Custom_Uart_Read_Char();
}
