/*
 * my_define.h
 *
 *  Created on: May 22, 2021
 *      Author: ZBOOK
 */

#ifndef SRC_MY_DEFINE_H_
#define SRC_MY_DEFINE_H_

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
#endif /* SRC_MY_DEFINE_H_ */
