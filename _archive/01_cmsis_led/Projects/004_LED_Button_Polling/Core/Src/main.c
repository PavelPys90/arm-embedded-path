/*
 * main.c (Projekt 6: Non-Blocking Polling)
 * Ziel: LED auf Knopfdruck auszuschalten
 * ein Taster eingelesen wird.
 * Framework: CMSIS/Bare-Metal
 * MCU: STM32F103C6
 */

#include "stm32f103x6.h"
#include <stdint.h>

// Variablen
extern volatile uint32_t g_msTicks;

// Prototypen
void GPIO_Init(void);
void Set_LED_On(void);
void Set_LED_Off(void);


int main(void){
	SysTick_Config(SystemCoreClock/1000);
	GPIO_Init();
	while (1){
		// if ((GPIOB->IDR & GPIO_IDR_IDR0)==0)
		if ((GPIOB->IDR & (0x1<<0))==0)
		{
			Set_LED_On();
		}
		else
		{
			Set_LED_Off();
		}
	}
}

// Inplementierungen
void GPIO_Init(void){
	// Takt für PORT C und B
	RCC->APB2ENR |= (0x1<<3 | 0x1<<4);
	// Alternativ mit Makro von CMSIS
	// RCC->APB2ENR |= (RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);

	// Bit Reset
	// Port B:Input MODE=1000
	GPIOB->CRL &= ~(0xF<<0);
	// Alternativ mit Makro von CMSIS
	//GPIOB->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	// Port C:
	GPIOC->CRH &= ~(0xF<<20);
	// Alternativ mit Makro von CMSIS
	// GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
	// Port Configuration
	// Port B:Input MODE=1000 0x8
	GPIOB->CRL |= (0x8<<0);
	// GPIOB->CRL |= GPIO_CRL_CNF0_1;
	// Auf Push-pull UP über ODR Register (Output Data Register 1= Up; 0= Down)
	GPIOB->ODR |= (0x1<<0);
	// GPIOB->ODR |= GPIO_ODR_ODR0;

	// Port C: CNF=00 (General purpose output push-pull); MODE=10 (Output: 2MHz)
	// 0010 --> 0x2
	GPIOC->CRH |= (0x2<<20);
	// GPIOC->CRH |= GPIO_CRH_MODE13_1;

}
void Set_LED_On(void){
	GPIOC->BSRR |= (0x1<<29);
	// Alternativ mit Makro von CMSIS
	// GPIOC->BSRR |= GPIO_BSRR_BR13;
}
void Set_LED_Off(void){
	GPIOC->BSRR |= (0x1<<13);
	// Alternativ mit Makro von CMSIS
	// GPIOC->BSRR |= GPIO_BSRR_BS13;
}


