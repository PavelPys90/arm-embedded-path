/*
 *
 * main.c
 * Goal:
 * Clock: HSI 8MHz (default)
 *
 */

// Include
# include "stm32f103x6.h"

// Global Variables
extern volatile uint32_t g_msTicks;
typedef enum{
	MODE_BLINKYALL,
	MODE_ALARM_FAST
}LED_Mode_t;

void LED_Init(void);
void LED_On(uint8_t LEDNumber);
void LED_Off(uint8_t LEDNumber);
void LED_Modus(LED_Mode_t mode, uint32_t delay);
void Delay_ms(uint32_t ms);

int main(void){
	// Sys Init
	SysTick_Config(SystemCoreClock / 1000);
	// Peri-Init
	LED_Init();
	LED_Off(0);
	while (1){
		LED_Modus(MODE_ALARM_FAST, 500);
	}
}
// Inplementations

void Delay_ms(uint32_t ms){
	uint32_t start = g_msTicks;
	while ((g_msTicks-start)<ms){
		// ___WFI();
	}
}

void LED_Init(void){
	//Clock
	RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN|
			RCC_APB2ENR_IOPBEN|
			RCC_APB2ENR_IOPAEN);
//	RCC->APB2ENR |= ((1<<4)|
//			(1<<3)|
//			(1<<2));
	// Bit reset
	GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
	GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
	GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
	GPIOB->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
	// GPIOC->CRH &= ~(0xF<<20);
	// GPIOA->CRL &= ~(0xF<<0);
	// GPIOA->CRH &= ~(0xF<<16);
	// GPIOB->CRH &= ~(0xF<<4);

	// Set Bits
	GPIOC->CRH |= GPIO_CRH_MODE13_1;
	GPIOA->CRL |= GPIO_CRL_MODE0_1;
	GPIOA->CRH |= GPIO_CRH_MODE10_1;
	GPIOB->CRH |= GPIO_CRH_MODE9_1;

	//GPIOC->CRH |= (0x2<<20);
	//GPIOA->CRL |= (0x2<<0);
	//GPIOA->CRH |= (0x2<<16);
	//GPIOB->CRH |= (0x2<<4);
}
void LED_On(uint8_t LEDNumber){
	if (LEDNumber == 0)
	{
		GPIOC->BSRR = GPIO_BSRR_BR13;
	}
	if (LEDNumber == 1)
	{
		GPIOA->BSRR = GPIO_BSRR_BS0;
	}
	if (LEDNumber == 2)
	{
		GPIOA->BSRR = GPIO_BSRR_BS10;
	}
	if (LEDNumber == 3)
	{
		GPIOB->BSRR = GPIO_BSRR_BS9;
	}
}

void LED_Off(uint8_t LEDNumber){
	if (LEDNumber == 0)
	{
		GPIOC->BSRR = GPIO_BSRR_BS13;
	}
	if (LEDNumber == 1)
	{
		GPIOA->BSRR = GPIO_BSRR_BR0;
	}
	if (LEDNumber == 2)
	{
		GPIOA->BSRR = GPIO_BSRR_BR10;
	}
	if (LEDNumber == 3)
	{
		GPIOB->BSRR = GPIO_BSRR_BR9;
	}
}
void LED_Modus(LED_Mode_t mode, uint32_t delay){
	switch (mode){
		case MODE_BLINKYALL:
			// Active High
			GPIOA->BSRR = (GPIO_BSRR_BS0 | GPIO_BSRR_BS10);
			GPIOB->BSRR = GPIO_BSRR_BS9;
			// Active low
			GPIOC->BSRR = GPIO_BSRR_BR13;

			Delay_ms(delay);
			// Active High
			GPIOA->BSRR = (GPIO_BSRR_BR0 | GPIO_BSRR_BR10);
			GPIOB->BSRR = GPIO_BSRR_BR9;
			// Active low
			GPIOC->BSRR = GPIO_BSRR_BS13;
			Delay_ms(delay);

			break;


		case MODE_ALARM_FAST:
			uint32_t alarmDelay = 250;
			LED_On(0); LED_On(1); LED_On(2); LED_On(3);
			Delay_ms(alarmDelay);
			LED_Off(0);LED_Off(1);LED_Off(2);LED_Off(3);
			Delay_ms(alarmDelay);

			break;
	}

}





