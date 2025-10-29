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

void LED_Init(void);
void LED_On(uint8_t LEDNumber);
void LED_Off(uint8_t LEDNumber);

int main(void){
	// Sys Init
	SysTick_Config(SystemCoreClock / 1000);
	// Peri-Init
	LED_Init();

	while (1){
		// ...
	}
}
// Inplementations

void Delay_ms(uint32_t ms){
	//...
}

void LED_Init(void){
	//...
}
void LED_On(uint8_t LEDNumber){
	// ...
}

void LED_Off(uint8_t LEDNumber){
	// ...
}





