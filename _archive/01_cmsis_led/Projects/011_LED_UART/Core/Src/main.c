// FILE HEADER

// Includes
#include "stm32f103x6.h"
#include "stdint.h"

// Define

//Type define

//Variables
extern volatile uint32_t g_msTicks;
static uint32_t blink_last_tick = 0;
static int blinker_state = 0; // 0=aus; 1=ein
// Prototypes
void GPIO_Init(void);
void LED_State(void);
void UART_Init(void);
void UART_Sent_LED_Status(void);
// Main Function
int main(void){

	// Inits
	SysTick_Config(SystemCoreClock / 1000);
	GPIO_Init();
	while(1){
		LED_State();
	}
}

// Functions
void GPIO_Init(void){
	// Clock activation
	RCC->APB2ENR |= (0x1<<4);
	// Bit reset
	GPIOC->CRH &= ~(0b1111<<20);
	// Bit configuration
	GPIOC->CRH |= (0b0010<<20);
}
void LED_State(void){
	// 1: Ist die Zeit schon um?
	if((g_msTicks - blink_last_tick)<500){
		return;
	}
	blink_last_tick = g_msTicks;
	if(blinker_state == 0){
		GPIOC->BSRR = (0x1<<29);
		blinker_state = 1;
	}
	else{
		GPIOC->BSRR = (0x1<<13);
		blinker_state = 0;
	}
}
void UART_Init(void){

}
void UART_Sent_LED_Status(void){

}









