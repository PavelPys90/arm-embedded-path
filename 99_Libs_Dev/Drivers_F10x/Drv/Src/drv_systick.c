#include "drv_systick.h"
// Private, global tick variable.
// 'static' = visible only in this file.
// 'volatile' = IMPORTANT! Tells the compiler that this variable
//              can change at any time (through an interrupt).
static volatile uint32_t g_ms_ticks = 0;

/**
 * @brief Initializes the SysTick for 1ms ticks.
 */
void drv_systick_init(void){
	// Calls the CMSIS-Core function.
	// SystemCoreClock / 1000 = Ticks for 1ms.
	// The function requires knowledge of the clock speed.
	if (SysTick_Config(SystemCoreClock / 1000U) != 0U){
		// Error condition that should never occur
		while(1);
	}
	// Sets the interrupt priority (optional, but good practice)
	// (Priorities on F1 are complex, 0 is the highest)
	NVIC_SetPriority(SysTick_IRQn, 0);
}
/**
 * @brief Returns the current ticks "atomically".
 */
uint32_t drv_systick_get_ticks(void){
	uint32_t ticks;
	// Critical section: Prevents the interrupt
	// from triggering while reading the variable.
	__disable_irq();
	ticks = g_ms_ticks;
	__enable_irq();

	return ticks;
}
/**
 * @brief Blocking delay function.
 */
void drv_systick_delay(uint32_t delay_ms){
	uint32_t start_time = drv_systick_get_ticks();

	// Waits until the time difference has elapsed
	// (This loop blocks the CPU)
	while ((drv_systick_get_ticks() - start_time) < delay_ms){
		// Does nothing (or __WFI() for "Wait For Interrupt")
		// __WFI(); // Power saving
	}
}
/**
 * @brief The "callback" that is called by the real ISR.
 */
void drv_systick_tick_handler(void){
	g_ms_ticks++;
}











