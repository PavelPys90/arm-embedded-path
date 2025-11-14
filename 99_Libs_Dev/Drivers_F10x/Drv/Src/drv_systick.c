#include "drv_systick.h"
// Private, globale Tick-Variable.
// 'static' = nur in dieser Datei sichtbar.
// 'volatile' = WICHTIG! Sagt dem Compiler, dass sich diese Variable
//              jederzeit (durch einen Interrupt) ändern kann.
static volatile uint32_t g_ms_ticks = 0;

/**
 * @brief Initialisiert den SysTick für 1ms Ticks.
 */
void drv_systick_init(void){
	// Ruft die CMSIS-Core-Funktion auf.
	// SystemCoreClock / 1000 = Ticks für 1ms.
	// Diese Funktion MUSS wissen, wie schnell der Takt ist!
	if (SysTick_Config(SystemCoreClock / 1000U) != 0U){
		// Der Fehler, sollte nie passieren!
		while(1);
	}
	// Setze die Interrupt-Priorität (optional, aber gute Praxis)
	// (Prioritäten auf F1 sind komplex, 0 ist die höchste)
	NVIC_SetPriority(SysTick_IRQn, 0);
}
/**
 * @brief Gibt die aktuellen Ticks "atomar" zurück.
 */
uint32_t drv_systick_get_ticks(void){
	uint32_t ticks;
	// "Kritischer Abschnitt": Verhindert, dass der Interrupt
	// zuschlägt, während die Variable gelesen wird.
	__disable_irq();
	ticks = g_ms_ticks;
	__enable_irq();

	return ticks;
}
/**
 * @brief Blockierende Delay-Funktion.
 */
void drv_systick_delay(uint32_t delay_ms){
	uint32_t start_time = drv_systick_get_ticks();

	// Warte, bis die Differenzzeit abgelaufen ist
	// (Diese Schleife blockiert die CPU!)
	while ((drv_systick_get_ticks() - start_time) < delay_ms){
		// Nichts tun (oder __WFI() für "Wait For Interrupt")
		// __WFI(); // strom sparend
	}
}
/**
 * @brief Der "Callback", der vom echten ISR aufgerufen wird.
 */
void drv_systick_tick_handler(void){
	g_ms_ticks++;
}











