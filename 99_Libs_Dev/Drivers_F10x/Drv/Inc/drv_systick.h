#ifndef __DRV_SYSTICK_H
#define __DRV_SYSTICK_H

#include "stm32f1xx.h"
#include <stdint.h>

/**
 * @brief Initialisiert den SysTick-Timer für 1ms-Interrupts.
 * @brief Verlässt sich darauf, dass 'SystemCoreClock' (aus system_stm32f1xx.c)
 * durch einen Aufruf von bsp_clock_init()
 * oder SystemCoreClockUpdate() korrekt gesetzt wurde.
 */
void drv_systick_init(void);

/**
 * @brief Gibt die Anzahl der Millisekunden seit dem Start zurück.
 * @brief Dies ist eine analoge 'millis()'-Funktion.
 * @return uint32_t Ticks in ms.
 */
uint32_t drv_systick_get_ticks(void);

/**
 * @brief Eine einfache, blockierende Delay-Funktion.
 * @brief Besser ist es, non-blocking mit drv_systick_get_ticks() zu arbeiten!
 * @param delay_ms Die zu wartende Zeit in Millisekunden.
 */
void drv_systick_delay(uint32_t delay_ms);

/**
 * @brief Die "Callback"-Funktion, die vom SysTick-Interrupt aufgerufen wird.
 * @brief Diese Funktion MUSS vom globalen 'SysTick_Handler' in
 * stm32f1xx_it.c aufgerufen werden.
 */
void drv_systick_tick_handler(void);

#endif // __DRV_SYSTICK_H
