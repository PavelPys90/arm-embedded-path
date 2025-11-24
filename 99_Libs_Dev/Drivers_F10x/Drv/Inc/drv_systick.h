#ifndef __DRV_SYSTICK_H
#define __DRV_SYSTICK_H

#include "stm32f1xx.h"
#include <stdint.h>

/**
 * @brief Initializes the SysTick timer for 1ms interrupts.
 * @brief Relies on 'SystemCoreClock' (from system_stm32f1xx.c)
 * being correctly set by a call to bsp_clock_init()
 * or SystemCoreClockUpdate().
 */
void drv_systick_init(void);

/**
 * @brief Returns the number of milliseconds since startup.
 * @brief This is an analogous 'millis()' function.
 * @return uint32_t Ticks in ms.
 */
uint32_t drv_systick_get_ticks(void);

/**
 * @brief A simple, blocking delay function.
 * @brief It's better to work non-blocking with drv_systick_get_ticks()!
 * @param delay_ms The time to wait in milliseconds.
 */
void drv_systick_delay(uint32_t delay_ms);

/**
 * @brief The "callback" function that is called by the SysTick interrupt.
 * @brief This function MUST be called by the global 'SysTick_Handler' in
 * stm32f1xx_it.c.
 */
void drv_systick_tick_handler(void);

#endif // __DRV_SYSTICK_H
