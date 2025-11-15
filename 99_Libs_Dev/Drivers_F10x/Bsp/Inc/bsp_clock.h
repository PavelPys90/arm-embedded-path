#ifndef __BSP_CLOCK_H
#define __BSP_ClOCK_H

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h> // Für 'true'/'false' Rückgabewerte

/**
 * @brief Definiert die verfügbaren Takt-Profile für unser Board.
 */
typedef enum{
	CLOCK_PROFILE_HSI_8MHZ_DEFAULT, // Der Standard nach Reset
	CLOCK_PROFILE_HSI_16MHZ_PLL,    // Dein HSI-Testfall
	CLOCK_PROFILE_HSE_72MHZ_PLL     // Der "Blue Pill" Standard (HSE 8MHz * 9)
	// (Hier könnte man später CLOCK_PROFILE_HSE_16MHZ_PLL etc. hinzufügen)
}bsp_clock_profile_t;

/**
 * @brief Initialisiert die MCU-Clocks (SYSCLK, HCLK, PCLKs) auf ein definiertes Profil.
 * @brief MUSS als allererste Funktion in main() aufgerufen werden!
 * @brief Ruft intern SystemCoreClockUpdate() auf, um die globale Variable
 * 'SystemCoreClock' für alle anderen Treiber (SysTick, UART) zu setzen.
 *
 * @param profile Das gewünschte Takt-Profil.
 * @return true bei Erfolg, false wenn die Konfiguration fehlgeschlagen ist
 * (z.B. HSE-Kristall schwingt nicht an).
 */
bool bsp_clock_init(bsp_clock_profile_t profile);

#endif // __BSP_CLOCK_H
