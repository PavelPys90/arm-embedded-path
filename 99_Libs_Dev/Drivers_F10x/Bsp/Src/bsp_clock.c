#include "bsp_clock.h"

// Ein Timeout für das Warten auf Oszillatoren (verhindert Endlos-Loops)
// Ca. 50.000 Zyklen.
#define CLOCK_TIMEOUT_VALUE 0xFFFU

/**
 * @brief Private Funktion: Konfiguriert HSI -> 16MHz PLL (Zum Testen)
 */
static bool clock_config_hsi_16mhz(void){
	// 1. HSI an und warten (RM0008 Rev-21 8.3.1)
	RCC->CR |= RCC_CR_HSION;
	uint32_t timeout = CLOCK_TIMEOUT_VALUE;
	while (!(RCC->CR & RCC_CR_HSIRDY)){			// (RM0008 Rev-21 8.3.1)
		if (--timeout == 0) return false; 		// HSI Fehler
	}

	// 2. PLL aus und warten (robust!)
	RCC->CR &= ~RCC_CR_PLLON;					// (RM0008 Rev-21 8.3.1)
	timeout = CLOCK_TIMEOUT_VALUE;
	while (RCC->CR & RCC_CR_PLLRDY){			// (RM0008 Rev-21 8.3.1)
		if (--timeout == 0) return false;		// Kann nicht stoppen?!
	}

	// 3. Flash Latenz (0 WS für <= 24MHz)
	FLASH->ACR &= ~FLASH_ACR_LATENCY;			// 0 Wait States (RM0008 Rev-21 3.3.3)
	FLASH->ACR |= FLASH_ACR_PRFTBE;				// Prefetch an (RM0008 Rev-21 3.3.3)

	// 4. Prescaler (alle auf /1)
	RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2); // (RM0008 Rev-21 7.3.2)

	// 5. PLL Konfiguration: HSI/2 als Quelle, Multiplikator x4
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);				// (RM0008 Rev-21 7.3.2)
	RCC->CFGR |= (0b0010<<18);										// PLLMULL = x4 (RM0008 Rev-21 7.3.2)

	// 6. PLL an und warten
	RCC->CR |= RCC_CR_PLLON;					// (RM0008 Rev-21 8.3.1)
	timeout = CLOCK_TIMEOUT_VALUE;
	while (!(RCC->CR & RCC_CR_PLLRDY)){
		if (--timeout == 0) return false; 		// PLL Lock-Fehler
	}

	// 7. Auf PLL als System-Takt umschalten
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	return true; // Erfolg
}
/**
 * @brief Private Funktion: Konfiguriert HSE -> 72MHz PLL (Standard)
 */
static bool clock_config_hse_72mhz(void) {
    // 1. HSE (External Crystal) einschalten
    RCC->CR |= RCC_CR_HSEON;
    uint32_t timeout = CLOCK_TIMEOUT_VALUE;
    while (!(RCC->CR & RCC_CR_HSERDY)) {
        if (--timeout == 0) return false; // HSE-Fehler! (Kristall defekt?)
    }

    // 2. WICHTIG: Flash-Latenz für 72 MHz einstellen (2 Wait States)
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2 | FLASH_ACR_PRFTBE;

    // 3. Bus-Prescaler
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 (max 36MHz) muss /2 sein!

    // 4. PLL Konfiguration: HSE als Quelle, Multiplikator x9
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PLLSRC;     // Bit 16: HSE als Quelle
    RCC->CFGR |= (0b0111 << 18);      // PLLMULL = x9

    // 5. PLL einschalten
    RCC->CR |= RCC_CR_PLLON;
    timeout = CLOCK_TIMEOUT_VALUE;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        if (--timeout == 0) return false; // PLL Lock-Fehler!
    }

    // 6. Auf PLL umschalten
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    return true; // Erfolg
}
/**
 * @brief Die öffentliche API-Funktion (der "Dispatcher")
 */
bool bsp_clock_init(bsp_clock_profile_t profile) {
    bool success = false;

    switch (profile) {
        case CLOCK_PROFILE_HSI_16MHZ_PLL:
            success = clock_config_hsi_16mhz();
            break;

        case CLOCK_PROFILE_HSE_72MHZ_PLL:
            success = clock_config_hse_72mhz();
            break;

        case CLOCK_PROFILE_HSI_8MHZ_DEFAULT:
        default:
            // (Code, um sicher auf HSI 8MHz zurückzufallen)
            RCC->CR |= RCC_CR_HSION;
            while (!(RCC->CR & RCC_CR_HSIRDY));
            RCC->CFGR &= ~RCC_CFGR_SW; // 00 = HSI
            while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
            success = true;
            break;
    }

    // **DER VERTRAG**
    // Aktualisiere die globale CMSIS-Variable 'SystemCoreClock',
    // auf die sich ALLE anderen Treiber (UART, SysTick) verlassen.
    // Diese Funktion lebt in 'system_stm32f1xx.c'
    if (success) {
        SystemCoreClockUpdate();
    }

    return success;
}














