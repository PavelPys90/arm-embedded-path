#include "stm32f1xx.h" // CMSIS Device Header für STM32F1
#include <stdint.h>

volatile uint32_t msTicks = 0; // 1 ms Tick-Zähler

// SysTick ISR
void SysTick_Handler(void) {
    msTicks++;
}

static void Delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms) {
        __NOP();
    }
}

int main(void) {
    // SystemInit() kommt aus dem Startup-File vor main(), HSI = 8 MHz aktiv
    // Optional: SystemCoreClock updaten, falls du später die Clock änderst
    // SystemCoreClockUpdate();

    // --- GPIOC Takt an ---
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

    // --- PC13 als Push-Pull Output @2MHz ---
    // PC13 liegt in CRH (PINs 8..15), Mode/CNF sind je 2 Bit
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13); // clear
    GPIOC->CRH |=  (0x2U << GPIO_CRH_MODE13_Pos);      // MODE13 = 10 (2 MHz), CNF13 = 00

    // --- SysTick auf 1 ms konfigurieren ---
    // CMSIS-Helfer: erzeugt IRQ bei Reload und setzt CLKSOURCE = core
    SysTick_Config(SystemCoreClock / 1000U);

    // Blue Pill: LED an PC13 ist "Low-aktiv" (0 = an, 1 = aus)
    while (1) {
        // LED AN
        GPIOC->BSRR = GPIO_BSRR_BR13;  // Reset Bit -> Pegel LOW
        Delay_ms(1000);

        // LED AUS
        GPIOC->BSRR = GPIO_BSRR_BS13;  // Set Bit -> Pegel HIGH
        Delay_ms(1000);
    }
}
