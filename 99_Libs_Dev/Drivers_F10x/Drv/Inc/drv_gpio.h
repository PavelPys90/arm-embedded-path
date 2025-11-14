#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

// Import der Register-Definition (GPIO_TypeDef, RCC)
#include "stm32f1xx.h"
#include <stdint.h> // Notwendig z.B. für uint8_t usw.

/**
 * GPIO Mode Konfiguration (Anhand des RM0008) - Vorteil man muss die einzelnen Modes nicht mehr nachschlagen
 * Namen sind sauber definiert und können belibig erweitert werden.
 */
typedef enum{
	// --- Input-Mode ---
	GPIO_MODE_INPUT_ANALOG			= 0b0000, 	// Analog
	GPIO_MODE_INPUT_FLOATING 		= 0b0100, 	// Input Floating (default)
	GPIO_MODE_INPUT_PULL 			= 0b1000, 	// Input, Pull-Up/Pull-Down

	// --- Output-Mode
	GPIO_MODE_OUTPUT_PP_10MHZ 		= 0b0001, 	// Output Push-Pull 10MHz
	GPIO_MODE_OUTPUT_PP_2MHZ 		= 0b0010, 	// Output Push-Pull 2MHz
	GPIO_MODE_OUTPUT_PP_50MHZ		= 0b0011,	// Output Push-Pull 50MHz

	GPIO_MODE_OUTPUT_OD_10MHZ		= 0b0101,	// Output Open-Drain 10MHz
	GPIO_MODE_OUTPUT_OD_2MHZ		= 0b0110,	// Output Open-Drain 2MHz
	GPIO_MODE_OUTPUT_OD_50MHZ		= 0b0111, 	// Output Open-Drain 50MHz

	// --- Alternate-Functions (For UART, I2C, SPI...) ---
	GPIO_MODE_AF_PP_50MHZ			= 0b1011,	// AF Push-Pull 50 MHz
	GPIO_MODE_AF_OD_50MHZ			= 0b1111	// AF Open-Drain 50MHz
}drv_gpio_mode_t;

/**
 * @brief Init eines einzelnen GPIO-Pins
 * @breif verschachtelt die komplexe CRL/CRH-Logik des STM32F1
 *
 * @param port Port des Pins (A,B oder C) z.B. GPIOA <-- Port A
 * @param pin_num Nummer des Pins. (0-15)
 * @param mode Der gewünschte Mode mit drv_gpio_mode_t
 */
void drv_gpio_init(GPIO_TypeDef* port, uint8_t pin_num, drv_gpio_mode_t mode);
/**
 * @brief Setzt einen Pin (zieht ihn auf HIGH). (Inline für max. Geschwindigkeit)
 */
static inline void drv_gpio_set(GPIO_TypeDef* port, uint8_t pin_num){
	// Register BSRR ist atomar (Interrupt-sicher) siehe: "RM0008 Rev-21 9.2.5"
	port->BSRR = (1U << pin_num);
}
/**
 * @brief Löscht einen Pin (zieht ihn auf LOW.)
 */
static inline void drv_gpio_clear(GPIO_TypeDef* port, uint8_t pin_num){
	// Register BRR ist atomar (Interrupt-sicher) siehe: "RM0008 Rev-21 9.2.6"
	port->BRR = (1U << pin_num);
}
/**
 * @brief Liest den Input-Status eines Pins. (Inline)
 * @return 1 wenn HIGH, 0 wenn LOW.
 */
static inline uint8_t drv_gpio_read(GPIO_TypeDef* port, uint8_t pin_num){
	// Lesen des Input Data Registers (IDR) siehe: "RM0009 Rev-21 9.2.3"
	return (port->IDR & (1U << pin_num)) ? 1U:0U;
}
#endif // __DRV_GPIO_H






