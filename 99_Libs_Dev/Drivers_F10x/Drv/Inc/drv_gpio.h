#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

// Import of register definitions (GPIO_TypeDef, RCC)#include "stm32f1xx.h"
#include "stm32f1xx.h"
#include <stdint.h> // Required e.g. for uint8_t etc.

/**
 * GPIO mode configuration (based on RM0008) - Advantage: you no longer have to look up each mode
 * Names are clearly defined and can be extended arbitrarily.
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
 * @brief Init of a single GPIO pin
 * @breif encapsulates the complex CRL/CRH logic of the STM32F1
 *
 * @param port Port of the pin (A, B or C) e.g. GPIOA <-- Port A
 * @param pin_num Pin number (0–15)
 * @param mode Desired mode using drv_gpio_mode_t
 */
void drv_gpio_init(GPIO_TypeDef* port, uint8_t pin_num, drv_gpio_mode_t mode);
/**
 * @brief Sets a pin (drives it HIGH). (inline for max. speed)
 */
static inline void drv_gpio_set(GPIO_TypeDef* port, uint8_t pin_num){
	// BSRR register is atomic (interrupt-safe) see: "RM0008 Rev-21 9.2.5"
	port->BSRR = (1U << pin_num);
}
/**
 * @brief Clears a pin (drives it LOW.)
 */
static inline void drv_gpio_clear(GPIO_TypeDef* port, uint8_t pin_num){
	// BRR register is atomic (interrupt-safe) see: "RM0008 Rev-21 9.2.6"
	port->BRR = (1U << pin_num);
}
/**
 * @brief Reads the input state of a pin. (inline)
 * @return 1 if HIGH, 0 if LOW.
 */
static inline uint8_t drv_gpio_read(GPIO_TypeDef* port, uint8_t pin_num){
	// Read the Input Data Register (IDR) see: "RM0009 Rev-21 9.2.3"
	return (port->IDR & (1U << pin_num)) ? 1U:0U;
}
#endif // __DRV_GPIO_H






