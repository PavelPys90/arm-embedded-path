#include "drv_gpio.h" // Include header

/**
 * @brief Helper function to enable the clock for a port
 * Called by drv_gpio_init
 */
static void drv_gpio_enable_port_clock(GPIO_TypeDef* port){
	// RCC->APB2ENR register in RM0008 Rev-21 8.3.7
	if (port == GPIOA){
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	} else if (port == GPIOB){
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	} else if (port == GPIOC){
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	} else if (port == GPIOD){
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	}
}

/**
 * @brief Implementation of GPIO initialization.
 */
void drv_gpio_init(GPIO_TypeDef* port, uint8_t pin_num, drv_gpio_mode_t mode){
	// 1. Pin check: Ensures pin number is valid (0–15)
	if (pin_num > 15){
		return;
	}
	// 1.1 Masking for the CR (Configuration Register)
	// Strip custom "Marker Bit" (Bit 4) because the hardware register only expects 4 bits.
	// Example: 0b11000 (PullUp) & 0x0F becomes 0b1000 (Hardware Input Pull Code)
	//          0b01000 (PullDown) & 0x0F also becomes 0b1000 (Hardware Input Pull Code)
	uint32_t cnf_mode_bits = (uint32_t)mode & 0x0F;
	// 2. Enable clock !!! IMPORTANT !!!
	drv_gpio_enable_port_clock(port);

	// 3. CRL/CRH logic (F1 specific) Pins 0–7 use CRL and 8–15 use CRH
	volatile uint32_t* cr_reg; // Pointer to CRL or CRH
	uint8_t cr_pin_pos = pin_num;
	if (pin_num < 8){
		// Pins 0–7: CRL register RM0008 Rev-21 9.2.1
		cr_reg = &port->CRL;
	}else{
		// Pins 8–15: CRH register RM0008 Rev-21 9.2.2
		cr_reg = &port->CRH;
		cr_pin_pos -= 8;  // Pin 8 becomes position 0 in CRH
	}

	// 4. Register operation (read-modify-write)
	uint8_t shift = cr_pin_pos * 4;		// 4 bits per pin
	uint32_t mask = 0b1111U << shift;	// Mask to clear the old 4 bits (ensures correct state configuration)

	// Disables interrupts to prevent disturbance during read-modify-write operation
	__disable_irq();

	uint32_t reg_val = *cr_reg;
	reg_val &= ~mask;						// Clears the old 4 bits
	reg_val |= (cnf_mode_bits << shift);	// Sets the new 4 bits
	*cr_reg = reg_val;						// Writes back to the register

	__enable_irq();   // Re-enables interrupts

	// 5. Special case: Handles input with pull-up or pull-down (F1 specific)
	if (mode == GPIO_MODE_INPUT_PULL_UP){
		// Sets ODR bit to 1 to activate Pull-up
		port->BSRR = (1U << pin_num);
	}
	else if (mode == GPIO_MODE_INPUT_PULL_DOWN) {
		// Sets ODR bit to 0 to activate Pull-Down
		port->BRR = (1U << pin_num);
	}
}
