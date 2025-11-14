#include "drv_gpio.h" // Einbinden der Header

/**
 * @brief Hilfsfunktion, um den Takt für einen Port zu aktivieren
 * Wird von drv_gpio_init aufgerufen
 */
static void drv_gpio_enable_port_clock(GPIO_TypeDef* port){
	// RCC->APB2ENR Register im RM0008 Rev-21 8.3.7
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
 * @brief Implementierung der GPIO-Initialisierung.
 */
void drv_gpio_init(GPIO_TypeDef* port, uint8_t pin_num, drv_gpio_mode_t mode){
	// 1. Pincheck: Pinnummer muss immer valide sein. (0-15)
	if (pin_num > 15){
		return;
	}

	// 2. Takt aktivieren !!! WICHTIG !!!
	drv_gpio_enable_port_clock(port);

	// 3. Die CRL/CRH-Logik (F1 spezifisch) Pin von 0-7 sind CRL und von 8-15 CRH
	volatile uint32_t* cr_reg; // Zeiger auf CRL oder CRH
	uint8_t cr_pin_pos = pin_num;
	if (pin_num < 8){
		// Pin 0-7 CRL Register RM0008 Rev-21 9.2.1
		cr_reg = &port->CRL;
	}else{
		// Pin 8-15 CRH Register RM0008 Rev-21 9.2.2
		cr_reg = &port->CRH;
		cr_pin_pos -= 8; // Pin 8 wird zu Position 0 im CRH
	}

	// 4 Register Operation (Read-Modify-Write)
	uint32_t cnf_mode_bits = (uint32_t)mode; // Die Enum-Werte sind schon als 4-Bit Maske definiert
	uint8_t shift = cr_pin_pos * 4;		// 4 Bits pro Pin
	uint32_t mask = 0b1111U << shift;	// Maske, um die 4 alten Bits zu löschen. (Wichtig um den gewünschten Zustand korrekt zu setzen.)

	// Um den Read-Modify-Write nicht zu stören werden in diesem Schritt die Interrupts verboten
	__disable_irq();

	uint32_t reg_val = *cr_reg;
	reg_val &= ~mask;						// Löschen der 4 alten Bits
	reg_val |= (cnf_mode_bits << shift);	// Setze die 4 neuen Bits
	*cr_reg = reg_val;						// Zurück ins Register schreiben

	__enable_irq();  // Interrupts wieder erlauben

	// 5. Sonderfall Input mit Pull-Up oder Pull-Down
	// F1 Spezifisch
	if (mode == GPIO_MODE_INPUT_PULL){
		port->BSRR = (1U << pin_num);
	}
}
