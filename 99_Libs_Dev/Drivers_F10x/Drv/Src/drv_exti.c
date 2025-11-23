#include "drv_exti.h"

// Storage for callback functions (for 16 pins)
static void (*g_exti_callbacks[16])(void) = {0};

void drv_exti_init(GPIO_TypeDef* port, uint8_t pin_num, drv_exti_trigger_t trigger, void (*callback_func)(void)){
	if (pin_num > 15) return;

	// 1. Save callback
	g_exti_callbacks[pin_num] = callback_func;

	// 2. Pin configuration - Input
	// Default Pull-Up (Button - GND)
	drv_gpio_init(port,pin_num, GPIO_MODE_INPUT_PULL_UP);

	// 3. Enables AFIO clock (Important for F1)
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;				// RM0008 Rev-21 7.3.7

	// 4. Mapping for AFIO configuration
	uint8_t port_code = 0;
	if (port == GPIOA) port_code = 0; 					// 0000
	else if (port == GPIOB) port_code = 1; 				// 0001
	else if (port == GPIOC) port_code = 2;				// 0010
	else if (port == GPIOD) port_code = 3;				// 0011

	// AFIO_EXTICR has 4 registers (0-3), each controls 4 pins.
	uint8_t reg_idx = pin_num / 4;
	uint8_t bit_pos = (pin_num % 4)*4;

	// Clears and sets the bits
	AFIO->EXTICR[reg_idx] &= ~(0xF << bit_pos);			// RM0008 Rev-21 9.4.3 - 9.4.6
	AFIO->EXTICR[reg_idx] |= (port_code << bit_pos);	// RM0008 Rev-21 9.4.3 - 9.4.6

	// 5. Set Trigger (Rising/Falling) in EXTI
	if (trigger == EXTI_TRIGGER_RISING || trigger == EXTI_TRIGGER_BOTH){
		EXTI->RTSR |= (1U << pin_num);					// RM0008 Rev-21 10.3.3
	} else {
		EXTI->RTSR &= ~(1U<<pin_num);					// RM0008 Rev-21 10.3.3
	}

	if (trigger == EXTI_TRIGGER_FALLING || trigger == EXTI_TRIGGER_BOTH){
		EXTI->FTSR |= (1U << pin_num);					// RM0008 Rev-21 10.3.4
	}else{
		EXTI->FTSR &= ~(1U << pin_num);					// RM0008 Rev-21 10.3.4
	}

	// 6. Enables Interrupt (Sets Mask)
	EXTI->IMR |= (1U << pin_num);						// RM0008 Rev-21 10.3.1

	// 7. NVIC (Configure CPU Core)
	IRQn_Type irq_n;
	if (pin_num == 0) irq_n = EXTI0_IRQn;
	else if (pin_num == 1) irq_n = EXTI1_IRQn;
	else if (pin_num == 2) irq_n = EXTI2_IRQn;
	else if (pin_num == 3) irq_n = EXTI3_IRQn;
	else if (pin_num == 4) irq_n = EXTI4_IRQn;
	else if (pin_num >= 5 && pin_num <= 9) irq_n = EXTI9_5_IRQn;
	else irq_n = EXTI15_10_IRQn; // Pins 10-15

	NVIC_SetPriority(irq_n, 2);	// Medium priority
	NVIC_EnableIRQ(irq_n);

	// 8. Clear pending flag
	drv_exti_clear_pending(pin_num);
}


// The handler that is called when the interrupt triggers.
// IMPORTANT: 'static' REMOVED so it can be called from stm32f1xx_it.c!
void drv_exti_irq_handler(uint8_t pin_num) {
    // 1. Checks if this pin triggered (Pending Register)
    if (EXTI->PR & (1U << pin_num)) {

        // 2. IMPORTANT: Clear flag (by writing a 1!)
        EXTI->PR = (1U << pin_num);

        // 3. Calls callback
        if (g_exti_callbacks[pin_num] != 0) {
            g_exti_callbacks[pin_num]();
        }
    }
}

void drv_exti_enable(uint8_t pin_num, uint8_t enable){
	if (pin_num > 15) return;
	if (enable){
		EXTI->IMR |= (1U << pin_num);		// RM0008 Rev-21 10.3.1
	}else{
		EXTI->IMR &= ~(1U << pin_num);		// RM0008 Rev-21 10.3.1
	}
}

void drv_exti_clear_pending(uint8_t pin_num){
	if (pin_num > 15) return;
	EXTI->PR = (1U << pin_num);				// RM0008 Rev-21 10.3.6
}

void drv_exti_deinit(uint8_t pin_num) {
    if (pin_num > 15) return;
    
    // 1. Clear callback
    g_exti_callbacks[pin_num] = NULL;
    
    // 4. Reset AFIO EXTICR mapping
    uint8_t reg_idx = pin_num / 4;
    uint8_t bit_pos = (pin_num % 4) * 4;
    AFIO->EXTICR[reg_idx] &= ~(0xF << bit_pos);  // RM0008 Rev-21 9.4.3-9.4.6
    
    // 5. Clear trigger configuration (RTSR/FTSR)
    EXTI->RTSR &= ~(1U << pin_num);  // RM0008 Rev-21 10.3.3
    EXTI->FTSR &= ~(1U << pin_num);  // RM0008 Rev-21 10.3.4
    
    // 6. Disable interrupt mask
    drv_exti_enable(pin_num, 0);
    
    // 8. Clear pending flag
    drv_exti_clear_pending(pin_num);
}
