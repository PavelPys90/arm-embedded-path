#ifndef __DRV_EXTI_H
#define __DRV_EXTI_H

#include "stm32f1xx.h"
#include "drv_gpio.h" // GPIO Enums
#include <stdint.h>

// Trigger condition
typedef enum {
    EXTI_TRIGGER_RISING,  // Rising edge (0 -> 1)
    EXTI_TRIGGER_FALLING, // Falling edge (1 -> 0)
    EXTI_TRIGGER_BOTH     // Both
} drv_exti_trigger_t;

/**
 * @brief Initializes a pin as an external interrupt.
 * * @param port      The GPIO Port (e.g. GPIOA)
 * @param pin_num   The pin number (0-15)
 * @param trigger   When should it trigger?
 * @param callback  Function (void name(void)) to be called.
 */
void drv_exti_init(GPIO_TypeDef* port, uint8_t pin_num, drv_exti_trigger_t trigger, void (*callback_func)(void));

/**
 * @brief The manager called by stm32f1xx_it.c.
 */
void drv_exti_irq_handler(uint8_t pin_num);

#endif // __DRV_EXTI_H
