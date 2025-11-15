#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "stm32f1xx.h"
#include <stdint.h>

/**
 * @brief Initialisiert einen UART-Port (inkl. Clock, GPIOs und Baudrate).
 *
 * @param instance Der UART-Port (z.B. USART1, USART2)
 * @param baud     Die gewünschte Baudrate (z.B. 115200)
 */
void drv_uart_init(USART_TypeDef* instance, uint32_t baud);

/**
 * @brief Sendet ein einzelnes Zeichen (blockierend).
 */
void drv_uart_send_char(USART_TypeDef* instance, char c);

/**
 * @brief Sendet einen String (blockierend).
 */
void drv_uart_send_string(USART_TypeDef* instance, const char* s);

#endif // __DRV_UART_H
