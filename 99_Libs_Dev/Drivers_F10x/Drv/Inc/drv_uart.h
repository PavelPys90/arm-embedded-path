#ifndef __DRV_UART_H
#define __DRV_UART_H

#include "stm32f1xx.h"
#include <stdint.h>

/**
 * @brief Initializes a UART port (incl. Clock, GPIOs, and baud rate).
 *
 * @param instance The UART port (e.g. USART1, USART2)
 * @param baud     The desired baud rate (e.g. 115200)
 */
void drv_uart_init(USART_TypeDef* instance, uint32_t baud);

/**
 * @brief Sends a single character (blocking).
 */
void drv_uart_send_char(USART_TypeDef* instance, char c);

/**
 * @brief Sends a string (blocking).
 */
void drv_uart_send_string(USART_TypeDef* instance, const char* s);

/**
 * @brief Receives a single character (blocking).
 */
char drv_uart_receive_char(USART_TypeDef* instance);

/**
 * @brief Enables RX interrupt and registers a callback.
 * @param instance  The UART port
 * @param callback  Function to call when a byte is received
 */
void drv_uart_enable_rx_interrupt(USART_TypeDef* instance, void (*callback)(uint8_t data));

/**
 * @brief The manager called by stm32f1xx_it.c
 */
void drv_uart_irq_handler(USART_TypeDef* instance);

#endif // __DRV_UART_H
