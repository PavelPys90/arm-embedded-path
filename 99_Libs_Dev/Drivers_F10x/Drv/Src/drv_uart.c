#include "drv_uart.h"
#include "drv_gpio.h"	// NEED GPIO DRIVER!

// Callback storage
static void (*g_uart1_rx_callback)(uint8_t) = 0;
static void (*g_uart2_rx_callback)(uint8_t) = 0;

/**
 * @brief Private function: Activates clocks and configures pins
 * based on the selected UART port.
 */
static void drv_uart_gpio_init(USART_TypeDef* instance){
	if (instance == USART1){
		// 1. Enable clocks
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;		// (RM0008 Rev-21 8.3.7)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;							// (RM0008 Rev-21 8.3.7)

		// 2. Configure pins (with drv_gpio!)
		// PA9 (TX) = Alternate Function Push-Pull, 50MHz
		drv_gpio_init(GPIOA, 9, GPIO_MODE_AF_PP_50MHZ);
		// PA10 (RX) = Input Floating
		drv_gpio_init(GPIOA, 10, GPIO_MODE_INPUT_FLOATING);
	}else if (instance == USART2){
		// 1. Enable clocks
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;		// (RM0008 Rev-21 8.3.7)
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;							// (RM0008 Rev-21 8.3.8)

		// 2. Configure pins
		// PA2 (TX) = Alternate Function Push-Pull, 50MHz
		drv_gpio_init(GPIOA, 2, GPIO_MODE_AF_PP_50MHZ);
		// PA3 (RX) = Input Floating
		drv_gpio_init(GPIOA, 3, GPIO_MODE_INPUT_FLOATING);
	}
	// (USART3 etc. can be added here)
}

/**
 * @brief Initializes a UART port.
 */
void drv_uart_init(USART_TypeDef* instance, uint32_t baud){
	// 1. Initialize GPIOs and clocks for this port
	drv_uart_gpio_init(instance);

	// 2. Determines clock frequency for baud rate calculation
	//    *** CLOCK SELECTION LOGIC ***
	uint32_t pclk = 0;
	if (instance == USART1){
		// USART1 connects to PCLK2 (faster clock)
		//  PCLK2 = SystemCoreClock, as configured in
		//  bsp_clock driver (e.g., at 72MHz)
		pclk = SystemCoreClock;
	}
	else{
		// USART2, 3, 4, 5 connect to PCLK1 (slower clock)
		// (In the 72MHz profile, PCLK1 = HCLK/2 = 36MHz)
		// The CMSIS variable SystemCoreClock represents HCLK.
		// Prescaler bits must be read (or known):

		// Simple (but correct) assumption for our 72MHz profile:
		if (SystemCoreClock == 72000000U){
			pclk = SystemCoreClock / 2; // PCLK1 is 36MHz
		}
		else{
			// For 16MHz or 8MHz profiles, PCLK1 = HCLK
			pclk = SystemCoreClock;
		}
	}
	// 3. Calculates baud rate (formula from RM0008)
	//    BRR = PCLK / (16 * Baud rate)
	//    Rounds to minimize error:
	uint32_t uartdiv = (pclk + (baud / 2U)) / baud;

	// 4. Sets BRR (Baud Rate Register)
	//    Lower 4 bits are the fraction, remaining bits are the mantissa
	instance->BRR = (uartdiv & 0xFFF0) | ((uartdiv & 0xFU) >> 0); // Compact version

	// 5. Enables UART (UE=UART Enable, TE=Tx Enable, RE=Rx Enable)
	instance->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/**
 * @brief Sends a single character (blocking).
 */
void drv_uart_send_char(USART_TypeDef* instance, char c){
	// Waits until the "Transmit Data Register" (DR) is empty (TXE flag)
	while ((instance->SR & USART_SR_TXE) == 0U) {
	        // Waiting loop (blocking)
	}
	// Writes the data byte to the register
	instance->DR = (uint32_t)c;
}

/**
 * @brief Sends a string (blocking).
 */
void drv_uart_send_string(USART_TypeDef* instance, const char* s){
	while (*s != '\0') {
	        drv_uart_send_char(instance, *s++);
	}
}

/**
 * @brief Receives a single character (blocking).
 */
char drv_uart_receive_char(USART_TypeDef* instance){
	// Waits until the "Receive Data Register" is not empty (RXNE flag)
	while ((instance->SR & USART_SR_RXNE) == 0U) {		// (RM0008 Rev-21 27.6.1)
		// Waiting loop
	}
	// Reads the data
	return (char)(instance->DR & 0xFF);					// (RM0008 Rev-21 27.6.2)
}

/**
 * @brief Enables RX interrupt and registers a callback.
 */
void drv_uart_enable_rx_interrupt(USART_TypeDef* instance, void (*callback)(uint8_t data)){
	// 1. Save callback
	if (instance == USART1) g_uart1_rx_callback = callback;
	else if (instance == USART2) g_uart2_rx_callback = callback;

	// 2. Enable RXNE Interrupt in UART
	instance->CR1 |= USART_CR1_RXNEIE;					// (RM0008 Rev-21 27.6.4)

	// 3. Enable NVIC
	if (instance == USART1) {
		NVIC_SetPriority(USART1_IRQn, 2);
		NVIC_EnableIRQ(USART1_IRQn);
	} else if (instance == USART2) {
		NVIC_SetPriority(USART2_IRQn, 2);
		NVIC_EnableIRQ(USART2_IRQn);
	}
}

/**
 * @brief The manager called by stm32f1xx_it.c
 */
void drv_uart_irq_handler(USART_TypeDef* instance){
	// Check if RXNE is set
	if (instance->SR & USART_SR_RXNE){					// (RM0008 Rev-21 27.6.1)
		// Read data (clears RXNE)
		uint8_t data = (uint8_t)(instance->DR & 0xFF);	// (RM0008 Rev-21 27.6.2)

		// Call callback
		if (instance == USART1 && g_uart1_rx_callback){
			g_uart1_rx_callback(data);
		} else if (instance == USART2 && g_uart2_rx_callback){
			g_uart2_rx_callback(data);
		}
	}
}
