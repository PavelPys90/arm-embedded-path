#include "drv_uart.h"
#include "drv_gpio.h"	// BRAUCHEN GPIO-TREIBER!

/**
 * @brief Private Funktion: Aktiviert Clocks und konfiguriert Pins
 * basierend auf dem gewählten UART-Port.
 */
static void drv_uart_gpio_init(USART_TypeDef* instance){
	if (instance == USART1){
		// 1. Clocks aktivieren
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;		// (RM0008 Rev-21 8.3.7)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;							// (RM0008 Rev-21 8.3.7)

		// 2. Pins konfigurieren (mit drv_gpio!)
		// PA9 (TX) = Alternate Function Push-Pull, 50MHz
		drv_gpio_init(GPIOA, 9, GPIO_MODE_AF_PP_50MHZ);
		// PA10 (RX) = Input Floating
		drv_gpio_init(GPIOA, 10, GPIO_MODE_INPUT_FLOATING);
	}else if (instance == USART2){
		// 1. Clocks aktivieren
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;		// (RM0008 Rev-21 8.3.7)
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;							// (RM0008 Rev-21 8.3.8)

		// 2. Pins konfigurieren
		// PA2 (TX) = Alternate Function Push-Pull, 50MHz
		drv_gpio_init(GPIOA, 2, GPIO_MODE_AF_PP_50MHZ);
		// PA3 (RX) = Input Floating
		drv_gpio_init(GPIOA, 3, GPIO_MODE_INPUT_FLOATING);
	}
	// (hier könnte man USART3 etc. hinzufügen)
}

/**
 * @brief Initialisiert einen UART-Port.
 */
void drv_uart_init(USART_TypeDef* instance, uint32_t baud){
	// 1. GPIOs und Clocks für diesen Port initialisieren
	drv_uart_gpio_init(instance);

	// 2. Taktfrequenz für die Baudraten-Berechnung ermitteln
	//    *** HIER IST DIE LOGIK ***
	uint32_t pclk = 0;
	if (instance == USART1){
		// USART1 hängt an PCLK2 (der schneller ist)
		//  PCLK2 = SystemCoreClock, was in
		//  bsp_clock-Treiber der Fall ist, z.B. bei 72MHz)
		pclk = SystemCoreClock;
	}
	else{
		// USART2, 3, 4, 5 hängen an PCLK1 (der langsamer ist)
		// (Im 72MHz-Profil war PCLK1 = HCLK/2 = 36MHz)
		// Die CMSIS-Variable SystemCoreClock ist HCLK.
		// Man muss die Prescaler-Bits lesen (oder einfach wissen):

		// Einfache (aber korrekte) Annahme für unser 72MHz-Profil:
		if (SystemCoreClock == 72000000U){
			pclk = SystemCoreClock / 2; // PCLK1 ist 36MHz
		}
		else{
			// Für 16MHz oder 8MHz Profile war PCLK1 = HCLK
			pclk = SystemCoreClock;
		}
	}
	// 3. Baudrate berechnen (Die Formel aus RM0008)
	//    BRR = PCLK / (16 * Baudrate)
	//    Runden kaufmännisch, um den Fehler zu minimieren:
	uint32_t uartdiv = (pclk + (baud / 2U)) / baud;

	// 4. BRR (Baud Rate Register) setzen
	//    Die unteren 4 Bits sind die Fraktion, der Rest die Mantisse
	instance->BRR = (uartdiv & 0xFFF0) | ((uartdiv & 0xFU) >> 0); // Kompakte Version

	// 5. UART aktivieren (UE=UART En, TE=Tx En, RE=Rx En)
	instance->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

/**
 * @brief Sendet ein einzelnes Zeichen (blockierend).
 */
void drv_uart_send_char(USART_TypeDef* instance, char c){
	// Warten, bis das "Transmit Data Register" (DR) leer ist (TXE-Flag)
	while ((instance->SR & USART_SR_TXE) == 0U) {
	        // Warte-Schleife (blocking)
	}
	// Das Daten-Byte in das Register schreiben
	instance->DR = (uint32_t)c;
}

/**
 * @brief Sendet einen String (blockierend).
 */
void drv_uart_send_string(USART_TypeDef* instance, const char* s){
	while (*s != '\0') {
	        drv_uart_send_char(instance, *s++);
	}
}
