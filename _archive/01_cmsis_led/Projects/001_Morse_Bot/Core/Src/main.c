/*
 *
 * main.c
 *
 * Goal: Systick and LED for Morse Code
 * Clock: HSI 8MHz (default)
 *
 *
 */

// Include
# include "stm32f103x6.h"

/* --- Global Variables ---*/
// Declaration of 'g_msTicks' here as "extern"
// Original is in 'stm32f1xx_it.c'
// 'volatile' is important, as it is changed in an ISR!
extern volatile uint32_t g_msTicks;

/* --- Morse Timing Constant ---*/
#define DIT_MS 100 // Base time unit : 100ms


/*--- Function Prototypes --- */
void Delay_ms(uint32_t ms);
void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

void UART1_Init(void);
void UART1_SendChar(char c);
char UART1_GetChar(void);

void morse_dit(void);
void morse_dah(void);
void morse_char(char c);
/* --- Main --- */
int main(void){
	/*
	 * 1. System-Initialization
	 * SystemInit() is already called by the startup code.
	 * SystemCoreClock is 8,000,000 (8MHz HSI)
	 */

	/*
	 * 2. Configure SysTick
	 * Configures the SysTick-Timer to trigger an interrupt every 1ms.
	 * (8,000,000 Ticks / 1000) = 8000 Ticks per 1ms.
	 */
	SysTick_Config(SystemCoreClock / 1000);

	/* 3. Initialize Peripherals */
	LED_Init();
	UART1_Init();
	// Send a short start message
	UART1_SendChar('R');
	UART1_SendChar('D');
	UART1_SendChar('Y');
	UART1_SendChar('\n'); // '\n' is also a single char

	/* 4. Main loop */
	while (1){
		// wait for character
		char c = UART1_GetChar();
		// Send Echo (so the user sees what was typed)
		UART1_SendChar(c);
		// Convert character to Morse code
		// all in lowercase, to keep the switch case block small
		morse_char((char)tolower(c));
		// Pause between letters
		Delay_ms(DIT_MS*3);

	}
}
/*--- Implementations --- */

void Delay_ms(uint32_t ms){
	uint32_t start = g_msTicks;

	// Wait until target difference is reached.
	// Attention! Watch out for overflow!
	// For short delays, this is robust enough.
	while ((g_msTicks - start)<ms){
		// __WFI(); Wait for Interrupt (Power saving)!! Inconvenient for Debug!
	}
}

void LED_Init(void){
	// 1. Enable clock for GPIO Port C (sits on APB2)
	// RM008, Section 7.3.7
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

	// 2. Configure Pin PC13
	//	RM0008, Section 9.2.2 (CRH = Control Register High, because of Pin 13)
	// General purpose output push-pull, 2MHz
	// CNF (GP Out PP): 00
	// Register-Bits for Pin 13 (CNF[1] CNF[0] MODE[1] MODE[0]): 0010

	// 2.1 clear old bits for Pin 13 (Bits 20-23)
	GPIOC->CRH &= ~(0xF<<20); // 0xF at 5th position (5*4 = 20)
	// 2.2 set new bits
	GPIOC->CRH |= (0x2 << 20);
}

void LED_On(void){
	// BSRR (Bit Set/Reset Register)
	// BR13 (Bit Reset) sets the bit to 0 - LED ON
	GPIOC->BSRR = GPIO_BSRR_BR13;
}

void LED_Off(void){
	//BS13 (Bit Set) sets the bit to 1 -> LED OFF
	GPIOC->BSRR = GPIO_BSRR_BS13;
}

void LED_Toggle(void){
	// Read ODR Bit 13. If it is 0, then set to 1 and vice versa.
	if ((GPIOC->ODR & GPIO_ODR_ODR13)==0)
	{
		LED_Off();
	}
	else
	{
		LED_On();
	}
}

void UART1_Init(void){
	// 1. Enable Clock
	// RM0008, Section 7.3.7 (APB2ENR)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN | 	// UART1 Clock
					RCC_APB2ENR_IOPAEN | 	// GPIOA Clock (Because the pins for UART are on Port A!
					RCC_APB2ENR_AFIOEN;		// Alternate Function Clock

	// 2.GPIO (PA9 TX, PA10 RX)
	// RM0008, Section 9.2.2
	// PA9 (TX): Alternate Function Push-Pull, 50MHz (Mode:01, CNF:10 -> 1001)
	GPIOA->CRH &= ~(0xF << 4); // Clear bits for Pin 9
	GPIOA->CRH |= (0x9 << 4); // Set 1001 (0x9)

	// PA10 (RX): Input Floating (Mode: 00, CNF: 01 -> 0100)
	GPIOA->CRH &= ~(0xF << 8); // Clear bits for Pin 10 (Bits 8-11)
	GPIOA->CRH |= (0x4<< 8); // Set 0100 (0x4)

	// 3. Baudrate (BRR) - 8 MHz / (16*9600) = 52.083
	//	Mantissa = 52 (0x34), Fraction = 1 (0x1)
	USART1->BRR = (0x34 << 4) | 0x01;


	// 4. Enable UART (CR1)
	USART1->CR1 |= USART_CR1_UE |	// Enable UART
				USART_CR1_TE |		// Enable Transmitter
				USART_CR1_RE;		// Enable Receiver
}

void UART1_SendChar(char c){
	// Wait until the "Transmit Data Register" is empty. (TXE-Flag)
	while (!(USART1->SR & USART_SR_TXE))
	{
			// Wait
	}
	// Write the character to the "Data Register" (DR)
	USART1->DR = c;
}

char UART1_GetChar(void){
	// Wait until the "Read Data Register Not Empty" flag is set (RXNE-Flag)
	while (!(USART1->SR & USART_SR_RXNE))
	{
		// Waiting...
	}
	// Read the character from the "Data Register" (DR)
	return (char)(USART1->DR & 0xFF);
}
/**
 * 'Dit' = Dot
 * 1x Dit_MS ON, 1x DIT_MS OFF
 */
void morse_dit(void){
	LED_On();
	Delay_ms(DIT_MS);
	LED_Off();
	Delay_ms(DIT_MS);
}
/*
 * 'Dah' = Dash
 * 3x DIT_MS ON, 1x DIT_MS OFF
 */
void morse_dah(void){
	LED_On();
	Delay_ms(DIT_MS*3);
	LED_Off();
	Delay_ms(DIT_MS);
}

/*
 * Morse code for the letters
 */
void morse_char(char c)
{
    switch (c)
    {
        // Letters
        case 'a': morse_dit(); morse_dah(); break;
        case 'b': morse_dah(); morse_dit(); morse_dit(); morse_dit(); break;
        case 'c': morse_dah(); morse_dit(); morse_dah(); morse_dit(); break;
        case 'd': morse_dah(); morse_dit(); morse_dit(); break;
        case 'e': morse_dit(); break;
        case 'f': morse_dit(); morse_dit(); morse_dah(); morse_dit(); break;
        case 'g': morse_dah(); morse_dah(); morse_dit(); break;
        case 'h': morse_dit(); morse_dit(); morse_dit(); morse_dit(); break;
        case 'i': morse_dit(); morse_dit(); break;
        case 'j': morse_dit(); morse_dah(); morse_dah(); morse_dah(); break;
        case 'k': morse_dah(); morse_dit(); morse_dah(); break;
        case 'l': morse_dit(); morse_dah(); morse_dit(); morse_dit(); break;
        case 'm': morse_dah(); morse_dah(); break;
        case 'n': morse_dah(); morse_dit(); break;
        case 'o': morse_dah(); morse_dah(); morse_dah(); break;
        case 'p': morse_dit(); morse_dah(); morse_dah(); morse_dit(); break;
        case 'q': morse_dah(); morse_dah(); morse_dit(); morse_dah(); break;
        case 'r': morse_dit(); morse_dah(); morse_dit(); break;
        case 's': morse_dit(); morse_dit(); morse_dit(); break;
        case 't': morse_dah(); break;
        case 'u': morse_dit(); morse_dit(); morse_dah(); break;
        case 'v': morse_dit(); morse_dit(); morse_dit(); morse_dah(); break;
        case 'w': morse_dit(); morse_dah(); morse_dah(); break;
        case 'x': morse_dah(); morse_dit(); morse_dit(); morse_dah(); break;
        case 'y': morse_dah(); morse_dit(); morse_dah(); morse_dah(); break;
        case 'z': morse_dah(); morse_dah(); morse_dit(); morse_dit(); break;

        // Numbers
        case '0': morse_dah(); morse_dah(); morse_dah(); morse_dah(); morse_dah(); break;
        case '1': morse_dit(); morse_dah(); morse_dah(); morse_dah(); morse_dah(); break;
        case '2': morse_dit(); morse_dit(); morse_dah(); morse_dah(); morse_dah(); break;
        case '3': morse_dit(); morse_dit(); morse_dit(); morse_dah(); morse_dah(); break;
        case '4': morse_dit(); morse_dit(); morse_dit(); morse_dit(); morse_dah(); break;
        case '5': morse_dit(); morse_dit(); morse_dit(); morse_dit(); morse_dit(); break;
        case '6': morse_dah(); morse_dit(); morse_dit(); morse_dit(); morse_dit(); break;
        case '7': morse_dah(); morse_dah(); morse_dit(); morse_dit(); morse_dit(); break;
        case '8': morse_dah(); morse_dah(); morse_dah(); morse_dit(); morse_dit(); break;
        case '9': morse_dah(); morse_dah(); morse_dah(); morse_dah(); morse_dit(); break;

        // Spacebar -> Pause between words
        case ' ':
            // Pause between words is 7x DIT_MS.
            // We already have 3x (Pause between letters).
            // So we need 4x extra.
            Delay_ms(DIT_MS * 4);
            break;

        // Unknown characters -> Do nothing
        default:
            break;
    }
}
