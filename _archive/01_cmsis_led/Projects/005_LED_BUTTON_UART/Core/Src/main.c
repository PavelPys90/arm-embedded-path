/*
 * Minimal CMSIS application for STM32F103C6T6
 * - Clock source: HSI boosted via PLL to 16 MHz
 * - LED on PC13 toggles in software states
 * - Button on PA0 toggles the active state (no blocking delays)
 * - UART1 (PA9/PA10) prints every state change
 */

#include "stm32f103x6.h"

#define LED_PIN              13U
#define BUTTON_PIN           0U

#define LED_PORT             GPIOC
#define BUTTON_PORT          GPIOA

#define LED_IDLE_PERIOD_MS   500U
#define LED_ACTIVE_PERIOD_MS 250U
#define BUTTON_DEBOUNCE_MS   30U

#define ENABLE_MCO_DIAG      1U

typedef enum {
    STATE_IDLE = 0,
    STATE_ACTIVE
} system_state_t;

static volatile uint32_t g_ms_ticks = 0;

static void SysTick_Init(void);
static void GPIO_Init(void);
static void UART1_Init(uint32_t baud);
static void uart_send_string(const char *s);
static void uart_send_char(char c);
static void uart_send_hex32(uint32_t value);
static void uart_send_dec(uint32_t value);
static void clock_diag_dump(void);
static void log_state(system_state_t state);
static uint32_t millis(void);
static uint8_t button_level(void);
static void led_toggle(void);
static void led_off(void);
#if ENABLE_MCO_DIAG
static void MCO_OutputSysclk(void);
#endif

int main(void) {
    SystemCoreClockUpdate();
    SysTick_Init();
    GPIO_Init();
#if ENABLE_MCO_DIAG
    MCO_OutputSysclk();
#endif
    UART1_Init(115200U);

    uart_send_string("\r\n=== CMSIS HSI 16MHz Demo ===\r\n");
    uart_send_string("Clock: HSI @ 16 MHz via PLL x4\r\n");
    uart_send_string("UART: 115200-8-N-1\r\n");
    clock_diag_dump();

    system_state_t state = STATE_IDLE;
    uint32_t led_timer = 0;
    uint32_t button_timer = 0;
    uint8_t button_latched = 1U;
    uint8_t button_consumed = 0U;

    log_state(state);
    led_off();

    while (1) {
        uint32_t now = millis();
        uint8_t level = button_level();

        if (level != button_latched) {
            button_latched = level;
            button_timer = now;
        }

        if ((now - button_timer) >= BUTTON_DEBOUNCE_MS) {
            if ((level == 0U) && (button_consumed == 0U)) {
                state = (state == STATE_IDLE) ? STATE_ACTIVE : STATE_IDLE;
                log_state(state);
                button_consumed = 1U;
            } else if (level != 0U) {
                button_consumed = 0U;
            }
        }

        uint32_t period = (state == STATE_ACTIVE) ? LED_ACTIVE_PERIOD_MS : LED_IDLE_PERIOD_MS;
        if ((now - led_timer) >= period) {
            led_timer = now;
            led_toggle();
        }

        __WFI(); // Sleep until next interrupt tick
    }
}

static void SysTick_Init(void) {
    if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
        while (1) {
        }
    }
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

static void GPIO_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1;
    LED_PORT->BSRR = (1U << LED_PIN);

    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->ODR |= (1U << BUTTON_PIN);
}

static void UART1_Init(uint32_t baud) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    /* Ensure default mapping PA9/PA10 */
    AFIO->MAPR &= ~AFIO_MAPR_USART1_REMAP;

    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9 | GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= (GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1);  /* PA9: AF push-pull */
    GPIOA->CRH |= GPIO_CRH_CNF10_0;                    /* PA10: floating input */

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    uint32_t pclk = SystemCoreClock;
    uint32_t usartdiv = (pclk + (baud / 2U)) / baud;
    uint32_t mantissa = usartdiv / 16U;
    uint32_t fraction = usartdiv - (mantissa * 16U);
    USART1->BRR = (mantissa << 4) | (fraction & 0xFU);

    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

static void uart_send_char(char c) {
    while ((USART1->SR & USART_SR_TXE) == 0U) {
    }
    USART1->DR = (uint32_t)c;
}

static void uart_send_string(const char *s) {
    while (*s != '\0') {
        uart_send_char(*s++);
    }
}

static void uart_send_hex32(uint32_t value) {
    for (int shift = 28; shift >= 0; shift -= 4) {
        uint8_t nibble = (value >> shift) & 0xFU;
        char c = (nibble < 10U) ? ('0' + nibble) : ('A' + (nibble - 10U));
        uart_send_char(c);
    }
}

static void uart_send_dec(uint32_t value) {
    char buffer[11];
    int idx = 10;
    buffer[idx] = '\0';
    if (value == 0U) {
        uart_send_char('0');
        return;
    }
    while (value > 0U && idx > 0) {
        idx--;
        buffer[idx] = '0' + (value % 10U);
        value /= 10U;
    }
    uart_send_string(&buffer[idx]);
}

static void clock_diag_dump(void) {
    uart_send_string("RCC_CFGR=0x");
    uart_send_hex32(RCC->CFGR);
    uart_send_string(" RCC_CR=0x");
    uart_send_hex32(RCC->CR);
    uart_send_string("\r\nSystemCoreClock=");
    uart_send_dec(SystemCoreClock);
    uart_send_string(" Hz\r\n");
}

static void log_state(system_state_t state) {
    if (state == STATE_IDLE) {
        uart_send_string("STATE: IDLE - LED slow blink\r\n");
    } else {
        uart_send_string("STATE: ACTIVE - LED fast blink\r\n");
    }
}

static uint32_t millis(void) {
    __disable_irq();
    uint32_t ticks = g_ms_ticks;
    __enable_irq();
    return ticks;
}

static uint8_t button_level(void) {
    return (BUTTON_PORT->IDR & (1U << BUTTON_PIN)) ? 1U : 0U;
}

static void led_toggle(void) {
    LED_PORT->ODR ^= (1U << LED_PIN);
}

static void led_off(void) {
    LED_PORT->BSRR = (1U << LED_PIN);
}

void SysTick_Handler(void) {
    g_ms_ticks++;
}

#if ENABLE_MCO_DIAG
static void MCO_OutputSysclk(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRH &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    GPIOA->CRH |= (GPIO_CRH_MODE8 | GPIO_CRH_CNF8_1);
    RCC->CFGR &= ~RCC_CFGR_MCO;
    RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
}
#endif
