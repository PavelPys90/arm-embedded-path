#include "stm32f103x6.h" // Correct header for STM32F103C6T6A

volatile uint32_t msTicks = 0;

void delay_ms(uint32_t ms);
void SystemClock_Config(void);
void GPIO_LED_Init(void);

int main(void)
{
    SystemClock_Config();         // Configure system clock using HSI
    SysTick_Config(8000);         // 8 MHz / 8000 = 1 ms tick
    GPIO_LED_Init();              // Initialize PC13 as output

    while (1)
    {
        GPIOC->ODR ^= GPIO_ODR_ODR13; // Toggle LED
        delay_ms(500);               // Wait 500 ms
    }
}

void delay_ms(uint32_t ms)
{
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}

void SysTick_Handler(void)
{
    msTicks++;
}

void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSION; // Enable HSI
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI is ready

    RCC->CFGR &= ~RCC_CFGR_SW;         // Clear system clock switch bits
    RCC->CFGR |= RCC_CFGR_SW_HSI;      // Select HSI as system clock

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI); // Wait until HSI is used

    // Set AHB, APB1, APB2 prescalers to /1
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
}

void GPIO_LED_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Enable GPIOC clock

    // Configure PC13 as push-pull output, 2 MHz
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13_1;

    // Set PC13 high (LED off for active-low configuration)
    GPIOC->BSRR = GPIO_BSRR_BS13;
}
