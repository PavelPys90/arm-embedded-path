/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Bare-Metal temperature measurement using internal sensor
  ******************************************************************************
  * @attention
  *
  * This program implements temperature measurement of the internal sensor
  * of STM32F103C6T6A using exclusively direct register access (Bare-Metal).
  * No ST HAL or LL libraries are used to maximize understanding of hardware registers.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f103x6.h"
#include <stdint.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Temperature sensor parameters from datasheet
#define V25         1.43f      // Voltage at 25°C (typical)
#define AVG_SLOPE   0.0043f    // 4.3mV/°C in volts
#define VDDA        3.3f       // Supply voltage assumption
#define ADC_MAX     4095.0f    // 12-bit ADC maximum value
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t adc_value = 0;
volatile float temperature_celsius = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void RCC_Init(void);
void ADC_Temp_Sensor_Init(void);
void Delay_ms(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Simple software delay in milliseconds
  * @param  ms: Number of milliseconds to wait
  * @retval None
  * @note   This function is inaccurate and only for demonstration purposes.
  *         For precise timing, use timers or SysTick.
  */
void Delay_ms(uint32_t ms)
{
  for (uint32_t i = 0; i < ms; i++)
  {
    for (volatile uint32_t j = 0; j < 8000; j++); // Tuned for 8MHz clock
  }
}

/**
  * @brief  System Clock Configuration
  * @retval None
  * @note   Configures RCC using HSI 8MHz as system clock
  *         Replaces auto-generated SystemClock_Config()
  */
void RCC_Init(void)
{
  /* RCC clock configuration using HSI (8MHz) */

  /* Reset RCC configuration (optional) */
  // RCC->CR |= RCC_CR_HSION;  // HSI is enabled by default after reset

  /* Configure Flash prefetch and wait states */
  // FLASH->ACR &= ~FLASH_ACR_LATENCY;  // 0 wait state sufficient for 8MHz

  /* HCLK (AHB clock) = SYSCLK */
  RCC->CFGR &= ~RCC_CFGR_HPRE;    // AHB prescaler = 1

  /* PCLK1 (APB1 clock) = HCLK */
  RCC->CFGR &= ~RCC_CFGR_PPRE1;   // APB1 prescaler = 1

  /* PCLK2 (APB2 clock) = HCLK */
  RCC->CFGR &= ~RCC_CFGR_PPRE2;   // APB2 prescaler = 1

  /* No PLL configuration - using HSI directly */
}

/**
  * @brief  ADC1 and temperature sensor initialization
  * @retval None
  * @note   Complete bare-metal ADC configuration for internal temperature sensor
  */
void ADC_Temp_Sensor_Init(void)
{
  /* 1. Enable ADC1 clock */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  /* 2. Configure ADC clock prescaler */
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;  // ADC clock = 8MHz / 6 = 1.33MHz

  /* 3. ADC configuration sequence */

  /* Enable temperature sensor and VREFINT */
  ADC1->CR2 |= ADC_CR2_TSVREFE;

  /* Power on ADC (two-step wakeup) */
  ADC1->CR2 |= ADC_CR2_ADON;      // First ADON set - ADC in power-up state
  Delay_ms(1);                    // Short stabilization delay
  ADC1->CR2 |= ADC_CR2_ADON;      // Second ADON set - ADC ready

  /* Perform ADC calibration */
  ADC1->CR2 |= ADC_CR2_CAL;       // Start calibration
  while (ADC1->CR2 & ADC_CR2_CAL) // Wait for calibration complete
  {
    /* Calibration in progress */
  }

  /* 4. Channel configuration */

  /* Set extended sampling time for temperature sensor (channel 16) */
  ADC1->SMPR1 |= (0b111 << 18);   // 239.5 cycles sampling time

  /* Configure conversion sequence - single conversion of channel 16 */
  ADC1->SQR1 = 0;                 // 1 conversion in sequence
  ADC1->SQR3 = 16;                // First conversion: channel 16
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // Note: HAL_Init() is not called in bare-metal approach

  /* USER CODE BEGIN Init */
  RCC_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  // Note: SystemClock_Config() is replaced by RCC_Init()

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  ADC_Temp_Sensor_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 IWDG->KR = 0xAAAA;
    /* Start ADC conversion */
    ADC1->CR2 |= ADC_CR2_ADON;        // Start single conversion

    /* Wait for conversion complete */
    while (!(ADC1->SR & ADC_SR_EOC))
    {
      /* Conversion in progress */
    }

    /* Read converted value */
    adc_value = ADC1->DR;

    /* Calculate temperature */
    float vsense = (VDDA * (float)adc_value) / ADC_MAX;
    temperature_celsius = ((V25 - vsense) / AVG_SLOPE) + 25.0f;

    /* Measurement interval */
    Delay_ms(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
