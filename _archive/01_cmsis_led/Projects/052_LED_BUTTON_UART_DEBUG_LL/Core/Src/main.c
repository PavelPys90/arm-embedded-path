/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stm32f1xx_ll_utils.h"
#include "ll_tick.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_IDLE = 0,
    STATE_ACTIVE
} system_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PD */
#define LED_IDLE_PERIOD_MS  500U
#define LED_ACTIVE_PERIOD_MS 250U
#define BUTTON_DEBOUNCE_MS  30U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
// Variablen für unsere State-Machine
system_state_t g_state = STATE_IDLE;
uint32_t g_led_timer = 0;
uint32_t g_button_timer = 0;
uint8_t g_button_latched = 1U; // 1 = Losgelassen (wegen PULL_UP)
uint8_t g_button_consumed = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void uart_send_char(char c);
static void uart_send_string(const char *s);
static void log_state(system_state_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Systick_Reinit_1ms(void)
{
    // aus und sauber neu setzen
    SysTick->CTRL = 0;
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;  // 1 ms
    SysTick->VAL  = 0;

    // Clock = HCLK, Interrupt an, SysTick an
    SysTick->CTRL =  SysTick_CTRL_CLKSOURCE_Msk
                   | SysTick_CTRL_TICKINT_Msk
                   | SysTick_CTRL_ENABLE_Msk;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  Systick_Reinit_1ms();   // <<< dazu
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // LED initial ausschalten (PC13 ist active-low, HIGH = AUS)
    // (Hinweis: MX_GPIO_Init hat sie schon auf LOW gesetzt,
    //  also schalten wir sie hier korrekt aus)
    LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_13);

    // Start-Nachrichten senden
    uart_send_string("\r\n=== LL (CubeMX) HSI 16MHz Demo ===\r\n");
    uart_send_string("Clock: 16 MHz (HSI/PLL)\r\n");
    uart_send_string("Framework: LL\r\n");

    // Initial-Status loggen
    log_state(g_state);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = LL_GetTick();

	      // 1 Hz Heartbeat zum Prüfen, dass der Tick läuft
	      static uint32_t last_hb = 0;
	      if ((now - last_hb) >= 1000U) {
	          last_hb = now;
	          LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	          uart_send_string(".");
	      }

	      // --- Button Debounce ---
	      uint8_t level = (LL_GPIO_ReadInputPort(GPIOA) & LL_GPIO_PIN_0) ? 1U : 0U;
	      if (level != g_button_latched) {
	          g_button_latched = level;
	          g_button_timer = now;
	      }

	      // --- State Machine ---
	      if ((now - g_button_timer) >= BUTTON_DEBOUNCE_MS) {
	          if ((level == 0U) && (g_button_consumed == 0U)) {
	              g_state = (g_state == STATE_IDLE) ? STATE_ACTIVE : STATE_IDLE;
	              log_state(g_state);
	              g_button_consumed = 1U;
	          } else if (level != 0U) {
	              g_button_consumed = 0U;
	          }
	      }

	      // --- LED Blinking (state-basiert) ---
	      uint32_t period = (g_state == STATE_ACTIVE) ? LED_ACTIVE_PERIOD_MS : LED_IDLE_PERIOD_MS;
	      if ((now - g_led_timer) >= period) {
	          g_led_timer = now;
	          LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
	      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(16000000);
  LL_SetSystemCoreClock(16000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_Init1msTick(SystemCoreClock);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Sendet ein einzelnes Zeichen (blockierend)
 */
static void uart_send_char(char c) {
    // Warten bis "Transmit Data Register" leer ist
    while (LL_USART_IsActiveFlag_TXE(USART1) != 1U) {}

    // Daten senden
    LL_USART_TransmitData8(USART1, (uint8_t)c);
}

/**
 * @brief Sendet einen String (blockierend)
 */
static void uart_send_string(const char *s) {
    while (*s != '\0') {
        uart_send_char(*s++);
    }
}

/**
 * @brief Sendet den aktuellen Status über UART
 */
static void log_state(system_state_t state) {
    if (state == STATE_IDLE) {
        uart_send_string("STATE: IDLE - LED slow blink\r\n");
    } else {
        uart_send_string("STATE: ACTIVE - LED fast blink\r\n");
    }
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
