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
#include "drv_gpio.h"
#include "drv_systick.h"
#include "bsp_clock.h"
#include "drv_uart.h"
#include "drv_exti.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void on_button_press(void) {
    // 1. LED umschalten
    drv_gpio_toggle(GPIOC, 13);

    // 2. Nachricht senden (Bestätigung)
    //    (Wir nutzen hier NUR die Sende-Funktion, die wir schon haben!)
    drv_uart_send_string(USART1, "Button gedrueckt! (EXTI)\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	bsp_clock_init(CLOCK_PROFILE_HSE_72MHZ_PLL);
	drv_systick_init();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	drv_gpio_init(GPIOC, 13, GPIO_MODE_OUTPUT_PP_2MHZ);
	drv_gpio_write(GPIOC, 13, 1);

	// UART Init (startet RX Interrupt)
	drv_uart_init(USART1, 115200);

	// Button Init (startet EXTI Interrupt)
	// PA0, Fallende Flanke (Drücken), rufe 'on_button_press'
	drv_exti_init(GPIOA, 0, EXTI_TRIGGER_FALLING, on_button_press);

	drv_uart_send_string(USART1, "System Ready. Druecke Button oder tippe...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


