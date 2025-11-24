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
//#include "drv_uart.h"
//#include "drv_adc.h"
//#include "drv_pwm.h"
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
	// Status LED Init (PC13)
	drv_gpio_init(GPIOC, 13, GPIO_MODE_OUTPUT_PP_2MHZ);
	drv_gpio_write(GPIOC, 13, 1);  // LED off (active low)

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*
	// Read ADC value from potentiometer
	uint16_t adc_value = drv_adc_read(ADC1, ADC_CHANNEL_0);

	// Convert ADC value (0-4095) to PWM duty cycle (0-100%)
	uint8_t duty_cycle = (adc_value * 100) / 4095;

	// Set PWM duty cycle (controls LED brightness)
	drv_pwm_set_duty(TIM2, PWM_CHANNEL_2, duty_cycle);

	// Convert values to strings for UART output
	char adc_str[6];
	char duty_str[4];
	uint_to_string(adc_value, adc_str);
	uint_to_string(duty_cycle, duty_str);

	// Send values over UART
	drv_uart_send_string(USART1, "ADC: ");
	drv_uart_send_string(USART1, adc_str);
	drv_uart_send_string(USART1, " | PWM: ");
	drv_uart_send_string(USART1, duty_str);
	drv_uart_send_string(USART1, "%\r\n");
	*/

	// Toggle Status LED (PC13)
	drv_gpio_toggle(GPIOC, 13);

	// Small delay for readability
	drv_systick_delay(1000);
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


