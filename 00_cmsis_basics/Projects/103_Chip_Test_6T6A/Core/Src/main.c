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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DBGMCU_IDCODE_ADDR   ((uint32_t)0xE0042000UL)  // DEV_ID[11:0], REV_ID[31:16]
#define FLASH_SIZE_KB_ADDR   ((uint32_t)0x1FFFF7E0UL)  // uint16_t: size in kB
#define UID_BASE_ADDR        ((uint32_t)0x1FFFF7E8UL)  // 96-bit Unique ID (3x32)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern UART_HandleTypeDef huart1; // CubeMX in usart.c
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static const char dma_msg[] = "Hello via USART1 DMA TX\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline uint32_t READ_IDCODE(void);
static inline uint16_t READ_FLASH_KB(void);
static inline void     READ_UID(uint32_t uid[3]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
  return ch;
}

/* Variante B: _write() für newlib-nano (auskommentieren, wenn syscalls.c existiert) */
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart1, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
  return len;
}

/* Helfer: Register-Leser */
static inline uint32_t READ_IDCODE(void) {
  volatile uint32_t *ID = (uint32_t*)DBGMCU_IDCODE_ADDR;
  return *ID;
}
static inline uint16_t READ_FLASH_KB(void) {
  volatile uint16_t *FS = (uint16_t*)FLASH_SIZE_KB_ADDR;
  return *FS; // kB
}
static inline void READ_UID(uint32_t uid[3]) {
  volatile uint32_t *U = (uint32_t*)UID_BASE_ADDR;
  uid[0] = U[0]; uid[1] = U[1]; uid[2] = U[2];
}

/* Optional: DMA-TX-Callback (Bestätigung) */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    const char done[] = "DMA TX complete\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)done, (uint16_t)(sizeof(done)-1), 100);
  }
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
  uint32_t brr   = USART1->BRR;
  uint32_t over  = (huart1.Init.OverSampling == UART_OVERSAMPLING_16) ? 16u : 8u;
  printf("Diag: PCLK2=%lu Hz, USART1->BRR=0x%04lX, Over=%lu\r\n",
         (unsigned long)pclk2, (unsigned long)brr, (unsigned long)over);

  // Remap sicher ausschalten (USART1 soll PA9/PA10 nutzen):
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_USART1_DISABLE();
  	uint32_t idcode = READ_IDCODE();
    uint16_t dev_id = (uint16_t)(idcode & 0x0FFFu);
    uint16_t rev_id = (uint16_t)((idcode >> 16) & 0xFFFFu);
    uint16_t flash_kb = READ_FLASH_KB();
    uint32_t uid[3]; READ_UID(uid);

    printf("\r\n=== Blue Pill ID Check ===\r\n");
    printf("DBGMCU_IDCODE : 0x%08lX  (DEV_ID=0x%03lX, REV_ID=0x%04lX)\r\n",
           (unsigned long)idcode, (unsigned long)dev_id, (unsigned long)rev_id);
    printf("Flash size     : %lu KB\r\n", (unsigned long)flash_kb);
    printf("UID            : %08lX%08lX%08lX\r\n",
           (unsigned long)uid[2], (unsigned long)uid[1], (unsigned long)uid[0]);

    // DMA-TX-Kurztest (USART1 -> DMA1 Channel 4)
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)dma_msg, (uint16_t)(sizeof(dma_msg)-1));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
