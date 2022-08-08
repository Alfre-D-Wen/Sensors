/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "pm_sensor.h"
#include "bsp_ili9341_lcd.h"
#include "bsp_spi_flash.h"
#include "core_delay.h"
#include "bsp_SysTick.h"
#include "bsp_dht11.h"
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
extern uint16_t PM_2_5_DATA;
extern uint8_t PM_RX_RESPOND_BUFFER[2];
extern uint8_t PM_RX_DATA_BUFFER[8];
char pm_display_buff[100];
char hum_display_buff[100];
char temp_display_buff[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  DHT11_Data_TypeDef DHT11_Data;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* PM2.5传感器停止自动传送 */
  PM_Stop_Transmit_Auto();

  /* LCD初始化 */
  ILI9341_Init();

  /* 温湿度传感器初始化 */
  DHT11_Init();

  /* 设置LCD字体，前景颜色，背景颜色 */
  LCD_SetFont(&Font8x16);
  LCD_SetColors(BLUE, BLACK);

  /* 清屏，显示全黑 */
  ILI9341_Clear(0, 0, LCD_X_LENGTH, LCD_Y_LENGTH);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 读取PM2.5数据 */
    PM_Read_Data();
    /* 读取温湿度数据 */
    DHT11_Read_TempAndHumidity(&DHT11_Data);
    /* 将写好的字符串发送到buffer */

    if ((PM_RX_RESPOND_BUFFER[0] | PM_RX_DATA_BUFFER[0]) == PM_FAIL)
    {
      sprintf(pm_display_buff, "PM2.5 Value Read Error");
    }
    else
    {
      sprintf(pm_display_buff, "PM2.5 Value: %d ug/m3", PM_2_5_DATA);
    }
    if (DHT11_Read_TempAndHumidity(&DHT11_Data) == SUCCESS)
    {
      sprintf(hum_display_buff, "Humidity Value: %d,%d %%RH", DHT11_Data.humi_int, DHT11_Data.humi_deci);
      sprintf(temp_display_buff, "Temperature Value: %d,%d ℃", DHT11_Data.temp_int, DHT11_Data.temp_deci);
    }
    else
    {
      sprintf(hum_display_buff, "Humidity Value Read Error");
      sprintf(temp_display_buff, "Temperature Value Read Error");
    }

    /* 清除单行文字 */
    LCD_ClearLine(LINE(5));
    LCD_ClearLine(LINE(7));
    LCD_ClearLine(LINE(9));
    // LCD_ClearLine(LINE(9));

    /* 显示该字符串 */
    ILI9341_DispStringLine_EN_CH(LINE(5), pm_display_buff);
    ILI9341_DispStringLine_EN_CH(LINE(7), hum_display_buff);
    ILI9341_DispStringLine_EN_CH(LINE(9), temp_display_buff);

    /*if (DHT11_Read_TempAndHumidity(&DHT11_Data) == SUCCESS)
    {
      printf("\r\n读取DHT11成功!\r\n\r\n湿度为%d.%d ％RH ，温度为 %d.%d℃ \r\n",
             DHT11_Data.humi_int, DHT11_Data.humi_deci, DHT11_Data.temp_int, DHT11_Data.temp_deci);
    }
    else
    {
      printf("Read DHT11 ERROR!\r\n");
    } */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int fputc(int ch, FILE *f)
{
  /* 发???一个字节数据到串口DEBUG_USART */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1000);

  return (ch);
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
