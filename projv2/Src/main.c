/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "lcd.h"
#include "quadspi.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include"stm32l476g_discovery_glass_lcd.h"
#include"stm32l476g_discovery_qspi.h"
#include<stdlib.h>
#define BUFFER_SIZE         ((uint32_t)0x0028)
#define WRITE_READ_ADDR     ((uint32_t)0x0050)
#define QSPI_BASE_ADDR      ((uint32_t)0x90000000)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile int adc_flag;
uint16_t adc_value;
uint16_t temperature;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
volatile int lcd_flag=0;
volatile int lcd_pomiar=0;
int ind;
int current_ind=1;



uint8_t qspi_aRxBuffer[BUFFER_SIZE];

int _write(int file, char *ptr, int len){

	HAL_UART_Transmit(&huart2, ptr ,len , 50);
	return len;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	if (hadc==&hadc1)
	{
		adc_value=HAL_ADC_GetValue(&hadc1);
		adc_flag=1;
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){

	/* if (HAL_ADC_PollForConversion(&hadc1, 30) == HAL_OK) {
		 adc_value = HAL_ADC_GetValue(&hadc1);
		 HAL_ADC_Start(&hadc1);
		 adc_flag=1;

		 }
*/
	HAL_ADC_Start_IT(&hadc1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	 if(GPIO_Pin == JOY_UP_Pin){
	 HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
	 ++current_ind;
	 current_ind=current_ind%101;
	 if(current_ind==100) current_ind = 1;

	 lcd_flag=1;
	 }
	 if(GPIO_Pin == JOY_DOWN_Pin){
		 HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin);
		 --current_ind;
		 if(current_ind==0)current_ind=100;
		 lcd_flag=1;
		 }
	 if(GPIO_Pin==JOY_CENTER_Pin){
		 lcd_pomiar=1;
	 }

	}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	unsigned short int TS_CAL1=*((volatile unsigned short int*)0x1FFF75A8);
	unsigned short int TS_CAL2=*((volatile unsigned short int*)0x1FFF75CA);

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  BSP_LCD_GLASS_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  HAL_RTC_MspInit (&hrtc);
  QSPI_demo();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  //uint8_t where[3];
  //BSP_QSPI_Read(where, WRITE_READ_ADDR,(uint32_t)0x0003);
  //ind=where[1];
  ind=0;
 __HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
 BSP_LCD_GLASS_ScrollSentence((uint8_t *)"PRESS JOY UP, DOWN OR CENTER TO NAVIGATE          ", 1, SCROLL_SPEED_HIGH);
 //HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */
	  if(adc_flag==1){

		temperature=(110-30)*(adc_value-TS_CAL1)/(TS_CAL2-TS_CAL1)+30;
		 printf("Zmierzono %d, %d ,%d\r\n", TS_CAL1 , TS_CAL2,temperature);
		 adc_flag=0;
		 char data[41];
		 HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
		 HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BIN);
		 printf("%2d: %02d :%02d\r\n", sTime.Hours , sTime.Minutes,sTime.Seconds);
		//BSP_LCD_GLASS_DisplayString((uint8_t *)str);
		 sprintf(data, "TEMP %d DATA %02d-%02d-20%02d GODZ %02d-%02d-%02d    ",(int)temperature, sDate.Date, sDate.Month, sDate.Year, sTime.Hours , sTime.Minutes, sTime.Seconds );

		 if(BSP_QSPI_Erase_Block(WRITE_READ_ADDR+(uint32_t)(0x1000*(ind+1))) != QSPI_OK)
		      {
		        BSP_LCD_GLASS_ScrollSentence((uint8_t *)"      QSPI ERASE : FAILED.", 1, SCROLL_SPEED_HIGH);
		      }
		      else
		      {

		 if(BSP_QSPI_Write((uint8_t *)data, WRITE_READ_ADDR+(uint32_t)(0x1000*(ind+1)), BUFFER_SIZE) != QSPI_OK)
		         {
		           BSP_LCD_GLASS_ScrollSentence((uint8_t *)"      QSPI WRITE : FAILED.", 1, SCROLL_SPEED_HIGH);
		         }
		         else
		         {
		        	 ++ind;
		        	 ind=ind%100;



		           /* Read back data from the QSPI memory */

		         }
		      }
		// char a[]="12";

		//BSP_LCD_GLASS_ScrollSentence((uint8_t *)strin, 1,150);
		//BSP_LCD_GLASS_ScrollSentence((uint8_t *)sTime->Seconds, 1,150);

	  }

	  if(lcd_flag==1){
		  char data1[6];
		  sprintf(data1, "POM%03d",current_ind);
		  BSP_LCD_GLASS_DisplayString((uint8_t*) data1);
		  lcd_flag=0;
	  }
	  //HAL_ADC_Start_IT(&hadc1);
	  //HAL_LCD_Write(&hlcd,LCD_RAM_REGISTER0, LCD_DIGIT3_COM0_SEG_MASK, (uint32_t *)"3")
	  if(lcd_pomiar==1){
      if(BSP_QSPI_Read(qspi_aRxBuffer, WRITE_READ_ADDR+(uint32_t)0x1000*(current_ind), BUFFER_SIZE) != QSPI_OK)
        BSP_LCD_GLASS_ScrollSentence((uint8_t *)"      QSPI READ : FAILED.", 1, SCROLL_SPEED_HIGH);
      else
      BSP_LCD_GLASS_ScrollSentence(qspi_aRxBuffer, 1, SCROLL_SPEED_HIGH);
      lcd_pomiar=0;
	  }

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
