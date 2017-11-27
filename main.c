/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "device.h"
#include "hardwareinit.h"
#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "lcd.h"
#include "uart.h"
#include "sensor.h"
#define Normal	1
#define	Error	5
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

//unsigned char tampung[100];

int waktu=0, detik=0, ms100=0;
//unsigned int sec=0;
unsigned long sek=0;
unsigned char start=0,mainLap1,mainLap2,mainLap3;
int menit=0,iddle,counterSec;
char kata[30];
unsigned char lcd[50];

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
static void MX_TIM2_Init(void);
void PORT(uint16_t counter);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim2)
{
	if(htim2->Instance== TIM2)
	{
		iddle=1;
		tikupdate();
		if(start==1){
			ms100++;
			if(ms100>=10){
				detik++;
				if(detik>=60){
					detik=0;
					menit++;
				}
				ms100=0;
			}
		}
		sprintf(lcd,"%d:%d:%d",menit,detik,ms100);
		//sprintf(kata,"%2d:%2d:%2d\r\n",menit,detik,ms100);
	}
}
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  lcd_init();
  lcd_gotoxy(0,0);
  lcd_clear();
  while (1)
  {
	  //lcd_gotoxy(0,0);
	  //lcd_putstr(lcd);

	  if(iddle)
	  {
	  	//HAL_UART_Transmit(&huart1,kata,10,10);
	  	iddle=0;
	  	lcd_gotoxy(0,0);
	  	lcd_putstr(lcd);
	 }
	if(sensorStartStopDetected())
	{
	  	start^=1;
		lcd_gotoxy(0,0);
		lcd_putstr("start");
	}
	if(sensorResetDetected())
	{

		lcd_gotoxy(0,0);
		lcd_putstr("reset");
		HAL_Delay(1000);
		lcd_clear();
		menit=0;
		detik=0;
		ms100=0;
	}
	/*if(sensor1Detected()==Normal&&start==1)
	{
		mainLap1++;
		sprintf(kata,"Mobil 1 -> lap%d = %2d:%2d:%2d\r\n",mainLap1,menit,detik,ms100);
		HAL_UART_Transmit(&huart1,kata,28,10);
		HAL_GPIO_TogglePin(GPIOC,LEDMERAH_Pin);
		if(mainLap1>=4)
		{
			sprintf(kata,"Mobil 1 -> FINISHED\r\n",mainLap1,menit,detik,ms100);
			HAL_UART_Transmit(&huart1,kata,21,10);
		}
		if(mainLap1>=5)
		{
		}
	}
	else if(sensor1Detected()==Error&&start==1)
	{
		sprintf(kata,"SENSOR 1 error\r\n");
		HAL_UART_Transmit(&huart1,kata,16,10);
		lcd_gotoxy(1,0);
		lcd_putstr("s 1 error");
		HAL_Delay(1500);
	}
	if(sensor2Detected()==Normal&&start==1)
	{
		mainLap1++;
		sprintf(kata,"Mobil 2 -> lap%d = %2d:%2d:%2d\r\n",mainLap1,menit,detik,ms100);
		HAL_UART_Transmit(&huart1,kata,28,10);
		HAL_GPIO_TogglePin(GPIOC,LEDMERAH_Pin);
		if(mainLap1>=4)
		{
			sprintf(kata,"Mobil 2 -> FINISHED\r\n",mainLap1,menit,detik,ms100);
			HAL_UART_Transmit(&huart1,kata,21,10);
		}
		if(mainLap1>=5)
		{
		}
	}
	else if(sensor2Detected()==Error&&start==1)
	{
		sprintf(kata,"SENSOR 2 error\r\n");
		HAL_UART_Transmit(&huart1,kata,16,10);
		lcd_gotoxy(1,0);
		lcd_putstr("s 2 error");
		HAL_Delay(1500);
	}
	if(sensor3Detected()==Normal&&start==1)
	{
		mainLap1++;
		sprintf(kata,"Mobil 3 -> lap%d = %2d:%2d:%2d\r\n",mainLap1,menit,detik,ms100);
		HAL_UART_Transmit(&huart1,kata,28,10);
		HAL_GPIO_TogglePin(GPIOC,LEDMERAH_Pin);
		if(mainLap1>=4)
		{
			sprintf(kata,"Mobil 3 -> FINISHED\r\n",mainLap1,menit,detik,ms100);
			HAL_UART_Transmit(&huart1,kata,21,10);
		}
		if(mainLap1>=5)
		{
		}
	}
	else if(sensor3Detected()==Error&&start==1)
	{
		sprintf(kata,"SENSOR 3 error\r\n");
		HAL_UART_Transmit(&huart1,kata,16,10);
		lcd_gotoxy(1,0);
		lcd_putstr("s 3 error");
		HAL_Delay(1500);
	}*/
  }
}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 200-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void PORT(uint16_t count){
	//Counter Show
	HAL_GPIO_WritePin(GPIOC,RS_Pin,count & (1<<14));
	HAL_GPIO_WritePin(GPIOB,RW_Pin,count & (1<<1));
	HAL_GPIO_WritePin(GPIOC,EN_Pin,count & (1<<15));
	HAL_GPIO_WritePin(GPIOA,D4_Pin,count & (1<<5));
	HAL_GPIO_WritePin(GPIOA,D5_Pin,count & (1<<6));
	HAL_GPIO_WritePin(GPIOA,D6_Pin,count & (1<<7));
	HAL_GPIO_WritePin(GPIOA,D7_Pin,count & (1<<8));
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
