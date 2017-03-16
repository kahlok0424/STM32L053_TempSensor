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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l0xx.h"
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_onewire.h"
//#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_ds18b20.h"
#include "tm_stm32f4_disco.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  /* USER CODE END WHILE */
	  char buf[40];
	  	     uint8_t devices, i, j, count, device[1][8];
	  	     uint8_t alarm_count;
	  	   	uint8_t alarm_device[1][8];
	  	   	uint8_t check = 0 ;
	  	   	float temp[1];
	  	       /* OneWire working structure */
	  	       TM_OneWire_t OneWire;

	  	       /* Initialize system */
	  	       //SystemInit();

	  	       TM_GPIO_Init(GPIOA, 									\
	  	           		  	   GPIO_Pin_5,											\
	  	       				   TM_GPIO_Mode_OUT,									\
	  	       				   TM_GPIO_OType_PP,									\
	  	       				   TM_GPIO_PuPd_NOPULL, 								\
	  	       				   TM_GPIO_Speed_Medium);

	  	       /* Initialize delay */
	  	       TM_DELAY_Init();

	  	       /* Initialize OneWire1 instance on pin PD0 */
	  	       TM_OneWire_Init(&OneWire, Temp_Sensor_Port, Temp_Sensor);

	  	       /* Initialize USART, TX: PB6, RX: PB7 */

	  	      // TM_USART_Init(USART1, TM_USART_PinsPack_2, 115200);
	  	       TM_GPIO_TogglePinValue(GPIOA, GPIO_Pin_5);
	  	       TM_GPIO_TogglePinValue(GPIOA, GPIO_Pin_5);
	  	       TM_GPIO_TogglePinValue(GPIOA, GPIO_Pin_5);
	  	       TM_GPIO_TogglePinValue(GPIOA, GPIO_Pin_5);

	  	       // for testing
	  	    /*   while(1)
	  	            {
	  	          	  TM_GPIO_TogglePinValue(GPIOC, GPIO_Pin_13);
	  	          	  ONEWIRE_DELAY(500000);

	  	          	 TM_GPIO_TogglePinValue(GPIOC, GPIO_Pin_13);
	  	          	 ONEWIRE_DELAY(500000);
	  	            }
	  	 */

	  	       /* Check for any device on 1-wire */
	  	       devices = TM_OneWire_First(&OneWire);
	  	       count = 0;
	  	       while (devices) {
	  	           /* Increase count variable */
	  	           count++;

	  	           /* Get full 8-bytes rom address */
	  	           TM_OneWire_GetFullROM(&OneWire, device[count - 1]);

	  	           /* Check for new device */
	  	           devices = TM_OneWire_Next(&OneWire);
	  	       }


	  	       /* If any devices on 1-wire */
	  	     //  if (count > 0) {
	  	     //      sprintf(buf, "Devices found on 1-wire instance: %d\n", count);
	  	       //    TM_USART_Puts(USART1, buf);

	  	           /* Display 64bit rom code */
	  	      /*     for (j = 0; j < count; j++) {
	  	               for (i = 0; i < 8; i++) {
	  	                   sprintf(buf, "0x%02X ", device[j][i]);
	  	                   TM_USART_Puts(USART1, buf);
	  	               }
	  	               TM_USART_Puts(USART1, "\n");
	  	           }  */
	  	   //    } else {
	  	           /* Nothing on OneWire */
	  	     //      TM_USART_Puts(USART1, "No devices on OneWire.\n\n");
	  	     //  }


	  	       /* Go through all connected devices and set resolution to 12bits */
	  	           for (i = 0; i < count; i++) {
	  	               /* Set resolution to 12bits */
	  	               TM_DS18B20_SetResolution(&OneWire, device[i], TM_DS18B20_Resolution_12bits);
	  	           }

	  	           /* Set high temperature alarm on device number 0, 25 degrees celcius */
	  	           TM_DS18B20_SetAlarmHighTemperature(&OneWire, device[0], 70);

	  	           TM_DS18B20_SetAlarmLowTemperature(&OneWire , device[0], -20);

	  	           /* Disable alarm temperatures on device number 1 */
	  	           TM_DS18B20_DisableAlarmTemperature(&OneWire, device[1]);

	  	           while (1) {
	  	               /* Start temperature conversion on all devices on one bus */
	  	               TM_DS18B20_StartAll(&OneWire);

	  	               /* Wait until all are done on one onewire port */
	  	               while (!TM_DS18B20_AllDone(&OneWire));

	  	               /* Read temperature from ROM address and store it to temps variable */
	  	               TM_DS18B20_Read(&OneWire, device[0], &temp[0]);
	  	               TM_GPIO_TogglePinValue(GPIOC, GPIO_Pin_13);

	  	                   }
  /* USER CODE BEGIN 3 */


  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Temp_Sensor|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Temp_Sensor_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Temp_Sensor|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
