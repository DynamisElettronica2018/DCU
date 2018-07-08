/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "lwip.h"
#include "rtc.h"
#include "tim.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

// Default value of ETH_RXBUFNB is 4U: it must be icrease to 16U. But the CubeMX can't change this value with any graphical tool. 
// It must change it manually for every compile.

#include "user_defines.h"
#include "telemetry_command.h"
#include "system.h"
#include "usbh_platform.h"
#include "data.h"
#include "user_usb.h"
//#include "user_sd.h"
#include "user_ethernet_udp.h"
#include "bno055.h"
#include "string_utility.h"
#include "can_id_defines.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  MPU_Config();
  CPU_Cache_Enable();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  MX_LWIP_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_GPIO_WritePin(KILL_CTRL_GPIO_Port, KILL_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  MX_DriverVbusHS(USB_VBUS_ENABLE);
  CAN1_Start();
  //CAN3_Start();
  //set_Rtc_Time((uint8_t)18, (uint8_t)45, (uint8_t)0);
  //SD_Check_And_Init();
  UDP_Init();
  initialize_Data();
  BNO055_Init();
  dcu_Debug_Get_Data();
  HAL_Delay(250);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim6);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(start_Acquisition_Request == START_ACQUISITION_REQUEST)
    {
      start_Acquisition_Request = START_ACQUISITION_DONE;

			if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_ERROR)
			{
				USB_Open_File();
				
				if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_OK)
				{
						HAL_TIM_Base_Start_IT(&htim7);
            CAN_Set_Dcu_Is_Alive_Packet();
				}
			}
    }
    
    else if(start_Acquisition_Request == STOP_ACQUISITION_REQUEST)
    {
      start_Acquisition_Request = START_ACQUISITION_DONE;
			
			if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_OK)
			{
        HAL_TIM_Base_Stop_IT(&htim7);
				USB_Close_File();

        if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_ERROR)
        {
            CAN_Set_Dcu_Is_Not_Alive_Packet();
        }
			}
    }
    
  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */
    
    MX_LWIP_Process();
    UDP_Send_Processig();
    UDP_Receive_Processig();
  }
  
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C4
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

//1 Hz
extern inline void TIM6_IRQ_User_Handler(void)
{
  dcu_Debug_Get_Data();
  CAN_Send_Dcu_Debug_Pakcet();
  
  if(dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] == UDP_DCU_STATE_OK)
	{
		UDP_Send_Queue(UDP_DCU_STATE_PORT, dcu_State_Packet, BUFFER_STATE_LEN);
    UDP_Send_Queue(UDP_TELEMETRY_DEBUG_PORT, dcu_Debug_Packet, BUFFER_DEBUG_LEN);
	}
}


//10 Hz
extern inline void TIM5_IRQ_User_Handler(void)
{
  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
  
  if(dcu_State_Packet[DCU_STATE_PACKET_TELEMETRY_ON] == UDP_DCU_STATE_OK)
	{
		if(dcu_State_Packet[DCU_STATE_PACKET_ACQUISITION_ON] == UDP_DCU_STATE_ERROR)
		{
			BNO055_Imu_Read_Data();
			Imu_Dcu_Conversion_To_Buffer();
		}
		
		//make_Data_Average();
		buffer_Telemetry_Pointer = (buffer_Telemetry_Pointer + BUFFER_LEN - 1) % BUFFER_LEN;
		UDP_Send_Queue(UDP_TELEMETRY_DATA_PORT, block_Buffer[buffer_Telemetry_Pointer], BUFFER_BLOCK_LEN);
	}
}


//100 Hz
extern inline void TIM7_IRQ_User_Handler(void)
{
  uint32_To_String(USB_Timestamp, block_Buffer[buffer_Write_Pointer], 7);
  USB_Timestamp += 10;
	BNO055_Imu_Read_Data();
	Imu_Dcu_Conversion_To_Buffer();
	buffer_Write_Pointer = (buffer_Write_Pointer + 1) % BUFFER_LEN;
  USB_Write_Block(block_Buffer[buffer_Read_Pointer]);
	prepare_Next_Buffer_Block(buffer_Read_Pointer, buffer_Write_Pointer);
	buffer_Read_Pointer = (buffer_Read_Pointer + 1) % BUFFER_LEN;
	
	if((USB_Timestamp % CLOSE_FILE_INTERVAL) == 0)
	{
		USB_Close_And_Open_File();
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(HAL_GPIO_ReadPin(USB_OVERCURRENT_GPIO_Port, USB_OVERCURRENT_Pin) == GPIO_PIN_RESET)
  {
    MX_DriverVbusHS(USB_VBUS_DISABLE);
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    dcu_State_Packet[DCU_STATE_PACKET_USB_READY] = UDP_DCU_STATE_ERROR;
    UDP_Send_Error(USB_OVERCURRENT);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
  
  while(1)
  {
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    HAL_Delay(40);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
