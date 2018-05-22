/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
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
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

#include "user_defines.h"
#include "user_externalVariables.h"
#include "telemetry_command.h"
#include "user_usb.h"
#include "tim.h"
#include "data.h"

CAN_FilterTypeDef CAN_FilterConfig_FIFO0;
CAN_FilterTypeDef CAN_FilterConfig_FIFO1;
CAN_RxHeaderTypeDef CAN_Received0MessageHeader;
CAN_RxHeaderTypeDef CAN_Received1MessageHeader;
uint8_t CAN_Received0MessageData[8];
uint8_t CAN_Received1MessageData[8];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan3;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 20;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* CAN3 init function */
void MX_CAN3_Init(void)
{

  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 20;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = CAN_RX_Pin|CAN_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN3)
  {
  /* USER CODE BEGIN CAN3_MspInit 0 */

  /* USER CODE END CAN3_MspInit 0 */
    /* CAN3 clock enable */
    __HAL_RCC_CAN3_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    /**CAN3 GPIO Configuration    
    PA8     ------> CAN3_RX
    PA15     ------> CAN3_TX 
    */
    GPIO_InitStruct.Pin = CAN_AUX_RX_Pin|CAN_AUX_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_CAN3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN3 interrupt Init */
    HAL_NVIC_SetPriority(CAN3_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN3_TX_IRQn);
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN3_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN3_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN3_SCE_IRQn);
  /* USER CODE BEGIN CAN3_MspInit 1 */

  /* USER CODE END CAN3_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, CAN_RX_Pin|CAN_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN3)
  {
  /* USER CODE BEGIN CAN3_MspDeInit 0 */

  /* USER CODE END CAN3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN3_CLK_DISABLE();
    __HAL_RCC_CAN2_CLK_DISABLE();
    /* Be sure that all peripheral instances that share the same clock need to be disabled */
    /**  HAL_RCC_CAN1_CLK_ENABLED--;
    *  if(HAL_RCC_CAN1_CLK_ENABLED==0){
    *    __HAL_RCC_CAN1_CLK_DISABLE();
    **/
  
    /**CAN3 GPIO Configuration    
    PA8     ------> CAN3_RX
    PA15     ------> CAN3_TX 
    */
    HAL_GPIO_DeInit(GPIOA, CAN_AUX_RX_Pin|CAN_AUX_TX_Pin);

    /* CAN3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN3_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN3_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN3_SCE_IRQn);
  /* USER CODE BEGIN CAN3_MspDeInit 1 */

  /* USER CODE END CAN3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


void CAN1_FilterSetUp(void)
{
  CAN_FilterConfig_FIFO0.FilterBank = 0;
  CAN_FilterConfig_FIFO0.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig_FIFO0.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfig_FIFO0.FilterIdHigh = (0x700 << 5);
  CAN_FilterConfig_FIFO0.FilterIdLow = 0x0000;
  CAN_FilterConfig_FIFO0.FilterMaskIdHigh = (0x700 << 5);
  CAN_FilterConfig_FIFO0.FilterMaskIdLow = 0x0000;
	CAN_FilterConfig_FIFO0.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig_FIFO0.FilterActivation = ENABLE;	
  CAN_FilterConfig_FIFO0.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig_FIFO0);	

	CAN_FilterConfig_FIFO1.FilterBank = 1;
  CAN_FilterConfig_FIFO1.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig_FIFO1.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfig_FIFO1.FilterIdHigh = (0x6EF << 5);
  CAN_FilterConfig_FIFO1.FilterIdLow = 0x0000;
  CAN_FilterConfig_FIFO1.FilterMaskIdHigh = (0x6EF << 5);
  CAN_FilterConfig_FIFO1.FilterMaskIdLow = 0x0000;
	CAN_FilterConfig_FIFO1.FilterFIFOAssignment = CAN_RX_FIFO1;
  CAN_FilterConfig_FIFO1.FilterActivation = ENABLE;	
  CAN_FilterConfig_FIFO1.SlaveStartFilterBank = 15;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig_FIFO1);	
}


void CAN1_Start(void)
{
	HAL_Delay(10);
	CAN1_FilterSetUp();
	HAL_Delay(10);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
}


void CAN1_Init(void)
{
	HAL_CAN_Stop(&hcan1);
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }	

	CAN1_Start();
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{  
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Received0MessageHeader, CAN_Received0MessageData);
  
  if(acquisition_On == UDP_DCU_STATE_OK)
  {
    dataConversion(CAN_Received0MessageHeader.StdId, CAN_Received0MessageData);
  }
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_Received1MessageHeader, CAN_Received1MessageData);
  
  if((CAN_Received1MessageData[1] == 1) && (acquisition_On == UDP_DCU_STATE_ERROR))
  {
    canStartAcquisitionRequest = 1;
  }      
  
  else if((CAN_Received1MessageData[1] == 2) && (acquisition_On == UDP_DCU_STATE_OK))
  {
    canStartAcquisitionRequest = 2;
  }
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
