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
#include "can_id_defines.h"
#include "adc.h"
#include "data.h"

static CAN_FilterTypeDef CAN_Filter_Config;
static CAN_RxHeaderTypeDef CAN_Received_0_Message_Header;
static CAN_RxHeaderTypeDef CAN_Received_1_Message_Header;
static CAN_TxHeaderTypeDef dcu_Debug_Packet_Header;
static uint8_t CAN_Received_0_Message_Data[8];
static uint8_t CAN_Received_1_Message_Data[8];
static uint8_t dcu_Debug_Packet_Data[6];

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

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

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin|CAN1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
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
    HAL_GPIO_DeInit(GPIOD, CAN1_RX_Pin|CAN1_TX_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */


extern void CAN1_Start(void)
{
	CAN1_Filter_Setup();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  
  dcu_Debug_Packet_Header.StdId = DCU_DEBUG_ID;
  dcu_Debug_Packet_Header.RTR = CAN_RTR_DATA;
  dcu_Debug_Packet_Header.IDE = CAN_ID_STD;
  dcu_Debug_Packet_Header.DLC = 6;
  dcu_Debug_Packet_Header.TransmitGlobalTime = DISABLE;
  dcu_Debug_Packet_Data[0] = 0;
  dcu_Debug_Packet_Data[1] = 0;
  dcu_Debug_Packet_Data[2] = 0;
  dcu_Debug_Packet_Data[3] = 0;
  dcu_Debug_Packet_Data[4] = 0;
  dcu_Debug_Packet_Data[5] = 0;
}


extern inline void CAN_Set_Dcu_Is_Alive_Packet(void)
{
  dcu_Debug_Packet_Data[5] = 1;
}

extern inline void CAN_Set_Dcu_Is_Not_Alive_Packet(void)
{
  dcu_Debug_Packet_Data[5] = 0;
}

extern inline void CAN_Send_Dcu_Debug_Pakcet(void)
{
  uint32_t dcu_Debug_Packet_Mailbox;
  
  dcu_Debug_Packet_Data[0] = (uint8_t)((DCU_Debug_Temperature >> 8) & 0x00FF);
  dcu_Debug_Packet_Data[1] = (uint8_t)(DCU_Debug_Temperature & 0x00FF);
  dcu_Debug_Packet_Data[2] = (uint8_t)((DCU_Debug_Current >> 8) & 0x00FF);
  dcu_Debug_Packet_Data[3] = (uint8_t)(DCU_Debug_Current & 0x00FF);
  HAL_CAN_AddTxMessage(&hcan1, &dcu_Debug_Packet_Header, dcu_Debug_Packet_Data, &dcu_Debug_Packet_Mailbox);
}


static void CAN1_Filter_Setup(void)
{	
	/*
  EFI
	IDs = 0110000xxxx
	MASK = 0x7F0 (0111 1111 0000) //First 0 is not considered because of bit shift
	Filter = 0x300 (0011 0000 0000)
	*/
  CAN_Filter_Config.FilterBank = 0;
  CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = (0x300 << 5);
  CAN_Filter_Config.FilterIdLow = 0x0000;
  CAN_Filter_Config.FilterMaskIdHigh = (0x7F0 << 5);
  CAN_Filter_Config.FilterMaskIdLow = 0x0000;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_Filter_Config.FilterActivation = ENABLE;	
  CAN_Filter_Config.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Config);	
	
	/*
  DAU
	IDs = 1100101xxxx
	MASK = 0x7F0 (0111 1111 0000) //First 0 is not considered because of bit shift
	Filter = 0x650 (0110 0101 0000)
	*/
  CAN_Filter_Config.FilterBank = 1;
  CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = (0x650 << 5);
  CAN_Filter_Config.FilterIdLow = 0x0000;
  CAN_Filter_Config.FilterMaskIdHigh = (0x7F0 << 5);
  CAN_Filter_Config.FilterMaskIdLow = 0x0000;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_Filter_Config.FilterActivation = ENABLE;	
  CAN_Filter_Config.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Config);	
	
	/*
  IMU + EBB
	IDs = 1110000xxxx
	MASK = 0x7F0 (0111 1111 0000) //First 0 is not considered because of bit shift
	Filter = 0x700 (0111 0000 0000)
	*/
  CAN_Filter_Config.FilterBank = 2;
  CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = (0x700 << 5);
  CAN_Filter_Config.FilterIdLow = 0x0000;
  CAN_Filter_Config.FilterMaskIdHigh = (0x7F0 << 5);
  CAN_Filter_Config.FilterMaskIdLow = 0x0000;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_Filter_Config.FilterActivation = ENABLE;	
  CAN_Filter_Config.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Config);	
	
  //*****FIFO 1 *****//

	/*START ACQ
	ID = 0x7F0
	MASK = 0xFFF
	Filter = 0x7F0
	
	FIFO 1 FILTER 0
	*/
	CAN_Filter_Config.FilterBank = 3;
  CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = (0x7F0 << 5);
  CAN_Filter_Config.FilterIdLow = 0x0000;
  CAN_Filter_Config.FilterMaskIdHigh = (0x7FF << 5);
  CAN_Filter_Config.FilterMaskIdLow = 0x0000;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO1;
  CAN_Filter_Config.FilterActivation = ENABLE;	
  CAN_Filter_Config.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Config);	
	
	/*GCU_CLUTCH_FB_SW_ID (Necessary to reject this message)
  ID: 0x310
	MASK: 0x7FF
	Filter: 0x310
	
	FIFO 1 FILTER 1
	*/
	CAN_Filter_Config.FilterBank = 4;
  CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = (0x310 << 5);
  CAN_Filter_Config.FilterIdLow = 0x0000;
  CAN_Filter_Config.FilterMaskIdHigh = (0x7FF << 5);
  CAN_Filter_Config.FilterMaskIdLow = 0x0000;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO1;
  CAN_Filter_Config.FilterActivation = ENABLE;	
  CAN_Filter_Config.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Config);
	
	/*DEBUG
  IDs: 01100010xxx
	Filter: 0x310
	Mask: 0x7F0
	
	FIFO 1 FILTER 2
	*/
	CAN_Filter_Config.FilterBank = 5;
  CAN_Filter_Config.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_Filter_Config.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_Filter_Config.FilterIdHigh = (0x310 << 5);
  CAN_Filter_Config.FilterIdLow = 0x0000;
  CAN_Filter_Config.FilterMaskIdHigh = (0x7F0 << 5);
  CAN_Filter_Config.FilterMaskIdLow = 0x0000;
	CAN_Filter_Config.FilterFIFOAssignment = CAN_RX_FIFO1;
  CAN_Filter_Config.FilterActivation = ENABLE;	
  CAN_Filter_Config.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter_Config);
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Received_0_Message_Header, CAN_Received_0_Message_Data); 
  data_Conversion(CAN_Received_0_Message_Header.StdId, CAN_Received_0_Message_Data);
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_Received_1_Message_Header, CAN_Received_1_Message_Data);
	
	if(CAN_Received_1_Message_Header.FilterMatchIndex == 0)
	{
		if(CAN_Received_1_Message_Data[1] == START_ACQUISITION_REQUEST)
		{
			start_Acquisition_Request = START_ACQUISITION_REQUEST;
		}      
		
		else if(CAN_Received_1_Message_Data[1] == STOP_ACQUISITION_REQUEST)
		{
			start_Acquisition_Request = STOP_ACQUISITION_REQUEST;
		}
	}
  
  else if(CAN_Received_1_Message_Header.FilterMatchIndex == 2)
  {
    debug_Data_Conversion(CAN_Received_1_Message_Header.StdId, CAN_Received_1_Message_Data);
  }
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
