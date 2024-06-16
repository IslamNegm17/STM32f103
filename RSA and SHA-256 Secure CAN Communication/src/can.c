/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
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

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Config(void){
    CAN_FilterTypeDef canfil;
    
    canfil.FilterBank = 0;
    canfil.FilterMode = CAN_FILTERMODE_IDMASK;
    canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfil.FilterIdHigh = 0;
    canfil.FilterIdLow = 0;
    canfil.FilterMaskIdHigh = 0;
    canfil.FilterMaskIdLow = 0;
    canfil.FilterScale = CAN_FILTERSCALE_32BIT;
    canfil.FilterActivation = ENABLE;
    canfil.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &canfil) != HAL_OK){
        Error_Handler();
		}
}

uint8_t CAN_Transmit_STDMessage(CAN_HandleTypeDef* CANx, uint32_t id, uint8_t* data, uint8_t length){
	  uint32_t canMailbox = 0x00000000; 
	  HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	  if (length > 8){
		   length = 8;
		}
    CAN_TxHeaderTypeDef txHeader;
    txHeader.DLC = length;
    txHeader.StdId = id;		
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.TransmitGlobalTime = DISABLE;
    
		HAL_Status = HAL_CAN_AddTxMessage(CANx, &txHeader, data, &canMailbox);
    
		return HAL_Status;
}

uint8_t CAN_SendNumberOfBytes (CAN_HandleTypeDef* CANx, uint32_t id, uint8_t* sendData, uint8_t NumOfBytes){
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
	  uint8_t FormerArrayIndex = 0;
    CAN_TxHeaderTypeDef txHeader;
	  
    uint8_t Frame_Number = (NumOfBytes / FRAME_DATA);
    if ((NumOfBytes % FRAME_DATA) != 0) {
        Frame_Number++;
    }		

    for (uint8_t itr = 0; itr < Frame_Number; itr++) {
        txHeader.DLC = (NumOfBytes - FormerArrayIndex >= FRAME_DATA) ? FRAME_DATA : (NumOfBytes - FormerArrayIndex);
        while (HAL_CAN_GetTxMailboxesFreeLevel(CANx) == 0);  // Wait until a mailbox is free
         HAL_Status &= CAN_Transmit_STDMessage(CANx, id, &sendData[FormerArrayIndex], txHeader.DLC);
        FormerArrayIndex += FRAME_DATA;
    }

    return HAL_Status;
}

uint8_t CAN_ReceiveUptoEightBytes(CAN_HandleTypeDef* CANx, uint8_t FifoNumber, uint8_t* recData, uint8_t* DataLen){
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    CAN_RxHeaderTypeDef rxHeader;

    while (!HAL_CAN_GetRxFifoFillLevel(CANx, FifoNumber));
    HAL_Status = HAL_CAN_GetRxMessage(CANx, FifoNumber, &rxHeader, recData);

    if (HAL_OK == HAL_Status && DataLen != NULL)
        *DataLen = rxHeader.DLC;

    return HAL_Status;
}

uint8_t CAN_ReceiveNumberOfBytes(CAN_HandleTypeDef* CANx, uint8_t FifoNumber, uint8_t* recData, uint8_t NumOfBytes){
    HAL_StatusTypeDef HAL_Status = HAL_ERROR;
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t FormerArrayIndex = 0;

    uint8_t Frame_Number = (NumOfBytes / FRAME_DATA);
    if ((NumOfBytes % FRAME_DATA) != 0) {
        Frame_Number++;
    }

    for (uint8_t itr = 0; itr < Frame_Number; itr++) {
        while (!HAL_CAN_GetRxFifoFillLevel(CANx, FifoNumber));
        HAL_Status &= HAL_CAN_GetRxMessage(CANx, FifoNumber, &rxHeader, &recData[FormerArrayIndex]);
        FormerArrayIndex += FRAME_DATA;
    }

    return HAL_Status;
}
/* USER CODE END 1 */
