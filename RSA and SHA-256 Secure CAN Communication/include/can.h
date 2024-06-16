/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define FRAME_DATA                        8
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Filter_Config (void);
uint8_t CAN_Transmit_STDMessage(CAN_HandleTypeDef* CANx, uint32_t id, uint8_t* data, uint8_t length);
uint8_t CAN_SendNumberOfBytes (CAN_HandleTypeDef* CANx, uint32_t id, uint8_t* sendData, uint8_t NumOfBytes);
uint8_t CAN_ReceiveUptoEightBytes( CAN_HandleTypeDef* CANx, uint8_t FifoNumber, uint8_t* recData, uint8_t* DataLen);
uint8_t CAN_ReceiveNumberOfBytes ( CAN_HandleTypeDef* CANx, uint8_t FifoNumber, uint8_t* recData, uint8_t NumOfBytes);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

