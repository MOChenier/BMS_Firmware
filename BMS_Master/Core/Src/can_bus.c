/*
 * can_bus.c
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */

#include "can_bus.h"


CAN_TxHeaderTypeDef	TxHeader;
uint8_t		TxData[2];
uint32_t	TxMailbox;

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

uint8_t RxDatacheck = 0; //Flag that indicates that a message has been received

int CAN1_send_mess(CAN_HandleTypeDef *hcan, uint8_t TxData[TxDLC])
{

    CAN_init();

	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
	   CAN_Error_Handler ();
	}


	return 0;
}

slave_return_t convert_to_struct(uint8_t RxData[8])
{
	slave_return_t return_message;

	return_message.content_header = RxData[0];

	return_message.cell_states = 0; //initialize cell states
	return_message.cell_states |= RxData[1] << 8;
	return_message.cell_states |= RxData[2];

	return_message.min_value_raw = RxData[3];

	return_message.max_value_raw = RxData[4];

	return_message.avg_value_raw = RxData[5];

	return return_message;

}


// Following functions are used for configuration


void CAN_init()
{

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = TxID;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = TxDLC;

}


void CAN_Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void CAN_Filter_Config(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        CAN_Error_Handler();
    }
}

void CAN_Activate_Interrupts(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_EPV) ||
        __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_BOF))
    {
        printf("CAN in error state, attempting reset\n");
        HAL_CAN_Stop(hcan);
        HAL_CAN_Init(hcan);
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        CAN_Error_Handler();
    }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
void MX_CAN1_Init_Loopback(CAN_HandleTypeDef *hcan)
{

  /* USER CODE BEGIN CAN1_Init 0 */
//	__HAL_RCC_CAN1_CLK_ENABLE();
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan->Instance = CAN1;
  hcan->Init.Prescaler = 16;
  hcan->Init.Mode = CAN_MODE_LOOPBACK;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = DISABLE;
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = DISABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(hcan) != HAL_OK)
  {
    CAN_Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

