/*
 * can_bus.c
 *
 *  Created on: Avril 11, 2025
 *      Author: Alexis (aka: ChatGPT + le code de Marco)
 */

#include "can_bus.h"

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[2];

FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

uint8_t RxDatacheck = 0; // Flag that indicates that a message has been received

int FDCAN1_send_mess(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData)
{
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
	{
		FDCAN_Error_Handler();
	}
	return 0;
}

slave_return_t convert_to_struct(uint8_t RxData[8])
{
	slave_return_t return_message;

	return_message.content_header = RxData[0];

	return_message.cell_states = 0;
	return_message.cell_states |= RxData[1] << 8;
	return_message.cell_states |= RxData[2];

	return_message.min_value_raw = RxData[3];
	return_message.max_value_raw = RxData[4];
	return_message.avg_value_raw = RxData[5];

	return return_message;
}

void FDCAN_set_std_header(uint16_t std_tx_id, uint8_t dlc)
{
	TxHeader.Identifier = std_tx_id;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

void FDCAN_set_ext_header(uint32_t ext_tx_id, uint8_t dlc)
{
	TxHeader.Identifier = ext_tx_id;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

void FDCAN_Error_Handler(void)
{
	__disable_irq();
	while (1)
	{
	}
}

void FDCAN_Filter_Config(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_FilterTypeDef sFilterConfig;

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x000;
	sFilterConfig.FilterID2 = 0x000;

	if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
	{
		FDCAN_Error_Handler();
	}
}

void FDCAN_Activate_Interrupts(FDCAN_HandleTypeDef *hfdcan)
{
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

	if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
	{
		FDCAN_Error_Handler();
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			FDCAN_Error_Handler();
		}
		RxDatacheck = 1;
	}
}

void MX_FDCAN1_Init_Loopback(FDCAN_HandleTypeDef *hfdcan)
{
	hfdcan->Instance = FDCAN1;
	hfdcan->Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
	hfdcan->Init.AutoRetransmission = DISABLE;
	hfdcan->Init.TransmitPause = DISABLE;
	hfdcan->Init.ProtocolException = DISABLE;
	hfdcan->Init.NominalPrescaler = 16;
	hfdcan->Init.NominalSyncJumpWidth = 1;
	hfdcan->Init.NominalTimeSeg1 = 13;
	hfdcan->Init.NominalTimeSeg2 = 2;
	hfdcan->Init.DataPrescaler = 1;
	hfdcan->Init.DataSyncJumpWidth = 1;
	hfdcan->Init.DataTimeSeg1 = 1;
	hfdcan->Init.DataTimeSeg2 = 1;
	hfdcan->Init.StdFiltersNbr = 1;
	hfdcan->Init.ExtFiltersNbr = 0;
	hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

	if (HAL_FDCAN_Init(hfdcan) != HAL_OK)
	{
		FDCAN_Error_Handler();
	}
}
