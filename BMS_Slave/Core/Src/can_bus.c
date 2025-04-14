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


osMessageQueueId_t canRxQueueHandle;

void can_send_Task(){

	FDCAN_setup_std_header();

	for(;;){

		//if (osSemaphoreAcquire(reg_pullHandle, osWaitForever) == osOK){

			FDCAN_set_header(TempertureCanID, 6);

			CAN_Temps.Temp1 = 12.0;
			CAN_Temps.Temp2 = 13.0;
			CAN_Temps.Temp3 = 14.0;

    		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, TempertureCanID, &CAN_Temps);

    		//osSemaphoreRelease(reg_pullHandle);

			// Yellow LED
			//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
			vTaskDelay(100);
    	//}


	}

}

void can_rcv(){

	canRxQueueHandle = osMessageQueueNew(8, sizeof(CANMessage_t), NULL);

	//for (;;)
	//{

        CANMessage_t msg;
        HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &msg.header, msg.data);

		if (osMessageQueueGet(canRxQueueHandle, &msg, NULL, osWaitForever) == osOK)
		{
			// Process the received CAN message
		}
		// Red LED
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		vTaskDelay(100);
	//}

}


HAL_StatusTypeDef SendRegisterGroupFDCAN(RegisterGroup* regData)
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t txData[64] = {0};
/*
    // Fill out FDCAN TX header
    txHeader.Identifier          = 0x321;                   // Change as needed
    txHeader.IdType              = FDCAN_STANDARD_ID;
    txHeader.TxFrameType         = FDCAN_DATA_FRAME;
    txHeader.DataLength          = FDCAN_DLC_BYTES_64;      // DLC = 64 bytes
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch       = FDCAN_BRS_ON;
    txHeader.FDFormat            = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker       = 0;
*/
    // Copy struct to buffer (only 55 bytes used)
    memcpy(txData, regData, sizeof(RegisterGroup));

    // Add message to Tx FIFO
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, txData);
}

int FDCAN1_send_mess(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData)
{
	if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData) != HAL_OK)
	{
		//FDCAN_Error_Handler();
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

void FDCAN_setup_std_header()
{
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

void FDCAN_set_header(uint16_t std_tx_id, uint8_t dlc)
{
	TxHeader.Identifier = std_tx_id;
	TxHeader.DataLength = FDCAN_DLC_BYTES_6;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        CANMessage_t msg;
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &msg.header, msg.data) == HAL_OK)
        {
            osMessageQueuePut(canRxQueueHandle, &msg, 0, 0);
        }
    }
}





