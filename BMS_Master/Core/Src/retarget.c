/*
 * retarget.c
 *
 *  Created on: Feb 25, 2025
 *      Author: marco
 */


#include "stdio.h"
#include "stdint.h"
#include "stm32f4xx.h"
//#include "usbd_cdc_if.h"

extern UART_HandleTypeDef huart1;


int __io_putchar(int ch)
{
//	uint8_t temp_ch;
//	temp_ch = (uint8_t)ch;
//	CDC_Transmit_FS(&temp_ch, 1);


	// Write character to ITM ch.0
	// ITM_SendChar(ch);
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
	return(ch);
}

