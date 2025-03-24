/*
 * misc.c
 *
 *  Created on: Feb 5, 2025
 *      Author: marco
 */

#include "misc.h"
#include "main.h"

uint8_t emergency_stop = 0;

extern volatile uint8_t blinking_HV_led;

float get_output_current(void)
{

	return 0;
}

void deactivate_main_contactor(void)
{
	//	Deactivate main contactor
	HAL_GPIO_WritePin(MAIN_CONTACTOR_GPIO_Port, MAIN_CONTACTOR_Pin, GPIO_PIN_RESET);

	//  Turn OFF HV light
	HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_RESET);
	blinking_HV_led = 0;

}

void activate_main_contactor(void)
{
	//	Activate main contactor
	HAL_GPIO_WritePin(MAIN_CONTACTOR_GPIO_Port, MAIN_CONTACTOR_Pin, GPIO_PIN_SET);

	if(HAL_GPIO_ReadPin(PRECHARGE_CONTACTOR_GPIO_Port, PRECHARGE_CONTACTOR_Pin) == GPIO_PIN_SET)
		blinking_HV_led = 2; // If precharge is ON, we want to blink the led rapidly (handled in timer interrupts)
	else
		blinking_HV_led = 0; // If only the main contactor is ON, don't blink

	// Turn ON HV light
	HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);


}

void deactivate_precharge_contactor(void)
{
	//	Deactivate main contactor
	HAL_GPIO_WritePin(PRECHARGE_PIN_GROUP, PRECHARGE_PIN, GPIO_PIN_RESET);
	blinking_HV_led = 0;

	// If main contactor is ON, ensures that the HV light is ON
	if(HAL_GPIO_ReadPin(MAIN_CONTACTOR_GPIO_Port, MAIN_CONTACTOR_Pin) == GPIO_PIN_SET)
		HAL_GPIO_WritePin(LIGHT_GPIO_Port, LIGHT_Pin, GPIO_PIN_SET);

}

void activate_precharge_contactor(void)
{
	//	Activate main contactor
	HAL_GPIO_WritePin(PRECHARGE_PIN_GROUP, PRECHARGE_PIN, GPIO_PIN_SET);
	blinking_HV_led = 1; // Set HV light to blinking mode (handled in timer interrupts)

}

