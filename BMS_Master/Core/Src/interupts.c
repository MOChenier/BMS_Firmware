/*
 * interupts.c
 *
 *  Created on: Feb 25, 2025
 *      Author: marco
 */
#include "main.h"
#include "can_bus.h"
#include "misc.h"
#include "stm32f4xx_hal.h"
#include "slave_com.h"

extern uint8_t RxData[8];
extern CAN_RxHeaderTypeDef RxHeader;

extern uint8_t master_mode; // 0: Standby, 1: Charge/Balancing 2: In Operation

extern volatile uint8_t timer_count;

extern volatile uint8_t all_connection_states; // b0:ignition, b1:charger, b2:EmgcyStop
extern volatile uint8_t blinking_HV_led; //0: No blinking, 1: Slow blinking, 2: Fast blinking

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//	slave_return_t return_message;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
//    	Start timer and turn ON Yellow LED
    	HAL_TIM_Base_Start_IT(&htim3);
        HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET); // Turn ON LED

//      Parse and store message according to type
//      If the header is of standard length (like for slave reception message)
        if(RxHeader.IDE == 0){
        	store_received_info(&RxHeader, RxData);
        }

        // Print or process the received message
        printf("Received CAN Message: ");
        for (int i = 0; i < RxHeader.DLC; i++)
        {
            printf("%02X ", RxData[i]);
        }
//        return_message = convert_to_struct(RxData);

        printf("\n");
    }
}

void HAL_CAN_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan)
{
	//    	Start timer and turn ON Yellow LED
	    	HAL_TIM_Base_Start_IT(&htim3);
	        HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET); // Turn ON LED
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


	if (htim->Instance == TIM2) {  // Check if it's the correct timer

    	if((timer_count % HB_10MS_PERIODS) == 0){
    		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin); // Toggle LED
    		printf("Hilo");
    	}

    	if((timer_count % CHARGER_CONNECTION_10MS_PERIODS) == 0){

    	}

    	if((timer_count % ASK_SLAVE_FOR_INFO_10MS_PERIODS) == 0){

    	}


    	// High voltage light blinking and activating logic
    	if((timer_count % SLOW_BLINKING_HV_10MS_PERIODS) == 0)
    	{
    		//  If the precharge contactor is ON, blink the HV light
    		if(blinking_HV_led == 1)
    			HAL_GPIO_TogglePin(LIGHT_GPIO_Port, LIGHT_Pin);
    	}
    	if((timer_count % FAST_BLINKING_HV_10MS_PERIODS) == 0)
    	{
    		//  If the precharge contactor and the main contactor are ON, blink the HV light rapidly
    		if(blinking_HV_led == 2)
    			HAL_GPIO_TogglePin(LIGHT_GPIO_Port, LIGHT_Pin);
    	}

    	// If timer counter has not reached max value, increment
    	((timer_count + 1) % TIMER_MAX) ? timer_count = 0 : timer_count++;
    }


	// Timer that handles the activation time of yellow led on CAN package Rx/Tx
    if (htim->Instance == TIM3) {
        HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_RESET); // Turn off LED
        HAL_TIM_Base_Stop_IT(&htim3);
    }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == IGNITION_Pin){ //Ignition interrupt triggered on both rising and falling edge.

		all_connection_states ^= IGNITION_MASK; //Ignition bit is toggled when edge is detected

	}else if(GPIO_Pin == CHARGER_DETECT_Pin){ //Ignition interrupt triggered on both rising and falling edge.

		all_connection_states ^= IGNITION_MASK; //Charger bit is toggled when edge is detected

	}else if(GPIO_Pin == EMERGENCY_STOP_Pin){

		all_connection_states |= IGNITION_MASK;

		// Interrupt handler for when emergency stop is pressed
		deactivate_main_contactor();
		deactivate_precharge_contactor();

	}


}
